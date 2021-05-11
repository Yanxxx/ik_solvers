#include <kdl/kdl.hpp> 
#include <kdl/chain.hpp> 
#include <kdl/tree.hpp> 
#include <kdl/segment.hpp> 
#include <kdl/chainfksolver.hpp> 
#include <kdl_parser/kdl_parser.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp> 
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames_io.hpp> 
#include <kdl/utilities/error.h>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include "kdl_conversions/kdl_msg.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_kdl.h"

#include <stdio.h> 
#include <iostream> 
#include <queue>
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "udp_client.h"
#include "udp_server.h"
#include <cmath>
#include <vector>


using namespace KDL; 
using namespace std; 

#define NO_OF_JOINTS 4
#define UDP_SERVER_IP "127.0.0.2"
#define UDP_SERVER_PORT 19001
#define UDP_CLIENT_IP UDP_SERVER_IP
#define UDP_CLIENT_PORT UDP_SERVER_PORT - 1

bool new_target = false;
bool program_terminated = false;
int print_count = 0;
int link_index[]={2,1,0,0};
double link_length[] = {0,0,0,0};

geometry_msgs::Transform target;
geometry_msgs::Transform _delta_pose;

sensor_msgs::JointState joint_state;
ros::NodeHandle * _p_node;

JntArray _current_position(NO_OF_JOINTS);
JntArray current_velocity(NO_OF_JOINTS);
JntArray robot_position(NO_OF_JOINTS);
JntArray robot_velocity(NO_OF_JOINTS);
JntArray target_joint_pose(NO_OF_JOINTS);

const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
};

const char* joint_position_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
};

void *udpserver(void * t) {
	(void) t;
	std::cout<<"init udp server receiving"<<std::endl;
	udp_server us(UDP_SERVER_IP, UDP_SERVER_PORT);
	std::cout<<"udp server receiving initiated"<<std::endl;
	char buff[1024];

	double recv_data[12] = {0};
	double c_p[6] = {0};
	double error = 0;
	int current_pose_sync_flag = 0;

	sensor_msgs::JointState robot_joint_states;

	robot_joint_states.name.resize(NO_OF_JOINTS);
	robot_joint_states.position.resize(NO_OF_JOINTS);
	for(int i = 0; i < NO_OF_JOINTS; i++){
		robot_joint_states.name[i] = j_name_list[i];
	}

	std::cout<<"start udp receiving"<<std::endl;
	ros::Publisher robot_joint_states_publisher = 
		_p_node->advertise<sensor_msgs::JointState>("robot2/joint_states", 1000);

	print_count = 0;
	int recv=0;
	std::cout<<"start udp receiving *************"<<std::endl;
	while (!program_terminated) {
		recv = us.timed_recv(buff, 1024, 50);
		memcpy(recv_data, buff, sizeof(double) * 12);

		for(int i = 0; i < NO_OF_JOINTS; i++){
			_current_position(i) = recv_data[i+6] * 3.14 / 180;
			robot_joint_states.position[i] = _current_position(i);
		}
		
		robot_joint_states.header.stamp = ros::Time::now();
		robot_joint_states_publisher.publish(robot_joint_states);

		if(print_count>=500){	
			std::cout << "original received messages: " ;
			for(int i=0;i<NO_OF_JOINTS;i++){
				std::cout<<recv_data[i+6]<<" ";
			}
			std::cout<<endl;
			print_count=0;
		}
		print_count++;
	}
	pthread_exit(NULL);
}

void Joint_State_Msg_Initialize(int size, char* joint_name_list[]){
	int i;
	joint_state.name.resize(size);
	joint_state.position.resize(size);
	for(i = 0; i < size; i++)
		joint_state.name[i] = joint_name_list[i];
}

void posmsgCallback(const geometry_msgs::Transform::ConstPtr&  msg)
{
	new_target = true;

	_delta_pose.translation.x = msg->translation.x;
	_delta_pose.translation.y = msg->translation.y;
	_delta_pose.translation.z = msg->translation.z;

	_delta_pose.rotation.x = msg->rotation.x;
	_delta_pose.rotation.y = msg->rotation.y;
	_delta_pose.rotation.z = msg->rotation.z;
	_delta_pose.rotation.w = msg->rotation.w;	

	std::cout<< "delta distance: "<< msg->translation.x<<" ";
	std::cout<< msg->translation.y<<" ";
	std::cout<< msg->translation.z<<std::endl;
}

double Bound(double angle){
	while(angle>3.1415926||angle < -3.1415926){
		if(angle>M_PI) angle -=2*M_PI;
		if(angle<-M_PI) angle +=2*M_PI;
	}
	return angle;
}

void TrimJoint(JntArray& joints){
	int i = 0;
	for(i =0;i<NO_OF_JOINTS;i++){
		joints(i) = Bound(joints(i));
	}
}

double l1 = link_length[0];
double l2 = link_length[1];
double l3 = link_length[2];
double l4 = link_length[3];

int AnalyticIK(Vector target_position, Rotation quaternion, JntArray& joint_position){
	double segment_2_3 = 0.1;

	double x = target_position[0];
	double y = target_position[1];
	double z = target_position[2] - l1;


	cout<< " link length, l1: " << l1 << " l2: " << l2 << " l3: " << l3 << " l4: " << l4 << endl;

	double xy = sqrt(x*x + y*y - l2*l2);

	double j1 = atan2(y,x) - atan2(xy,l2);

	cout<< "virtual sway: " << atan2(y,x) << " arm triagnle: " << atan2(xy,l2) << endl;

	double a = xy * xy + z * z;
	double b = l3 * l3 - l4 * l4 + a;

	double criterion = b*b+4*a*l3*l3;
	if (criterion >= 0){
		criterion = sqrt(criterion);
		double x1 = (xy * b + z * criterion) / 2 / a;
		double x2 = (xy * b - z * criterion) / 2 / a;

		double y1 = (z * b - xy * criterion) / 2 / a;
		double y2 = (z * b + xy * criterion) / 2 / a;
		
		cout<< "ee position: x: " << xy << " y: " << z << endl;

		cout<< "solved joint results: x1: " << x1 
		<< " x2: " << x2 << " y1: " << y1 << " y2: " << y2 << endl;

		double j2_tmp_1 = atan2(y1,x1);
		double j2_tmp_2 = atan2(y2,x2);

		double j3_tmp_1 = atan2((z - y1), (xy - x1)) - j2_tmp_1;
		double j3_tmp_2 = atan2((z - y2), (xy - x2)) - j2_tmp_2;

		cout<< "solved joint results: j2_1: " << j2_tmp_1 
		<< " j2_2: " << j2_tmp_2 << " j3_1: " << j3_tmp_1 << " j3_2: " << j3_tmp_2 << endl;

		double j2 = 0 , j3 = 0, j4 = 0;

		if (j2_tmp_1 >= -M_PI_2 && j2_tmp_1 <= M_PI_2 
		&& j3_tmp_1 >= 0 && j3_tmp_1 <= M_PI_2){
			j2 = j2_tmp_1;
			j3 = j3_tmp_1;
		}
		if (j2_tmp_2 >= 0 && j2_tmp_2 <= M_PI_2 
		&& j3_tmp_2 >= 0 && j3_tmp_2 <= M_PI){
			j2 = j2_tmp_2;
			j3 = j3_tmp_2;
		}

		joint_position(0) = j1;
		joint_position(1) = j2;
		joint_position(2) = j3;
		joint_position(3) = j4;

		return 0;
	}
	else{
		double j2 = atan2(xy,z);
		double j3 = 0;
		double j4 = 0;

		joint_position(0) = j1;
		joint_position(1) = j2;
		joint_position(2) = j3;
		joint_position(3) = j4;
		return 0;
	}

}

int ForwardKinematic(vector<double>& translation){
	double j1 = _current_position(0);
	double j2 = _current_position(1);
	double j3 = _current_position(2);

	double xy = l3 * sin(j2) + l4 * sin(j2 + j3);
	double z = l3 * cos(j2) + l4 * cos(j2 + j3) + l1;

	double x = sqrt(xy * xy + l2 * l2) * cos(j1 + atan2(xy, l2));
	double y = sqrt(xy * xy + l2 * l2) * sin(j1 + atan2(xy, l2));

	translation[0] = x;
	translation[1] = y;
	translation[2] = z;
}

void *ik_fun(void *t) {
	ros::NodeHandle *pnode = (ros::NodeHandle *)t;
	_p_node = pnode;
	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);

	Tree my_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_3R.urdf", my_tree);

	Tree reduced_aubo_i3_arm_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_reduced.urdf", reduced_aubo_i3_arm_tree);

	Chain chain;
	my_tree.getChain("world","tcp_Link",chain);

	Chain reduced_aubo_i3_arm_chain;
	my_tree.getChain("world","wrist1_link",reduced_aubo_i3_arm_chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(reduced_aubo_i3_arm_chain);
	// std::cout<<chain<<std::endl;
	
	for (int i = 1; i < 5; i ++){
		auto link = chain.getSegment(i).getJoint();
		auto origin = link.JointOrigin();
		link_length[i-1] = origin[link_index[i-1]];
		// cout<< origin[0]<<" "<< origin[1]<<" "<< origin[2]<<" "<<endl;
		cout<<link_length[i - 1]<<endl;
	}

	l1 = link_length[0];
	l2 = link_length[1];
	l3 = link_length[2];
	l4 = link_length[3];

	unsigned int nj = chain.getNrOfJoints();
	unsigned int ns = chain.getNrOfSegments();
	printf("kinematics information: nj=%d, ns=%d\n",nj,ns);
	JntArray jointpositions = JntArray(nj);
	JntArray cur_jointpositions = JntArray(nj);
	Frame cartpos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);

	int pre_robot_receive_count = 0;
	int robot_pose_sync_flag = 0;


	struct tms begin_tms,end_tms;
	clock_t begin,end;
	int kinematics_status;
	double position[6]={0};
	position[4] = 90;
	position[5] = 0;
	char buff[1024] = "";
	udp_client uc(UDP_CLIENT_IP, UDP_CLIENT_PORT);

	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// construct the destination frame
	Vector vec(0,0,0);
	Rotation rot(0,0,0,0,0,0,0,0,0);
	ros::Publisher jointstates_publisher = 
		pnode->advertise<sensor_msgs::JointState>("robot1/joint_states", 1000);
	
	Frame eeFrame;

	std::vector<double> current_cart_position(3,0); 

	while(ros::ok()){
		ros::spinOnce();
		if(!new_target){
			continue;
		}

		std::cout<<"current pose: ";
		for(int i = 0; i< NO_OF_JOINTS;i++){
			std::cout<<_current_position(i)* 180 / 3.1415926<<" ";
		}
		std::cout<<std::endl;

		// fksolver.JntToCart(_current_position, eeFrame);
		// target.translation.x = _delta_pose.translation.x + eeFrame.p[0];
		// target.translation.y = _delta_pose.translation.y + eeFrame.p[1];
		// target.translation.z = _delta_pose.translation.z + eeFrame.p[2];
		// cout<< "current position: x: " << eeFrame.p[0] <<" y: "<< eeFrame.p[1] <<" z: " << eeFrame.p[2] <<endl;
		// cout<< "target position: x: " << vec[0] <<" y: "<< vec[1] <<" z: " << vec[2] <<endl;

		ForwardKinematic(current_cart_position);
		target.translation.x = _delta_pose.translation.x + current_cart_position[0];
		target.translation.y = _delta_pose.translation.y + current_cart_position[1];
		target.translation.z = _delta_pose.translation.z + current_cart_position[2];
		vec.x(target.translation.x);
		vec.y(target.translation.y);
		vec.z(target.translation.z);

		cout<< "current position: x: " << current_cart_position[0] <<" y: "<< current_cart_position[1] <<" z: " << current_cart_position[2] <<endl;
		cout<< "target position: x: " << vec[0] <<" y: "<< vec[1] <<" z: " << vec[2] <<endl;

		rot = Rotation::Quaternion(
				_delta_pose.rotation.x,
				_delta_pose.rotation.y,
				_delta_pose.rotation.z,
				_delta_pose.rotation.w);
	
		Frame TargetFrame(rot,vec);		
		// TargetFrame.M.DoRotX(-M_PI_2);
		// TargetFrame.M.DoRotX(-M_PI);
		// TargetFrame.M.DoRotY(M_PI);
		TargetFrame.M.DoRotZ(M_PI);

		begin = clock();
		// kinematics_status = iksolver.CartToJnt(_current_position, TargetFrame, jointpositions);

		kinematics_status = AnalyticIK(vec, TargetFrame.M, jointpositions);

		cout << "kinematic status: "<< kinematics_status 
		<< " joint 1: " << jointpositions(0) << " joint 2: " << jointpositions(1) 
		<< " joint 3: " << jointpositions(2) << " joint 4: " << jointpositions(3) << endl;

		//TrimJoint(jointpositions);		
		end = clock();

		if(kinematics_status >= 0){
			// std::cout<<"target pose is: ";
			for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
				joint_state.position[i] = jointpositions(i);					
				_current_position(i) = jointpositions(i);			
				position[i]  = jointpositions(i) * 180 / 3.1415926;	
				// std::cout<<position[i]<<" ";
			}			
			// std::cout<<std::endl;
			memcpy(buff, position, sizeof(double) * 6);
			uc.send(buff, sizeof(double) * 6);
			joint_state.header.stamp = ros::Time::now();
			jointstates_publisher.publish(joint_state);
		}
		else{
			printf("ik solver failed!\r\n");
		}
		new_target = false;
	}
	pthread_exit(NULL);
}

int main(int argc,char** argv){
	ros::init(argc, argv, "ik_solver");

	ros::start(); 
	ros::NodeHandle node_handle;
	ros::Subscriber tarpos_sub = 
		node_handle.subscribe("tarpos_pub", 1000, posmsgCallback);

	// create and start threads for udp communication
	pthread_t server_thread;
	pthread_t ik_thread;
	pthread_attr_t attr;

	// Initialize and set thread joinable		
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	
	std::cout<<"start thread"<<std::endl;	
	pthread_create(&server_thread, &attr, udpserver, NULL);
	pthread_create(&ik_thread, &attr, ik_fun, (void *)&node_handle);
	pthread_attr_destroy(&attr);
	void *status;
	pthread_join(server_thread, &status);
	pthread_join(ik_thread, &status);
	pthread_exit(NULL);

	std::cin.get();
	program_terminated = true;
	ros::shutdown();
}