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
#include <iomanip>      // std::setprecision


using namespace KDL; 
using namespace std; 

#define NO_OF_JOINTS 6
#define UDP_SERVER_IP "127.0.0.2"
#define UDP_SERVER_PORT 19001
#define UDP_CLIENT_IP UDP_SERVER_IP
#define UDP_CLIENT_PORT UDP_SERVER_PORT - 1

// define robot working space 
#define X_LOWER_BOUND 0.1
#define X_UPPER_BOUND 0.6
#define Y_LOWER_BOUND -0.6
#define Y_UPPER_BOUND 0.4
#define Z_LOWER_BOUND -0.1
#define Z_UPPER_BOUND 0.4


bool new_target = false;
bool program_terminated = false;
int print_count = 0;
int link_index[]={2,1,0,0};
double link_length[] = {0,0,0,0};

ChainFkSolverPos_recursive* _robot_fk_solver;
ros::Publisher* simulator_joint_publisher;

geometry_msgs::Transform target;
geometry_msgs::Transform _delta_pose;

sensor_msgs::JointState joint_state;
ros::NodeHandle * _p_node;

udp_client* _udp_angle_publsher;

JntArray _current_position(NO_OF_JOINTS);
JntArray current_velocity(NO_OF_JOINTS);
JntArray robot_position(NO_OF_JOINTS);
JntArray robot_velocity(NO_OF_JOINTS);
JntArray target_joint_pose(NO_OF_JOINTS);
Rotation _rotation(1,0,0,0,1,0,0,0,1);
Vector _position(0,0,0);
double _yaw = M_PI / 4;//0;

const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
"wrist2_joint",
"wrist3_joint",
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
			_current_position(i) = recv_data[i+6] * M_PI / 180;
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

	// std::cout<< "delta distance: "<< msg->translation.x<<" ";
	// std::cout<< msg->translation.y<<" ";
	// std::cout<< msg->translation.z<<std::endl;
}

void orimsgCallback(const geometry_msgs::Transform::ConstPtr&  msg)
{
	char buff[128] = {0};
	_delta_pose.rotation.x = msg->rotation.x;
	_delta_pose.rotation.y = msg->rotation.y;
	_delta_pose.rotation.z = msg->rotation.z;
	_delta_pose.rotation.w = msg->rotation.w;	

	_rotation = Rotation::Quaternion(
				_delta_pose.rotation.x,
				_delta_pose.rotation.y,
				_delta_pose.rotation.z,
				_delta_pose.rotation.w);
	double position[6] = {0};
	double roll=0, pitch=0, yaw=0;
	_rotation.GetRPY(roll, pitch, yaw);

	cout << "Controller input angle: \t" << roll << endl;

	cout << "conditions: "  << (roll > -M_PI * 2 / 3 && roll <= 0) 
			<< " " << ((roll < 0 && roll <= -M_PI * 2 / 3) || (roll > 0 && roll >= M_PI * 2 / 3))
			<< " " << (roll < M_PI * 2 / 3 && roll >= 0) << endl;

	if (roll > -M_PI * 2 / 3 && roll <= 0) {
		roll = M_PI / 4;
	}
	else if ((roll < 0 && roll <= -M_PI * 2 / 3) || (roll > 0 && roll >= M_PI * 2 / 3)) {
		roll = 0;
	}
	else if (roll < M_PI * 2 / 3 && roll >= 0){
		roll = -M_PI / 4;
	} 

	cout << "Discrete angle: \t" << roll << endl;

	double joint4_zero_point = _current_position(2) + _current_position(1) - M_PI;
	_current_position(3) = roll + joint4_zero_point;
	joint_state.position[3] = roll + joint4_zero_point;

	for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
		position[i]  = _current_position(i) * 180 / M_PI;	
	}

	std::cout<< "Orientation Joint Positions:\t";
	for(int i = 0; i< NO_OF_JOINTS;i++){
		std::cout << setprecision(3) << "\tjoint "<< i + 1 <<": " << position[i];
	}
	std::cout<<std::endl;

	// memcpy(buff, position, sizeof(double) * 6);
	// _udp_angle_publsher->send(buff, sizeof(double) * 6);
	// joint_state.header.stamp = ros::Time::now();
	// simulator_joint_publisher->publish(joint_state);
	// Frame eeFrame;
	// _robot_fk_solver->JntToCart(_current_position, eeFrame);
	// eeFrame.M.GetRPY(roll, pitch, yaw);
	_yaw = roll;
}

double Bound(double angle){
	while(angle>M_PI||angle < -M_PI){
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

int InBound(Vector& position){
	if (position[0] <= X_LOWER_BOUND) position[0] = X_LOWER_BOUND;
	if (position[0] >= X_UPPER_BOUND) position[0] = X_UPPER_BOUND;
	if (position[1] <= Y_LOWER_BOUND) position[1] = Y_LOWER_BOUND;
	if (position[1] >= Y_UPPER_BOUND) position[1] = Y_UPPER_BOUND;
	if (position[2] <= Z_LOWER_BOUND) position[2] = Z_LOWER_BOUND;
	if (position[2] >= Z_UPPER_BOUND) position[2] = Z_UPPER_BOUND;

	return 0;
}

int INFO(string title, double data[]){
	return 0;
}

void *ik_fun(void *t) {
	ros::NodeHandle *pnode = (ros::NodeHandle *)t;
	_p_node = pnode;
	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);

	Tree my_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_soft_restrict.urdf", my_tree);

	Tree reduced_aubo_i3_arm_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_reduced.urdf", reduced_aubo_i3_arm_tree);

	Chain chain;
	my_tree.getChain("world","tcp_Link",chain);

	Chain reduced_aubo_i3_arm_chain;
	my_tree.getChain("world","tcp_Link",reduced_aubo_i3_arm_chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	_robot_fk_solver = &fksolver;
	// std::cout<<chain<<std::endl;
	
	for (int i = 1; i < 5; i ++){
		auto link = chain.getSegment(i).getJoint();
		auto origin = link.JointOrigin();
		link_length[i-1] = origin[link_index[i-1]];
		cout<<link_length[i - 1]<<endl;
	}

	l1 = link_length[0];
	l2 = link_length[1];
	l3 = link_length[2];
	l4 = link_length[3];

	unsigned int nj = chain.getNrOfJoints();
	unsigned int ns = chain.getNrOfSegments();
	printf("kinematics information: nj=%d, ns=%d\n",nj,ns);
	nj = reduced_aubo_i3_arm_chain.getNrOfJoints();
	ns = reduced_aubo_i3_arm_chain.getNrOfSegments();
	printf("***kinematics information: nj=%d, ns=%d***\n",nj,ns);
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
	_udp_angle_publsher = &uc;

	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// iksolver.display_information = 1;
	// construct the destination frame
	Vector vec(0,0,0);
	Rotation rot(0,0,0,0,0,0,0,0,0);
	Rotation ee_yaw(1,0,0,0,1,0,0,0,1);

	Rotation ee_base(1,0,0,0,1,0,0,0,1);

	ros::Publisher jointstates_publisher = 
		pnode->advertise<sensor_msgs::JointState>("robot1/joint_states", 1000);
	simulator_joint_publisher = &jointstates_publisher;

	Frame eeFrame;

	std::vector<double> current_cart_position(3,0); 

	while(ros::ok()){
		ros::spinOnce();
		if(!new_target){
			continue;
		}

		fksolver.JntToCart(_current_position, eeFrame);

		vec[0] = _delta_pose.translation.x + eeFrame.p[0];
		vec[1] = _delta_pose.translation.y + eeFrame.p[1];
		vec[2] = _delta_pose.translation.z + eeFrame.p[2];
		
		cout << setprecision(3) << "delta position:\t\tx: " << _delta_pose.translation.x <<
		"\t\ty: "<< _delta_pose.translation.y <<
		"\t\tz: " << _delta_pose.translation.z <<endl;

		cout << setprecision(3) << "current position:\tx: " << eeFrame.p[0] <<
		"\t\ty: "<< eeFrame.p[1] <<
		"\t\tz: " << eeFrame.p[2] <<endl;

		cout << setprecision(3) << "target position:\tx: " << vec[0] <<
		"\t\ty: "<< vec[1] <<
		"\t\tz: " << vec[2] <<endl;

		InBound(vec);
		cout << setprecision(3) << "Inbound position:\tx: " << vec[0] <<
		"\t\ty: "<< vec[1] <<
		"\t\tz: " << vec[2] <<endl;

		rot = Rotation::Identity();
		rot.DoRotX(M_PI);
		rot.DoRotZ(-M_PI);
		ee_yaw = Rotation::Identity();
		ee_yaw.DoRotX(_yaw);

		// TargetFrame.M.DoRotX(M_PI);
		// TargetFrame.M.DoRotZ(-M_PI_2);

		begin = clock();
		// kinematics_status = iksolver.CartToJnt(_current_position, TargetFrame, jointpositions);

		jointpositions(4) = M_PI_2;
		jointpositions(5) = 0;
		// fksolver.JntToCart(jointpositions, eeFrame);
		// double r = 0, p = 0, y = 0;

		ee_base = Rotation::Identity();
		double target_angle = atan2(vec[1],vec[0]) - acos(l2 / sqrt(vec[0]*vec[0]+vec[1]*vec[1]));
		ee_base.DoRotZ(target_angle);
		rot = ee_yaw * rot;
		rot = ee_base  * rot; 
		// Frame ftf(rot, eeFrame.p);
		Frame TargetFrame(rot, vec);
		kinematics_status = iksolver.CartToJnt(_current_position, TargetFrame, jointpositions);

		std::cout<<"kinematic status:  "<< kinematics_status << endl;
		std::cout<< "IK Joint Positions:\t";
		// kinematics_status = iksolver.CartToJnt(_current_position, TargetFrame, jointpositions);

		for(int i = 0; i< NO_OF_JOINTS;i++){
			std::cout << setprecision(3) << "\tjoint "<< i + 1 <<": " << jointpositions(i) * 180 / M_PI;
		}
		std::cout<<std::endl;


		std::cout<< "Current Joint Positions:";
		for(int i = 0; i< NO_OF_JOINTS;i++){
			std::cout << setprecision(3) << "\tjoint "<< i + 1 <<": " << _current_position(i) * 180 / M_PI;
		}
		std::cout<<std::endl;

		//TrimJoint(jointpositions);
		end = clock();

		if(kinematics_status >= 0){
			// std::cout<<"target pose is: ";
			for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
				joint_state.position[i] = jointpositions(i);					
				_current_position(i) = jointpositions(i);			
				position[i]  = jointpositions(i) * 180 / M_PI;	
				// std::cout<<position[i]<<" ";
			}

			std::cout<< "Target Joint Positions:\t";
			for(int i = 0; i< NO_OF_JOINTS;i++){
				std::cout << setprecision(3) << "\tjoint "<< i + 1 <<": " << position[i];
			}
			std::cout<<std::endl;
			//joint_state.position[5] = 0;
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
	ros::Subscriber tarori_sub = 
		node_handle.subscribe("tarori_pub", 1000, orimsgCallback);

	// init robot start position
	_current_position(0) = 0;
	_current_position(1) = 0;
	_current_position(2) = M_PI_2;
	_current_position(3) = 0;
	_current_position(4) = M_PI_2;
	_current_position(5) = 0;


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
