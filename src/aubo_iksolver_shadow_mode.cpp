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
	my_tree.getChain("world","tcp_Link",reduced_aubo_i3_arm_chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(reduced_aubo_i3_arm_chain);
	// std::cout<<chain<<std::endl;

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
	bool kinematics_status;
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

		fksolver.JntToCart(_current_position, eeFrame);
		target.translation.x = _delta_pose.translation.x + eeFrame.p[0];
		target.translation.y = _delta_pose.translation.y + eeFrame.p[1];
		target.translation.z = _delta_pose.translation.z + eeFrame.p[2];
		vec.x(target.translation.x);
		vec.y(target.translation.y);
		vec.z(target.translation.z);

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
		kinematics_status = iksolver.CartToJnt(_current_position, TargetFrame, jointpositions);
		TrimJoint(jointpositions);		
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