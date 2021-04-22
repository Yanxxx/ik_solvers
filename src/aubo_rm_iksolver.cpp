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
#include <unistd.h>
#include <memory>

using namespace KDL; 
using namespace std; 

#define NO_OF_JOINTS 4
#define UDP_SERVER_IP "127.0.0.2"
#define UDP_SERVER_PORT 19001
#define UDP_CLIENT_IP UDP_SERVER_IP
#define UDP_CLIENT_PORT UDP_SERVER_PORT - 1

// global variables 
const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
};

JntArray _current_position(NO_OF_JOINTS);

sensor_msgs::JointState _joint_state;
Chain * _p_chain;
ros::Publisher* _p_publisher;

std::shared_ptr<ros::Publisher> _p_robot_state_publisher;
ros::Publisher * _p_ik_state_publisher;

std::shared_ptr<ChainFkSolverPos_recursive> _p_fksolver;
std::shared_ptr<ChainIkSolverPos_LMA> _p_iksolver;
std::shared_ptr<udp_client> _p_uc;
std::shared_ptr<ros::NodeHandle> _p_node;



bool _program_terminated = false;

// functions 
void *udpserver(void *t) {
    (void)t;
  	std::cout<<"init udp server receiving"<<std::endl;
    udp_server us(UDP_SERVER_IP, UDP_SERVER_PORT);
  	std::cout<<"udp server receiving initiated"<<std::endl;
    char buff[1024];
    double recv_data[12] = {0};
    int print_count = 0;
    int recv=0;
    int i;

	sensor_msgs::JointState robot_joint_states;

	robot_joint_states.name.resize(NO_OF_JOINTS);
	robot_joint_states.position.resize(NO_OF_JOINTS);
	for(int i = 0; i < NO_OF_JOINTS; i++){
		robot_joint_states.name[i] = j_name_list[i];
	}

	std::cout<<"start udp receiving"<<std::endl;
	ros::Publisher robot_joint_states_publisher =
		_p_node->advertise<sensor_msgs::JointState>("robot2/joint_states", 1000);

  	std::cout<<"start udp receiving"<<std::endl;
    while (!_program_terminated) {
        recv = us.timed_recv(buff, 1024, 50);
		if(recv >= 0){
			memcpy(recv_data, buff, sizeof(double) * 12);

			for(i = 0; i < NO_OF_JOINTS; i++){
				_current_position(i) = recv_data[i+6] * 3.14 / 180;
				robot_joint_states.position[i] = recv_data[i+6] * 3.14 / 180;
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
    }
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

int iksolver_fun(geometry_msgs::Transform target, JntArray& joint_positions){	
    Vector vec(0,0,0);
	Rotation rot(0,0,0,0,0,0,0,0,0);

    vec.x(target.translation.x);
    vec.y(target.translation.y);
    vec.z(target.translation.z);

    rot = Rotation::Quaternion(
            target.rotation.x,
            target.rotation.y,
            target.rotation.z,
            target.rotation.w);

    Frame TargetFrame(rot,vec);
    TargetFrame.M.DoRotX(-M_PI);
    TargetFrame.M.DoRotY(M_PI);

	std::cout<<"curent position: ";
	for (int i = 0; i< NO_OF_JOINTS; i++) {
		std::cout<< _current_position(i) <<" ";
	}
	std::cout<< std::endl;
	// std::cout<< TargetFrame<<std::endl;

    int kinematics_status = _p_iksolver->CartToJnt(_current_position, TargetFrame, joint_positions);

	// std::cout<< "kdl results " << kinematics_status<< std::endl;

	// std::cout<<"target position: ";
	// for (int i = 0; i< NO_OF_JOINTS; i++) {
	// 	std::cout<< joint_positions(i) <<" ";
	// }
	// std::cout<< std::endl;
    TrimJoint(joint_positions);
    return kinematics_status + 101;
}

void Joint_State_Msg_Initialize(int size, char* joint_name_list[]){
    int i;
    _joint_state.name.resize(size);
    _joint_state.position.resize(size);
    for(i = 0; i < size; i++)
        _joint_state.name[i] = joint_name_list[i];
}

void posmsgCallback(const geometry_msgs::Transform::ConstPtr&  msg)
{
	// new_target = true;
	Frame eeFrame;
    geometry_msgs::Transform target;
    JntArray joint_positions(NO_OF_JOINTS);
	_p_fksolver->JntToCart(_current_position, eeFrame);	
	
	target.rotation.x = msg->rotation.x;
	target.rotation.y = msg->rotation.y;
	target.rotation.z = msg->rotation.z;
	target.rotation.w = msg->rotation.w;	
	
	target.translation.x = msg->translation.x + eeFrame.p[0];
	target.translation.y = msg->translation.y + eeFrame.p[1];
	target.translation.z = msg->translation.z + eeFrame.p[2];

	std::cout<< "delta distance: "<< msg->translation.x<<" ";
	std::cout<< msg->translation.y<<" ";
	std::cout<< msg->translation.z<<std::endl;

    double position[6] = {0,0,0,0,90,0};
    char buff[128]={0};

    // ik computation
    int kinematics_status = iksolver_fun(target, joint_positions);
	std::cout<<"ik results "<<kinematics_status<<std::endl;

    // send ik results
    if (kinematics_status >= 0){
        std::cout<<"target pose is: ";
        for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
            position[i]  = joint_positions(i) * 180 / 3.1415926;	
			_current_position(i) = joint_positions(i);
			_joint_state.position[i] = joint_positions(i);
            std::cout<<position[i]<<" ";
        }			
        memcpy(buff, position, sizeof(double) * 6);
        _p_uc->send(buff, sizeof(double) * 6);
		std::cout<< "publish robot1 topic"<<std::endl;
		_p_ik_state_publisher->publish(_joint_state);
    }
}

void *RefreshRosMsg(void *t){
	(void ) t;
	while(ros::ok()){
		ros::spinOnce();
	}
}

int main(int argc,char** argv){
    // init ros 
	ros::init(argc, argv, "ik_solver");
	ros::start(); 
	// ros::NodeHandle node_handle;
	_p_node = std::make_shared<ros::NodeHandle>();
	ros::Subscriber tarpos_sub = _p_node->subscribe("tarpos_pub", 1000, posmsgCallback);


    // init ros node
	// ros::NodeHandle *pnode = &node_handle;
	// Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);
    _joint_state.name.resize(NO_OF_JOINTS);
    _joint_state.position.resize(NO_OF_JOINTS);
    for(i = 0; i < NO_OF_JOINTS; i++)
        _joint_state.name[i] = j_name_list[i];

    // init robot tree and chain
	Tree my_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_3R.urdf",my_tree);
	Chain chain;
	
	my_tree.getChain("world","tcp_Link",chain);

    
    // init fk solver
	// ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	_p_chain = &chain;
	// _p_fksolver = &fksolver;
	// _p_fksolver = new ChainFkSolverPos_recursive(chain);

	_p_fksolver = std::make_shared<ChainFkSolverPos_recursive>(chain);

	unsigned int nj = chain.getNrOfJoints();
	unsigned int ns = chain.getNrOfSegments();
	printf("kinematics information: nj=%d, ns=%d\n",nj,ns);
	JntArray jointpositions = JntArray(nj);
	JntArray cur_jointpositions = JntArray(nj);
	Frame cartpos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);

    // init ik solver
	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	// ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// std::cout << "my tree " << my_tree << std::endl;
	// std::cout << "chain " << chain << std::endl;
	// std::cout << "eps = " << eps <<std::endl;
	// std::cout << "eps_joints = " << eps_joints << std::endl;
	// std::cout << "maxiter = " << maxiter << std::endl;
    // _p_iksolver = &iksolver;

	_p_iksolver = std::make_shared<ChainIkSolverPos_LMA>(chain, eps, maxiter, eps_joints);

	// _p_ik_state_publisher = std::make_shared<ros::Publisher>();
	// _p_ik_state_publisher.Reset(&_p_node->advertise<sensor_msgs::JointState>("robot1/joint_states", 1000));

	// _p_iksolver = new ChainIkSolverPos_LMA(chain, eps, maxiter, eps_joints);

	// construct the destination frame
	Vector vec(0,0,0);
	Rotation rot(0,0,0,0,0,0,0,0,0);
    
    // init joint state publisher
	ros::Publisher pub = 
		// _p_node->advertise<sensor_msgs::JointState>("robot1/joint_states", 1000);
		_p_node->advertise<sensor_msgs::JointState>("robot1/joint_states", 1000);

	//std::shared_ptr<ros::Publisher> jointstates_publisher(pub);
    // _p_publisher = &jointstates_publisher;
	_p_ik_state_publisher = &pub;


	tf::TransformBroadcaster br;
	tf::Transform t1;

	// create and start threads for udp communication
	pthread_t server_thread;
	pthread_t ros_thread;
	pthread_attr_t attr;

	// Initialize and set thread joinable		
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	//int st = 1;
	std::cout<<"start thread"<<std::endl;	
	pthread_create(&server_thread, &attr, udpserver, NULL);
	pthread_create(&ros_thread, &attr, RefreshRosMsg, NULL);
    // init pose sending process

	// udp_client uc(UDP_CLIENT_IP, UDP_CLIENT_PORT);
    // _p_uc = &uc;

	_p_uc = std::make_shared<udp_client>(UDP_CLIENT_IP, UDP_CLIENT_PORT);

	sleep(1);

	pthread_attr_destroy(&attr);
	void *status;
	pthread_join(server_thread, &status);
	pthread_exit(NULL);

	std::cin.get(); 
	_program_terminated = true;
	ros::shutdown();
}