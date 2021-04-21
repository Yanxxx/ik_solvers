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

#include <stdio.h> 
#include <iostream> 
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>

#define NO_OF_JOINTS 4

using namespace KDL; 
using namespace std; 

bool new_target = false;
geometry_msgs::Transform target;
sensor_msgs::JointState joint_state;


const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
};
/*


const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
"wrist2_joint",
"wrist3_joint"
};
*/
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
	target.translation.x = msg->translation.x;
	target.translation.y = msg->translation.y;
	target.translation.z = msg->translation.z;

	// target.angular.x = msg->angular.x;
	// target.angular.y = msg->angular.y;
	// target.angular.z = msg->angular.z;
	target.rotation.x = msg->rotation.x;
	target.rotation.y = msg->rotation.y;
	target.rotation.z = msg->rotation.z;
	target.rotation.w = msg->rotation.w;	

	std::cout<<"received control message"<<std::endl;
	// ROS_INFO("x:[%f] y:[%f] z:[%f]", target.linear.x, target.linear.y, target.linear.z);
	// ROS_INFO("tx:[%f] ty:[%f] tz:[%f]", target.angular.x, target.angular.y, target.angular.z);
}

int main(int argc,char** argv){
	ros::init(argc, argv, "ik_solver");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	//ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_cmd", 1000);
	ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);

	ros::Subscriber tarpos_sub = n.subscribe("tarpos_pub", 1000, posmsgCallback);

	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);

	Tree my_tree; 
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3_3R.urdf",my_tree);

	Chain chain; 
	my_tree.getChain("world","tcp_Link",chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);


	unsigned int nj = chain.getNrOfJoints(); 
	unsigned int ns = chain.getNrOfSegments();
	printf("kinematics information: nj=%d, ns=%d\n",nj,ns);
	JntArray jointpositions = JntArray(nj); 
	JntArray qz(nj);
	JntArray q_last(nj);
	Frame cartpos;
	//used for time statistic
	int sc_clk_tck = sysconf(_SC_CLK_TCK);
	struct tms begin_tms,end_tms;
	clock_t begin,end;
	bool kinematics_status;


	for(unsigned int i=0;i<nj;i++){

		qz(i)=0;
		q_last(i) = 0;
	}

	//inverse kinematics
	//initialize the solver
	double eps = 1e-5;
	double eps_joints = 1e-15;
	int maxiter = 500;
	ChainIkSolverPos_LMA iksolver(chain,eps,maxiter,eps_joints);
	// construct the destination frame
	Vector vec(0,0,0);
	Rotation rot(0,0,0,0,0,0,0,0,0);

	while(ros::ok()){
		if(!new_target){
			ros::spinOnce();
			continue;
		}
		vec.x(target.translation.x);
		vec.y(target.translation.y);
		vec.z(target.translation.z);

		// rot = Rotation::RPY(target.angular.x, target.angular.y, target.angular.z);

  		rot = Rotation::Quaternion(
  				target.rotation.x,
				target.rotation.y,
				target.rotation.z,
				target.rotation.w);

		Frame TargetFrame(rot,vec);

		TargetFrame.M.DoRotZ(M_PI/2);

		begin = clock();
		kinematics_status = iksolver.CartToJnt(q_last, TargetFrame, jointpositions);
		
		std::cout 	<<"ik success:"<<kinematics_status<<endl
					<< jointpositions(0) << ","
					<< jointpositions(1) << ","
					<< jointpositions(2) << std::endl;
		if(jointpositions(1) < -1.57){
			q_last = qz;
			
			cout<<"bad configuration! Reset value..."<<endl;
			ros::spinOnce();
			continue;
		}
			


		end = clock();
		if(kinematics_status >= 0){
			for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
				
				std_msgs::Float64 num;
				num.data = jointpositions(i);
				q_last(i) = jointpositions(i);

				joint_state.position[i] = jointpositions(i);
			}
			joint_state.header.stamp = ros::Time::now();
			jointstates_publisher.publish(joint_state);
			//printf("ik solver succeed!\r\n");


		}
		else
			printf("ik solver failed!\r\n");
		new_target = false;
		ros::spinOnce();
	}


}

