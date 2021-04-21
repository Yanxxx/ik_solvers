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
#include <queue>
#include <sys/times.h>
#include <unistd.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/JointState.h>

#include "udp_client.h"
#include "udp_server.h"
#include <cmath>

#define NO_OF_JOINTS 6
#define T	30	//target position update period(ms)
#define nInterpolationPoints		6	// interpolation points between two target
#define CMD_Q_MAX		12		//max depth of command q(now choose twice of nInterpolationPoints)


using namespace KDL; 
using namespace std; 

bool new_target = false;
bool robot_control_flag = false;
//geometry_msgs::Twist target;

Chain * p_chain;
ChainFkSolverPos_recursive* p_fksolver;

geometry_msgs::Transform target;
geometry_msgs::Transform robot_pose;
sensor_msgs::JointState joint_state;
int print_count;

#define UDP_SERVER_IP "127.0.0.2"
#define UDP_SERVER_PORT 19001
#define UDP_CLIENT_IP UDP_SERVER_IP
#define UDP_CLIENT_PORT UDP_SERVER_PORT - 1

const char* j_name_list[]={
"shoulder_joint",
"upperArm_joint",
"foreArm_joint",
"wrist1_joint",
"wrist2_joint",
"wrist3_joint"
};

JntArray current_position(6);
JntArray current_velocity(6);

JntArray robot_position(6);
JntArray robot_velocity(6);

JntArray target_joint_pose(6);

JntArray ik_result(6);

queue<JntArray> qcmd;

int sync_robot_pose_flag = 0;

void *udpclient(void *t) {
	(void)t;
	udp_client uc(UDP_CLIENT_IP, UDP_CLIENT_PORT);
	char buff[1024] = "";
	double position[6] = {0};
	JntArray send_pose(6);

	double acc_max = 0.7;
	double vel_max = 0.7;

	double error = 0;
	int i;
	int sign = 1;
	double vel = 0;
	double joint_error = 0;

	double joint_errors[6] = {0};
	double joint_step[6] = {0};

	double temp = 0;

	int send_flag = 0;

	std::cout<<"start udp sending"<<std::endl;
  	while (1) {
		if(!robot_control_flag){
			if(sync_robot_pose_flag>2500){
				// std::cout<<"sync current position with robot postion"<<std::endl;
				for(i=0;i< NO_OF_JOINTS ;i++){
					if(abs(current_velocity(i)-robot_velocity(i))>0.05)
						current_velocity(i) = robot_velocity(i);
					if(abs(current_position(i)-robot_position(i))>0.05)
						current_position(i) = robot_position(i);
				}
			}
			usleep(5000);
			continue;
		}
		// std::cout<<"position to move"<<std::endl;
		// for (i = 0; i<NO_OF_JOINTS;i++){
		// 	joint_errors[i] = target_joint_pose(i) - current_position(i);
		// 	std::cout<<joint_errors[i]<<" ";
		// }
		// std::cout<<std::endl;
		//std::cout<<"send out positions "<<std::endl;

		send_flag = 0;

		// remark: reset step size need to be reset. smooth requriments 

		for(i=0;i< NO_OF_JOINTS ;i++){
			joint_errors[i] = target_joint_pose(i)-current_position(i);
			// std::cout<<joint_errors[i]<<" ";
			// joint_step[i] = joint_errors[i] / 7;
			if(abs(joint_errors[i]) < 0.01){
				send_flag+=1;
				continue;
			}else{
				if(abs(joint_errors[i])<0.7){
					position[i] = target_joint_pose(i);
				}else{
					if(joint_errors[i] > 0.7) temp = 0.7;
					if(joint_errors[i] < -0.7) temp = -0.7;
					current_position(i) += temp;
					position[i] = current_position(i);
				}
			}
		}

		std::cout<<"send flag: "<<send_flag<<std::endl;

		if(send_flag < 6){
			

			std::cout<<"Sent position: "<<std::endl;
			for(i = 0; i < 6; i++){
				//position[i] = send_pose(i);
				std::cout<<position[i]<<" ";
			}
			std::cout<<std::endl;
			
			// std::cout<<"ik target pose"<<std::endl;
			// for(i = 0; i < 6; i++){
			// 	std::cout<<ik_result(i)* 180 / 3.1415926<<" ";
			// 	}
			// 	std::cout<<std::endl;
				
			memcpy(buff, position, sizeof(double) * 6);
			uc.send(buff, sizeof(double) * 6);
		}

		//std::cout<<"udp sending package"<<std::endl;

    	usleep(5000);//need to adjust
	}
}


void *udpserver(void *t) {
  (void)t;
  	std::cout<<"init udp server receiving"<<std::endl;
  udp_server us(UDP_SERVER_IP, UDP_SERVER_PORT);
  	std::cout<<"udp server receiving initiated"<<std::endl;
  char buff[1024];

  double recv_data[12] = {0};
  
  	std::cout<<"start udp receiving"<<std::endl;

  print_count = 0;
  while (1) {
	robot_control_flag = false;
    us.timed_recv(buff, 1024, 50);
    memcpy(recv_data, buff, sizeof(double) * 12);
    int i;
	robot_control_flag = true;

    for(i = 0; i < 6; i++){
    	//current_velocity(i) = recv_data[i];//*3.1415926/180;//*180/3.1415926;
    	//current_position(i) = recv_data[i+6];//*3.1415926/180;//*180/3.1415926;

    	robot_velocity(i) = recv_data[i];//*180/3.1415926;
    	robot_position(i) = recv_data[i+6];//*180/3.1415926;
    }
    
    if(!new_target){
  		for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
  				joint_state.position[i] = robot_position(i)*3.1415926/180;
  				// joint_state.position[i] = qcmd.front()(i);;
  			}
  		joint_state.header.stamp = ros::Time::now();
  		//jointstates_publisher.publish(joint_state);
  
		
  
//    std::cout << "publish robot status to simulator: " ;//<< position[0] << std::endl;
    
//  for(int i=0;i<6;i++){
//      std::cout<<joint_state.position[i]<<" ";
//  }
//  std::cout<<endl;
  	}
    ////<< position[0] << std::endl;
  
	if(print_count>=50){	
		std::cout << "original received messages: " ;
		for(int i=0;i<6;i++){
			std::cout<<recv_data[i+6]<<" ";
		}
		std::cout<<endl;
		print_count=0;
	}
	print_count++;
  //  std::cout << "received messages: " ;//<< position[0] << std::endl;
    
  //for(int i=0;i<6;i++){
//      std::cout<<current_position(i)<<" ";
  //}
//  std::cout<<endl;
  }
}


void Joint_State_Msg_Initialize(int size, char* joint_name_list[]){
    int i;
    joint_state.name.resize(size);
    joint_state.position.resize(size);
    for(i = 0; i < size; i++)
        joint_state.name[i] = joint_name_list[i];
}
/*
void posmsgCallback(const geometry_msgs::Twist::ConstPtr&  msg)
{
	new_target = true;
	target.linear.x = msg->linear.x;
	target.linear.y = msg->linear.y;
	target.linear.z = msg->linear.z;

	target.angular.x = msg->angular.x;
	target.angular.y = msg->angular.y;
	target.angular.z = msg->angular.z;

	//ROS_INFO("x:[%f] y:[%f] z:[%f]", target.linear.x, target.linear.y, target.linear.z);
	//ROS_INFO("tx:[%f] ty:[%f] tz:[%f]", target.angular.x, target.angular.y, target.angular.z);
}
*/
void posmsgCallback(const geometry_msgs::Transform::ConstPtr&  msg)
{
	new_target = true;
	/*
	target.translation.x = msg->translation.x + robot_pose.translation.x;
	target.translation.y = msg->translation.y + robot_pose.translation.y;
	target.translation.z = msg->translation.z + robot_pose.translation.z;
	
	target.translation.x = msg->translation.x;// + robot_pose.translation.x;
	target.translation.y = msg->translation.y;// + robot_pose.translation.y;
	target.translation.z = msg->translation.z;// + robot_pose.translation.z;

	target.rotation.x = msg->rotation.x;
	target.rotation.y = msg->rotation.y;
	target.rotation.z = msg->rotation.z;
	target.rotation.w = msg->rotation.w;
		target.rotation.x = x;
	target.rotation.y = y;
	target.rotation.z = z;
	target.rotation.w = w;
	*/
	
// 	Rotation r1,r2, r3;
// 	r1.Quaternion(msg->rotation.x,msg->rotation.y,msg->rotation.z,msg->rotation.w);
// 	r2.Quaternion(robot_pose.rotation.x,robot_pose.rotation.y,robot_pose.rotation.z,robot_pose.rotation.w);
	
// 	r3 = r2 * r1;
	
//   double x,y,z,w;
//   r3.GetQuaternion(x,y,z,w);  
  
//   std::cout<<"quaternion: x:"<<x<<"y: "<<y<<"z: "<<z<<"w: "<<w<<std::endl;
  
//   std::cout<<"translation: x:"<<msg->translation.x<<"y: "<<msg->translation.y<<"z: "<<msg->translation.z<<std::endl;
  	Frame eeFrame;
	p_fksolver->JntToCart(current_position, eeFrame);
	
	robot_pose.translation.x = eeFrame.p[0];
	robot_pose.translation.y = eeFrame.p[1];
	robot_pose.translation.z = eeFrame.p[2];
	
	// double x,y,z,w;
	// eeFrame.M.GetQuaternion(x,y,z,w);  
	
	// robot_pose.rotation.x = x;
	// robot_pose.rotation.y = y;
	// robot_pose.rotation.z = z;
	// robot_pose.rotation.w = w;

	target.rotation.x = msg->rotation.x;
	target.rotation.y = msg->rotation.y;
	target.rotation.z = msg->rotation.z;
	target.rotation.w = msg->rotation.w;
	
	
	target.translation.x = msg->translation.x + robot_pose.translation.x;
	target.translation.y = msg->translation.y + robot_pose.translation.y;
	target.translation.z = msg->translation.z + robot_pose.translation.z;
	
	
//	target.translation.x = robot_pose.translation.x;
//	target.translation.y = robot_pose.translation.y;
//	target.translation.z = robot_pose.translation.z;


std::cout<<"delta pose"<<std::endl;

	ROS_INFO("x:[%f] y:[%f] z:[%f]", msg->translation.x, msg->translation.y, msg->translation.z);
	ROS_INFO("qx:[%f] qy:[%f] qz:[%f], qw[%f]", msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w);

	
std::cout<<"target pose"<<std::endl;
	ROS_INFO("x:[%f] y:[%f] z:[%f]", target.translation.x, target.translation.y, target.translation.z);
	ROS_INFO("qx:[%f] qy:[%f] qz:[%f], qw[%f]", target.rotation.x, target.rotation.y, target.rotation.z, target.rotation.w);
	
std::cout<<"robot pose"<<std::endl;
	ROS_INFO("x:[%f] y:[%f] z:[%f]", robot_pose.translation.x, robot_pose.translation.y, robot_pose.translation.z);
	ROS_INFO("qx:[%f] qy:[%f] qz:[%f], qw[%f]", robot_pose.rotation.x, robot_pose.rotation.y, robot_pose.rotation.z, target.rotation.w);
}

double Bound(double angle){
	while(angle>3.1415926||angle <-3.1415926){
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
	Joint_State_Msg_Initialize(NO_OF_JOINTS,(char**)j_name_list);

	Tree my_tree;
	kdl_parser::treeFromFile("/home/yan/catkin_ws/src/aubo_description/urdf/aubo_i3.urdf",my_tree);

	//chain = new Chain();
	
	udp_client uc(UDP_CLIENT_IP, UDP_CLIENT_PORT);
	
	Chain chain;
	
	my_tree.getChain("world","wrist3_Link",chain);
	//ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

	p_chain = &chain;
	p_fksolver = &fksolver;

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
	ros::Publisher jointstates_publisher = pnode->advertise<sensor_msgs::JointState>("joint_states", 1000);
	while(ros::ok()){
  		if(!new_target){
  			ros::spinOnce();
  			sync_robot_pose_flag+=1;
  			jointstates_publisher.publish(joint_state);
  			continue;
  		}

		sync_robot_pose_flag=0;

  		vec.x(target.translation.x);
  		vec.y(target.translation.y);
  		vec.z(target.translation.z);

  		rot = Rotation::Quaternion(
  				target.rotation.x,
				target.rotation.y,
				target.rotation.z,
				target.rotation.w
  				);

  		Frame TargetFrame(rot,vec);


  		begin = clock();
  		kinematics_status = iksolver.CartToJnt(q_last, TargetFrame, jointpositions);
		TrimJoint(jointpositions);
		// Frame eeFrame;
		// p_fksolver->JntToCart(jointpositions, eeFrame);
		
		// robot_pose.translation.x = eeFrame.p[0];
		// robot_pose.translation.y = eeFrame.p[1];
		// robot_pose.translation.z = eeFrame.p[2];

		// std::cout<<"target cart pose: "<< eeFrame.p[0]<<" " <<eeFrame.p[1]<<" " <<eeFrame.p[2]<<std::endl;

  		end = clock();
  		//cout<<"time last:"<< end-begin<<endl;
  		// publish as joint_state
  		if(kinematics_status >= 0){
			robot_control_flag = true;
			std::cout<<"target pose is: " <<std::endl;

  			for(unsigned int i = 0; i < NO_OF_JOINTS ;i++){
  				q_last(i) = jointpositions(i);
  				ik_result(i) = jointpositions(i);

  				joint_state.position[i] = jointpositions(i);
				target_joint_pose(i) = jointpositions(i) * 180 / 3.1415926;
				std::cout<<target_joint_pose(i)<<" ";
  				//joint_state.position[i] = qcmd.front()(i);;
  			}
			std::cout<<std::endl;
  			joint_state.header.stamp = ros::Time::now();
  			jointstates_publisher.publish(joint_state);
  			//printf("ik solver succeed!\r\n");
  			//generate_interpolation(jointpositions);
  		}
  		else
  			printf("ik solver failed!\r\n");
  		new_target = false;
  		ros::spinOnce();
  	}
}

int main(int argc,char** argv){
	ros::init(argc, argv, "ik_solver");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle node_handle;

	//ros::Publisher jointstates_publisher = n.advertise<sensor_msgs::JointState>("joint_cmd", 1000);

	ros::Subscriber tarpos_sub = node_handle.subscribe("tarpos_pub", 1000, posmsgCallback);


	// create and start threads for udp communication
	pthread_t server_thread;
	pthread_t client_thread;
	pthread_t ik_thread;
	pthread_attr_t attr;

	// Initialize and set thread joinable		
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	int st = 1;
	int ct = 2;
	
	std::cout<<"start thread"<<std::endl;
	
	pthread_create(&server_thread, &attr, udpserver, (void *)&st);
	pthread_create(&client_thread, &attr, udpclient, (void *)&ct);
	pthread_create(&ik_thread, &attr, ik_fun, (void *)&node_handle);
	pthread_attr_destroy(&attr);
	void *status;
	pthread_join(server_thread, &status);
	pthread_join(client_thread, &status);
	pthread_join(ik_thread, &status);
	pthread_exit(NULL);




}

