/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/plan_commander/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace plan_commander {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv),
	isConnected(false)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"plan_commander");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	state_publisher = n.advertise<planner_msgs::Mapbuilder>("/map_builder_cmd",5);
	pose_sub = n.subscribe<planner_msgs::Mapbuilder>("/plan_pose_recv",100,&QNode::pose_callback, this);
	isConnected = true;
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"plan_commander");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
	start();
	state_publisher = n.advertise<planner_msgs::Mapbuilder>("/map_builder_cmd",5);
	pose_sub = n.subscribe<planner_msgs::Mapbuilder>("/plan_pose_recv",100,&QNode::pose_callback, this);
	isConnected = true;
	return true;
}

void QNode::send_transition(std::string str,unsigned int id_,float value1, float value2, float value3){

  if(isConnected)
  {
      planner_msgs::Mapbuilder msg;
      msg.id = id_;
      msg.state = str;
      msg.val1 = value1;
      msg.val2 = value2;
      msg.val3 = value3;
      state_publisher.publish(msg);
  }
}
void QNode::pose_callback(const planner_msgs::Mapbuilder::ConstPtr &msg)
{

     pose_recv_msg = *msg;
     poseInfoUpdated();

}

void QNode::run() {
	ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {
	    ros::spinOnce();
	    loop_rate.sleep();
	/*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);		
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	*/
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace plan_commander
