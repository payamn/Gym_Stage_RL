#include "ros/ros.h"
#include <ros/package.h>

#include <follower_rl/agent_num.h>
#include <std_srvs/Empty.h>
//#include <mutex>          // std::mutex
//#include <iostream>
//#include <X11/Xlib.h>
//#include <mutex>
#include <boost/thread/thread.hpp>

int agent_num = 0;
boost::mutex mtx;
bool handle_agent_numbering(follower_rl::agent_num::Request  &req,
         follower_rl::agent_num::Response &res)
{
	ROS_INFO("handle_agent_numbering!");
	mtx.lock();
	res.num = agent_num;
	agent_num += 1;
	mtx.unlock();
	return true;
}



int main(int argc, char **argv)
{
	ros::init( argc, argv, "manager_node");
	ros::NodeHandle* n = new ros::NodeHandle();
	ros::ServiceServer reset_random_srv_ = n->advertiseService("agent_num", handle_agent_numbering);
	ros::spin();
}
