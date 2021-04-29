#ifndef GAZEBO_TRAJECTORY
#define GAZEBO_TRAJECTORY

#include <ros/ros.h>
#include "arm_catch_pkg/TrajectoryPredict.h"
#include "gazebo_msgs/ModelStates.h"
#include <cstdlib>

class GazeboTrajectorySubscriber{
public:
	GazeboTrajectorySubscriber(ros::NodeHandle &n);
	void trajectoryCB(const gazebo_msgs::ModelStates &msg);

protected:
	ros::ServiceClient _client;
	ros::Subscriber _sub;
	ros::Publisher _pub;
	bool published;
};



#endif