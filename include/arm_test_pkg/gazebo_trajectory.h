#ifndef GAZEBO_TRAJECTORY
#define GAZEBO_TRAJECTORY

#include <ros/ros.h>
#include "arm_catch_pkg/TrajectoryPredict.h"
#include "gazebo_msgs/ModelStates.h"
#include <cstdlib>
#include "Eigen/Dense"
#include <cmath>
#include "arm_test_pkg/ball_trajectory_predict.h"

class GazeboTrajectorySubscriber{
public:
	GazeboTrajectorySubscriber(ros::NodeHandle &n);
	void trajectoryCB(const gazebo_msgs::ModelStates &msg);

protected:
	ros::Subscriber _sub;
	ros::Publisher _pub;
	bool published;
};



#endif