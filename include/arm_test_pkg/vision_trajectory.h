#ifndef VISION_TRAJECTORY
#define VISION_TRAJECTORY

#include <ros/ros.h>
#include "arm_catch_pkg/TrajectoryPredict.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>
#include "Eigen/Dense"
#include <cmath>
#include "arm_test_pkg/ball_trajectory_predict.h"

class VisionTrajectorySubscriber{
public:
	VisionTrajectorySubscriber(ros::NodeHandle &n);
	void trajectoryCB(const geometry_msgs::Pose &msg);

protected:
	ros::Subscriber _sub;
	ros::Publisher _pub;
	bool _published;
	bool _prev_pose_exists;
	ros::Time _prev_pose_time;
	geometry_msgs::Pose _prev_pose;
};



#endif