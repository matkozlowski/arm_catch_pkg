#ifndef VISION_TRAJECTORY
#define VISION_TRAJECTORY

#include <ros/ros.h>
// #include "arm_catch_pkg/TrajectoryPredict.h"
#include <actionlib/client/simple_action_client.h>
#include "control_msgs/GripperCommandGoal.h"
#include "control_msgs/GripperCommandAction.h"


#include "geometry_msgs/PoseStamped.h"
#include <cstdlib>
#include "Eigen/Dense"
#include <cmath>
#include "arm_test_pkg/ball_trajectory_predict.h"

class VisionTrajectorySubscriber{
public:
	VisionTrajectorySubscriber(ros::NodeHandle &n);
	void trajectoryCB(const geometry_msgs::PoseStamped &msg);

protected:
	ros::Subscriber _sub;
	ros::Publisher _pub;
	bool _published;
	bool _prev_pose_exists;
	//ros::Time _prev_pose_time;
	geometry_msgs::PoseStamped _prev_pose;

	static const int _SAMPLE_COUNT = 2;
	int _samples_taken;
	geometry_msgs::PoseStamped _pose_samples[_SAMPLE_COUNT];
};



#endif
