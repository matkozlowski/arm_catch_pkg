#ifndef BALL_TRAJECTORY_PREDICT
#define BALL_TRAJECTORY_PREDICT

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

void predict_trajectory(const geometry_msgs::Pose start_pose, const geometry_msgs::Twist start_twist,
		float time_s, geometry_msgs::Pose &pred_pose, geometry_msgs::Twist &pred_twist);


#endif