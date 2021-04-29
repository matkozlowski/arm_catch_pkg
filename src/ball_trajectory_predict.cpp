#include <ros/ros.h>
#include "arm_catch_pkg/TrajectoryPredict.h"


bool predict(arm_catch_pkg::TrajectoryPredict::Request &req, 
		arm_catch_pkg::TrajectoryPredict::Response &res){

	geometry_msgs::Pose pred_pose;
	geometry_msgs::Twist pred_twist;

	// Vertical Motion (z)
	// z = z_0 + (v_0z * t) + ((1/2) * -g * t^2)
	// v_z = v_0z - g*t
	pred_pose.position.z = req.start_pose.position.z + (req.start_twist.linear.z * req.time_s) + 
		(-4.9035 * pow(req.time_s, 2));
	pred_twist.linear.z = req.start_twist.linear.z - (9.807 * req.time_s);

	// Horizontal Motion (x)
	// x = x_0 + v_x * t
	// v_x = v_0x
	pred_pose.position.x = req.start_pose.position.x + (req.start_twist.linear.x * req.time_s);
	pred_twist.linear.x = req.start_twist.linear.x;

	// Horizontal Motion (y)
	// y = y_0 + v_y * t
	// v_y = v_0y
	pred_pose.position.y = req.start_pose.position.y + (req.start_twist.linear.y * req.time_s);
	pred_twist.linear.y = req.start_twist.linear.y;

	// Fill out response
	res.predicted_pose = pred_pose;
	res.predicted_twist = pred_twist;
	return true;

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_trajectory_predict");
	ros::NodeHandle node_handle;

	ros::ServiceServer service = node_handle.advertiseService("trajectory_predict", predict);

	ros::spin();

	return 0;
}