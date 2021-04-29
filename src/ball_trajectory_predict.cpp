#include <ros/ros.h>
#include "arm_catch_pkg/TrajectoryPredict.h"


bool predict(arm_catch_pkg::TrajectoryPredict::Request &req, 
			 arm_catch_pkg::TrajectoryPredict::Response &res){

	geometry_msgs::Pose prediction;

	// Vertical Motion (z)
	// z = z_0 + (v_0z * t) + ((1/2) * g * t^2)
	prediction.position.z = req.start_pose.position.z + (req.start_twist.linear.z * req.time_s) + 
							(-4.9035 * pow(req.time_s, 2));

	// Horizontal Motion (x)
	// x = x_0 + v_x * t
	prediction.position.x = req.start_pose.position.x + (req.start_twist.linear.x * req.time_s);

	// Horizontal Motion (y)
	// y = y_0 + v_y * t
	prediction.position.y = req.start_pose.position.y + (req.start_twist.linear.y * req.time_s);

	// Fill out response
	res.predicted_pose = prediction;
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