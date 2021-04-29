#include "arm_test_pkg/ball_trajectory_predict.h"


void predict_trajectory(geometry_msgs::Pose start_pose, geometry_msgs::Twist start_twist,
		float time_s, geometry_msgs::Pose &pred_pose, geometry_msgs::Twist &pred_twist){
	// Vertical Motion (z)
	// z = z_0 + (v_0z * t) + ((1/2) * -g * t^2)
	// v_z = v_0z - g*t
	pred_pose.position.z = start_pose.position.z + (start_twist.linear.z * time_s) + 
		(-4.9035 * pow(time_s, 2));
	pred_twist.linear.z = start_twist.linear.z - (9.807 * time_s);

	// Horizontal Motion (x)
	// x = x_0 + v_x * t
	// v_x = v_0x
	pred_pose.position.x = start_pose.position.x + (start_twist.linear.x * time_s);
	pred_twist.linear.x = start_twist.linear.x;

	// Horizontal Motion (y)
	// y = y_0 + v_y * t
	// v_y = v_0y
	pred_pose.position.y = start_pose.position.y + (start_twist.linear.y * time_s);
	pred_twist.linear.y = start_twist.linear.y;
}