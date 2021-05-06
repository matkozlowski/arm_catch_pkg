#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <string>
#include "ros/service_client.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ApplyBodyWrench.h"

// Launches the ball in the simulation from a provided position with a provided velocity
// Takes 6 command line arguments in this order:
// 	X, Y, Z position of ball
//	X, Y, Z velocity of ball
int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_launch");
	ros::NodeHandle node_handle;

	// Create a client for the gazebo service that sets model states
	ros::ServiceClient modelState_Client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	// Create object for ball model in gazebo
	gazebo_msgs::ModelState modelState;
	modelState.model_name = (std::string) "RoboCup SPL Ball";

	// Determine the starting pose of the ball
	float x_pos = 0;
	float y_pos = 0;
	float z_pos = 0;
	if(argc == 7){
		x_pos = std::stof(argv[1]);
		y_pos = std::stof(argv[2]);
		z_pos = std::stof(argv[3]);
	}
	modelState.pose.position.x = x_pos;
	modelState.pose.position.y = y_pos;
	modelState.pose.position.z = z_pos;
	modelState.pose.orientation.w = 1.0;
	modelState.pose.orientation.x = 0.0;
	modelState.pose.orientation.y = 0.0;
	modelState.pose.orientation.z = 0.0;

	// Determine the starting velocity of the ball
	float x_vel = 0;
	float y_vel = 0;
	float z_vel = 0;
	if(argc == 7){
		x_vel = std::stof(argv[4]);
		y_vel = std::stof(argv[5]);
		z_vel = std::stof(argv[6]);
	}
	modelState.twist.linear.z = z_vel;
	modelState.twist.linear.y = y_vel;
	modelState.twist.linear.x = x_vel;

	// Call the service to set the position and velocity of the ball
	gazebo_msgs::SetModelState srv;
	srv.request.model_state = modelState;
	modelState_Client.call(srv);

	return 0;
}