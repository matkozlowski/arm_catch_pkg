#include <ros/ros.h>
#include <cstdlib>
#include <ctime>
#include <string>
#include "ros/service_client.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ApplyBodyWrench.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_launch");
	ros::NodeHandle node_handle;


	ros::ServiceClient modelState_Client = node_handle.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

	
	geometry_msgs::Pose modelPose;
	float x_pos = 0;
	float y_pos = 0;
	float z_pos = 0;
	if(argc == 7){
		x_pos = std::stof(argv[1]);
		y_pos = std::stof(argv[2]);
		z_pos = std::stof(argv[3]);
	}
	modelPose.position.x = x_pos;
	modelPose.position.y = y_pos;
	modelPose.position.z = z_pos;
	modelPose.orientation.w = 1.0;
	modelPose.orientation.x = 0.0;
	modelPose.orientation.y = 0.0;
	modelPose.orientation.z = 0.0;

	gazebo_msgs::ModelState modelState;
	modelState.model_name = (std::string) "RoboCup SPL Ball";
	modelState.pose = modelPose;
	
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

	gazebo_msgs::SetModelState srv;
	srv.request.model_state = modelState;

	modelState_Client.call(srv);



	// Range for targeting:
	// -0.3 < x < 0.3
	// 0.3 < y < 0.75
	// 0.6 < z < 1.0
	// srand (static_cast <unsigned> (time(0)));
	// float x_min = -0.3;
	// float x_max =0.3;
	// float y_min = 0.3;
	// float y_max = 0.75;
	// float z_min = 0.6;
	// float z_max = 1.0;
	// float randX = x_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(x_max-x_min)));
	// float randY = y_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(y_max-y_min)));
	// float randZ = z_min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(z_max-z_min)));


	return 0;
}