#include "arm_test_pkg/gazebo_trajectory.h"


GazeboTrajectorySubscriber::GazeboTrajectorySubscriber(ros::NodeHandle &n){
	_sub = n.subscribe("/gazebo/model_states", 1000, &GazeboTrajectorySubscriber::trajectoryCB, this);
	_client = n.serviceClient<arm_catch_pkg::TrajectoryPredict>("trajectory_predict");
	_pub = n.advertise<geometry_msgs::Pose>("arm_goal_pose", 1000);
	published = false;
}

void GazeboTrajectorySubscriber::trajectoryCB(const gazebo_msgs::ModelStates &msg){
	arm_catch_pkg::TrajectoryPredict srv;

	for(int n = 0; n < msg.name.size(); n++){
		if(msg.name.at(n).compare("RoboCup SPL Ball") == 0){
			srv.request.start_pose = msg.pose.at(n);
			srv.request.start_twist = msg.twist.at(n);
			srv.request.time_s = 1;

			_client.call(srv);

			

			geometry_msgs::Pose goal_pose(srv.response.predicted_pose);
			goal_pose.position.z -= 0.5; // Arm is vertically offset from ground
			if(!published && goal_pose.position.z > 0 && goal_pose.position.y > 0){
				ROS_INFO("Trajectory Prediction Service Called\nX: %f\nY: %f\nZ: %f", 
					goal_pose.position.x,
					goal_pose.position.y,
					goal_pose.position.z); 
				_pub.publish(goal_pose);
				published = true;
			}
			
			return;
		}
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gazebo_trajectory_client");
	ros::NodeHandle node_handle;

	GazeboTrajectorySubscriber gts(node_handle);
	
	ros::spin();

	return 0;
}