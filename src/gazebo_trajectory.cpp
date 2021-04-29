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
			// for(float t = 0.5; t < 1.5; t += 0.5){
			// 	srv.request.time_s = t;
			// 	_client.call(srv);

			// 	if(srv.response.predicted_pose.position.z > 0.5 && 
			// 		srv.response.predicted_pose.position.z < 0.8 &&
			// 		srv.response.predicted_pose.position.y > 0.2 &&
			// 		srv.response.predicted_pose.position.y < 0.8){
					
			// 		break;
			// 	}
			// }
			

			

			geometry_msgs::Pose goal_pose(srv.response.predicted_pose);
			goal_pose.position.z -= 0.5; // Arm is vertically offset from ground

			Eigen::MatrixXd xAxis(3,1);
			xAxis(0,0) = 1;
			xAxis(1,0) = 0;
			xAxis(2,0) = 0;
			Eigen::MatrixXd zAxis(3,1);
			zAxis(0,0) = 0;
			zAxis(1,0) = 0;
			zAxis(2,0) = 1;

			// Need to add pi to be angled towards the ball, rather than in line with it
			float pitch = atan2(srv.response.predicted_twist.linear.z, srv.response.predicted_twist.linear.y) + M_PI;
			float yaw = atan2(srv.response.predicted_twist.linear.y, srv.response.predicted_twist.linear.x) + M_PI;

			Eigen::AngleAxisd rotX(pitch, xAxis);
			Eigen::AngleAxisd rotZ(yaw, zAxis);

			Eigen::Quaterniond initRot(1, 0, 0, 0);
			Eigen::Quaterniond rotated = initRot * rotX * rotZ;

			goal_pose.orientation.x = rotated.x();
			goal_pose.orientation.y = rotated.y();
			goal_pose.orientation.z = rotated.z();
			goal_pose.orientation.w = rotated.w();

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