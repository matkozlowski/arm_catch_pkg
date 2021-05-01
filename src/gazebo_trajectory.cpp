#include "arm_test_pkg/gazebo_trajectory.h"


GazeboTrajectorySubscriber::GazeboTrajectorySubscriber(ros::NodeHandle &n){
	_sub = n.subscribe("/gazebo/model_states", 1000, &GazeboTrajectorySubscriber::trajectoryCB, this);
	_pub = n.advertise<geometry_msgs::Pose>("arm_goal_pose", 1000);
	_published = false;
	_prev_pose_exists = false;
}

void GazeboTrajectorySubscriber::trajectoryCB(const gazebo_msgs::ModelStates &msg){


	if(!_published){
	for(int n = 0; n < msg.name.size(); n++){
		if(msg.name.at(n).compare("RoboCup SPL Ball") == 0){
			// Ensure that a previous pose has been recorded
			if(!_prev_pose_exists){
				_prev_pose = msg.pose.at(n);
				_prev_pose_exists = true;
				_prev_pose_time = ros::Time::now();
				break;
			}

			double delta_t = (ros::Time::now() - _prev_pose_time).toSec();
			geometry_msgs::Pose start_pose = msg.pose.at(n);
			geometry_msgs::Twist start_twist;
			if(delta_t <= 0.001){
				break;
			}
			start_twist.linear.x = (start_pose.position.x - _prev_pose.position.x) / (delta_t);
			start_twist.linear.y = (start_pose.position.y - _prev_pose.position.y) / (delta_t);
			start_twist.linear.z = (start_pose.position.z - _prev_pose.position.z) / (delta_t);

			_prev_pose = start_pose;
			_prev_pose_time = ros::Time::now();
			geometry_msgs::Twist actual_twist = msg.twist.at(n);


			geometry_msgs::Pose pred_pose;
			geometry_msgs::Twist pred_twist;
			float time_s;

			for(float t = 0.0; t < 4.0; t+=0.01){
				time_s = t;
				predict_trajectory(start_pose, start_twist, time_s, pred_pose, pred_twist);
				if(pred_pose.position.z > 0.5 && 
					pred_pose.position.z < 0.8 &&
					pred_pose.position.y > 0.2 &&
					pred_pose.position.y < 0.8 &&
					pred_pose.position.x > -0.3 &&
					pred_pose.position.x < 0.3){
					
					break;
				}
			}

			geometry_msgs::Pose goal_pose(pred_pose);
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
			float pitch = atan2(pred_twist.linear.z, pred_twist.linear.y) + M_PI;
			float yaw = atan2(pred_twist.linear.y, pred_twist.linear.x) + M_PI;

			Eigen::AngleAxisd rotX(pitch, xAxis);
			Eigen::AngleAxisd rotZ(yaw, zAxis);

			Eigen::Quaterniond initRot(1, 0, 0, 0);
			Eigen::Quaterniond rotated = initRot * rotX * rotZ;

			goal_pose.orientation.x = rotated.x();
			goal_pose.orientation.y = rotated.y();
			goal_pose.orientation.z = rotated.z();
			goal_pose.orientation.w = rotated.w();

			if(!_published && goal_pose.position.z > 0 && goal_pose.position.y > 0){
				ROS_INFO("Predicted Position:\nX: %f\nY: %f\nZ: %f", 
					goal_pose.position.x,
					goal_pose.position.y,
					goal_pose.position.z); 
				ROS_INFO("TWIST Comparison:\nX actual: %f\nY actual: %f\nZ actual: %f\nX pred: %f\nY pred: %f\nZ pred: %f", 
					actual_twist.linear.x,
					actual_twist.linear.y,
					actual_twist.linear.z,
					pred_twist.linear.x,
					pred_twist.linear.y,
					pred_twist.linear.z); 
				_pub.publish(goal_pose);
				_published = true;
			}
			
			return;
		}
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