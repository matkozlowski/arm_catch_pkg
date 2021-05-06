#include "arm_test_pkg/vision_trajectory.h"


// Class that subscribes to position of ball that was determined through camera
// and uses that information to predict the trajectory of the ball and publish
// a goal position for the arm to move to in order to catch the ball
VisionTrajectorySubscriber::VisionTrajectorySubscriber(ros::NodeHandle &n){
	_sub = n.subscribe("/ball_pose_vision", 1000, &VisionTrajectorySubscriber::trajectoryCB, this);
	_pub = n.advertise<geometry_msgs::Pose>("arm_goal_pose", 1000);
	_published = false;
	_prev_pose_exists = false;
}


void VisionTrajectorySubscriber::trajectoryCB(const geometry_msgs::Pose &msg){

	// Ensures that only a single goal is ever published
	if(!_published){
		// Ensure that a previous pose has been recorded
		// If not, then record this pose as the previous pose and exit
		if(!_prev_pose_exists){
			_prev_pose = msg;
			_prev_pose_exists = true;
			_prev_pose_time = ros::Time::now();
			return;
		}

		// Determine the difference in time between now and when the last pose was recorded
		double delta_t = (ros::Time::now() - _prev_pose_time).toSec();
		
		geometry_msgs::Pose start_pose = msg;
		geometry_msgs::Twist start_twist;

		// Ensure that the time difference is not too small, as it may cause larger errors
		// If it is too small, skip the most recently received pose and wait for the next one
		if(delta_t <= 0.001){
			return;
		}
		
		// Determine the velocity of the ball using the current and previous positions
		start_twist.linear.x = (start_pose.position.x - _prev_pose.position.x) / (delta_t);
		start_twist.linear.y = (start_pose.position.y - _prev_pose.position.y) / (delta_t);
		start_twist.linear.z = (start_pose.position.z - _prev_pose.position.z) / (delta_t);

		// Update the value of the previous pose
		_prev_pose = start_pose;
		_prev_pose_time = ros::Time::now();

		geometry_msgs::Pose pred_pose;
		geometry_msgs::Twist pred_twist;



		// Loop through different time values to determine when ball will be in
		// a position where the arm can catch it
		for(float t = 0.0; t < 4.0; t+=0.01){
			// Predict the position of the ball after a certain amount of time
			predict_trajectory(start_pose, start_twist, t, pred_pose, pred_twist);

			// Stop once the first position is found that is within a certain bounding
			// box in front of the arm
			// TODO:: Change this bounding box, is is currently just a rough guess
			if(pred_pose.position.z > 0.5 && 
				pred_pose.position.z < 0.8 &&
				pred_pose.position.y > 0.2 &&
				pred_pose.position.y < 0.8 &&
				pred_pose.position.x > -0.3 &&
				pred_pose.position.x < 0.3){
				
				break;
			}
		}

		// Set goal position to be predicted position
		geometry_msgs::Pose goal_pose(pred_pose);
		goal_pose.position.z -= 0.5; // Arm is vertically offset from ground



		// Determine rotation of end-effector that aligns it with the predicted
		// trajectory of the ball
		Eigen::MatrixXd xAxis(3,1);
		xAxis(0,0) = 1;
		xAxis(1,0) = 0;
		xAxis(2,0) = 0;
		Eigen::MatrixXd zAxis(3,1);
		zAxis(0,0) = 0;
		zAxis(1,0) = 0;
		zAxis(2,0) = 1;

		// Determine the necessary pitch and yaw of the arm
		// Need to add pi to be angled towards the ball, rather than in line with it
		float pitch = atan2(pred_twist.linear.z, pred_twist.linear.y) + M_PI;
		float yaw = atan2(pred_twist.linear.y, pred_twist.linear.x) + M_PI;

		// Determine the orientation of the end-effectory using pitch and yaw
		Eigen::AngleAxisd rotX(pitch, xAxis);
		Eigen::AngleAxisd rotZ(yaw, zAxis);
		Eigen::Quaterniond initRot(1, 0, 0, 0);
		Eigen::Quaterniond rotated = initRot * rotX * rotZ;

		// Set goal orientation to be predicted orientation
		goal_pose.orientation.x = rotated.x();
		goal_pose.orientation.y = rotated.y();
		goal_pose.orientation.z = rotated.z();
		goal_pose.orientation.w = rotated.w();


		// Ensure goal pose is in front of arm
		if(goal_pose.position.z > 0 && goal_pose.position.y > 0){
			ROS_INFO("Predicted Position:\nX: %f\nY: %f\nZ: %f", 
				goal_pose.position.x,
				goal_pose.position.y,
				goal_pose.position.z); 
			ROS_INFO("TWIST:\nX pred: %f\nY pred: %f\nZ pred: %f",
				pred_twist.linear.x,
				pred_twist.linear.y,
				pred_twist.linear.z); 

			// Publish the goal pose
			_pub.publish(goal_pose);
			_published = true;
		}
		
		return;
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_trajectory_client");
	ros::NodeHandle node_handle;

	VisionTrajectorySubscriber gts(node_handle);
	
	ros::spin();

	return 0;
}