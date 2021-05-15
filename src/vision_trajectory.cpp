#include "arm_test_pkg/vision_trajectory.h"


// Class that subscribes to position of ball that was determined through camera
// and uses that information to predict the trajectory of the ball and publish
// a goal position for the arm to move to in order to catch the ball
VisionTrajectorySubscriber::VisionTrajectorySubscriber(ros::NodeHandle &n){
	_sub = n.subscribe("/ball_pose_vision", 1000, &VisionTrajectorySubscriber::trajectoryCB, this);
	_pub = n.advertise<geometry_msgs::Pose>("arm_goal_pose", 1000);
	_published = false;
	_prev_pose_exists = false;

	_samples_taken = 0;
}


void VisionTrajectorySubscriber::trajectoryCB(const geometry_msgs::PoseStamped &msg){
	
	// Ensures that only a single goal is ever published
	if(!_published){

		if(_samples_taken < _SAMPLE_COUNT){
			_pose_samples[_samples_taken] = msg;
			_samples_taken++;
			return;
		}

		bool t_found = false;
		float pred_t = 0;
		geometry_msgs::Pose pred_poses[_SAMPLE_COUNT - 1];
		geometry_msgs::Twist pred_twists[_SAMPLE_COUNT - 1];

		for(int s = 0; s < _SAMPLE_COUNT - 1; s++){
			double delta_t = (_pose_samples[s+1].header.stamp - _pose_samples[s].header.stamp).toSec();

			geometry_msgs::PoseStamped prev_pose = _pose_samples[s];
			geometry_msgs::PoseStamped start_pose = _pose_samples[s+1];
			geometry_msgs::Twist start_twist;

			start_twist.linear.x = (start_pose.pose.position.x - prev_pose.pose.position.x) / (delta_t);
			start_twist.linear.y = (start_pose.pose.position.y - prev_pose.pose.position.y) / (delta_t);
			start_twist.linear.z = (start_pose.pose.position.z - prev_pose.pose.position.z) / (delta_t);

			geometry_msgs::Pose pred_pose;
			geometry_msgs::Twist pred_twist;

			if(!t_found){
			for(float t = 0.0; t < 4.0; t+=0.01){
				// Predict the position of the ball after a certain amount of time
				predict_trajectory(start_pose.pose, start_twist, t, pred_pose, pred_twist);

				// Stop once the first position is found that is within a certain bounding
				// box in front of the arm
				// TODO:: Change this bounding box, is is currently just a rough guess
				if(pred_pose.position.z > 0.5 && 
					pred_pose.position.z < 0.8 &&
					pred_pose.position.y > 0.2 &&
					pred_pose.position.y < 0.8 &&
					pred_pose.position.x > -0.3 &&
					pred_pose.position.x < 0.3){
					
					t_found = true;
					pred_t = t;

					pred_poses[s] = pred_pose;
					pred_twists[s] = pred_twist;

					break;
				}
			}
			}

			pred_t -= delta_t;
			predict_trajectory(start_pose.pose, start_twist, pred_t, pred_pose, pred_twist);
			pred_poses[s] = pred_pose;
			pred_twists[s] = pred_twist;
		}

		geometry_msgs::Pose avg_pred_pose;
		geometry_msgs::Twist avg_pred_twist;
		double x_sum = 0;
		double y_sum = 0;
		double z_sum = 0;
		double xt_sum = 0;
		double yt_sum = 0;
		double zt_sum = 0;
		for(int s = 0; s < _SAMPLE_COUNT - 1; s++){
			ROS_INFO("PRED TWISTS:\nX pred: %f\nY pred: %f\nZ pred: %f",
				pred_twists[s].linear.x,
				pred_twists[s].linear.y,
				pred_twists[s].linear.z);
			x_sum += pred_poses[s].position.x;
			y_sum += pred_poses[s].position.y;
			z_sum += pred_poses[s].position.z;
			xt_sum += pred_twists[s].linear.x;
			yt_sum += pred_twists[s].linear.y;
			zt_sum += pred_twists[s].linear.z;
		}
		avg_pred_pose.position.x = x_sum / (double)(_SAMPLE_COUNT - 1);
		avg_pred_pose.position.y = y_sum / (double)(_SAMPLE_COUNT - 1);
		avg_pred_pose.position.z = z_sum / (double)(_SAMPLE_COUNT - 1);
		avg_pred_twist.linear.x = xt_sum / (double)(_SAMPLE_COUNT - 1);
		avg_pred_twist.linear.y = yt_sum / (double)(_SAMPLE_COUNT - 1);
		avg_pred_twist.linear.z = zt_sum / (double)(_SAMPLE_COUNT - 1);

		// Set goal position to be predicted position
		geometry_msgs::Pose goal_pose(avg_pred_pose);
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
		float pitch = atan2(avg_pred_twist.linear.z, avg_pred_twist.linear.y) + M_PI;
		float yaw = atan2(avg_pred_twist.linear.y, avg_pred_twist.linear.x) + M_PI;

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
				avg_pred_twist.linear.x,
				avg_pred_twist.linear.y,
				avg_pred_twist.linear.z); 

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