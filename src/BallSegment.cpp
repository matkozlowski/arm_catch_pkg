#include <ros/ros.h>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include "gazebo_msgs/GetModelState.h"
// #include <xtensor/xarray.hpp>
// #include <xtensor/xmath.hpp>
// #include <xtensor/xio.hpp>
// #include <xtensor/xtensor.hpp>
// #include <xtensor/xrandom.hpp>
// #include <xtensor/xlinalg.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class BallSegment {

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;
	cv::Point _ball_center;
	ros::Publisher _ball_pose_pub;

	ros::ServiceClient _gz_client;

	message_filters::Subscriber<sensor_msgs::Image> _im_sub;
	message_filters::Subscriber<PointCloud> _pc_sub;
	message_filters::TimeSynchronizer<sensor_msgs::Image, PointCloud> t_sync;



	public:BallSegment() : it_(nh_), _im_sub(nh_, "/camera/color/image_raw", 1), _pc_sub(nh_, "/camera/depth/points", 1),
		t_sync(_im_sub, _pc_sub, 10) {

		image_pub_ = it_.advertise("/color_filter/segmented_ball", 1);
		_ball_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("ball_pose_vision", 1000);

		_gz_client = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

		t_sync.registerCallback(&BallSegment::syncCB, this);
	}

	void syncCB(const sensor_msgs::ImageConstPtr& img, const PointCloud::ConstPtr& pcloud){
		gazebo_msgs::GetModelState srv;
		srv.request.model_name = "RoboCup SPL Ball";
		_gz_client.call(srv);

		printf("--------------\n");
		printf("-------ACTUAL-------\n");
		ROS_INFO("x: %f, y: %f, z: %f\n", srv.response.pose.position.x, srv.response.pose.position.y, srv.response.pose.position.z);

		cv_bridge::CvImagePtr cv_img;
		try {
			cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat HSVImage;
		cv::Mat ThreshImage;
		
		// transform image into HSV
		cv::cvtColor(cv_img->image, HSVImage, CV_BGR2HSV);
		cv::inRange(HSVImage, cv::Scalar(0, 50, 0), cv::Scalar(255, 255, 255), ThreshImage);

		// Get center of the maske out blob using image moments
		cv::Moments moments = cv::moments(ThreshImage);
		cv::Point centerPoint(moments.m10/moments.m00, moments.m01/moments.m00);
		cv::circle(ThreshImage, centerPoint, 2, cv::Scalar(128,0,0), -1);

		//_ball_image = ThreshImage;

		cv_bridge::CvImage segmentedBall(std_msgs::Header(), "mono8", ThreshImage);
		image_pub_.publish(segmentedBall.toImageMsg());



		if(centerPoint.x >= 0 && centerPoint.y >= 0){
			pcl::PointXYZ pc_point = sphere_fit(ThreshImage, pcloud);
			printf("-------PREDICTED-------\n");
			ROS_INFO("x: %f, y: %f, z:%f\n\n", pc_point.x, pc_point.z + 0.25, -pc_point.y + 0.55);
			geometry_msgs::PoseStamped ball_pose;
			ball_pose.pose.position.x = pc_point.x; 
			ball_pose.pose.position.y = pc_point.z + .25;
			ball_pose.pose.position.z = -pc_point.y + .55;

			std_msgs::Header hdr;
			hdr.stamp = ros::Time::now();
			ball_pose.header = hdr;

			_ball_pose_pub.publish(ball_pose);

			

			

			

			// // Assemble the point matrices
			// xt::xarray<double> spX, spY, spZ, A, F; 
			// spX = xt::xarray<double>(ball_pose.position.x);
			// spY = xt::xarray<double>(ball_pose.position.y);
			// spZ = xt::xarray<double>(ball_pose.position.z);
			// A = xt::zeros<double>((spX.size(), 4));
			// xt::col(A, 0) = ball_pose.position.x * 2;
			// xt::col(A, 1) = ball_pose.position.y * 2;
			// xt::col(A, 2) = ball_pose.position.z * 2;
			// xt::col(A, 3) = 1;
			
			// // Assemble the lstqr matrix
			// F = xt::zeros<double>((spX.size(), 1));
			// xt::col(F, 0) = (spX * spX) + (spY * spY) + (spZ * spZ);
			// auto res = xt::linalg::lstsq(A, F);
			
			// // here is the solution to the least square fit 
			// xt::xarray<double> C = get<0>(res);
            
			// double t = (C[0] * C[0]) + (C[1] * C[1]) + (C[2] * C[2]) + C[3];	
			// // here is the radius of the ball - do with this value what you will
			// double radius = sqrt(t);
			
		}

		return;
	}

	pcl::PointXYZ sphere_fit(const cv::Mat image, const PointCloud::ConstPtr pcloud){

		std::vector<cv::Point> ball_pixels;
		cv::findNonZero(image, ball_pixels);

		int index = 0;
		std::vector<pcl::PointXYZ> ball_points;
		for(cv::Point image_point : ball_pixels){
			pcl::PointXYZ pc_point = pcloud->at(image_point.x, image_point.y);

			if(!std::isnan(pc_point.x) && !std::isnan(pc_point.y) && !std::isnan(pc_point.z)){
				ball_points.push_back(pc_point);
			}
			index++;
		}

		double alpha = 0.05;
		double true_radius = 0.03;
		pcl::PointXYZ pred_center = ball_points.at(0);

		for(int i = 0; i < 100; i++){
			for(int p = 0; p < ball_points.size(); p++){
				double pred_r = sphere_radius(ball_points.at(p), pred_center);

				double gradX = gradient(true_radius, pred_r, ball_points.at(p), pred_center, 0);
				double gradY = gradient(true_radius, pred_r, ball_points.at(p), pred_center, 1);
				double gradZ = gradient(true_radius, pred_r, ball_points.at(p), pred_center, 2);
				
				pred_center.x -= gradX * alpha;
				pred_center.y -= gradY * alpha;
				pred_center.z -= gradZ * alpha;
			}
		}

		return pred_center;
	}

	double gradient(double r, double pred_r, pcl::PointXYZ point, pcl::PointXYZ center, int abc){
		double abc_point = 0;
		double abc_center = 0;

		switch(abc){
			case 0:
				abc_point = point.x;
				abc_center = center.x;
				break;
			case 1:
				abc_point = point.y;
				abc_center = center.y;
				break;
			case 2:
				abc_point = point.z;
				abc_center = center.z;
				break;
		}

		double radius = (sphere_radius(point, center));
		if(radius != 0.0)
			return (2 * (r - pred_r) * (abc_point - abc_center)) / radius;
		return 0;

	}

	double sphere_radius(pcl::PointXYZ point, pcl::PointXYZ center){
		return sqrt(pow((point.x - center.x),2) + pow((point.y - center.y),2) + pow((point.z - center.z),2));
	}

	void imageSplit (const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_img;
		try {
			cv_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat HSVImage;
		cv::Mat ThreshImage;
		
		// transform image into HSV
		cv::cvtColor(cv_img->image, HSVImage, CV_BGR2HSV);
		cv::inRange(HSVImage, cv::Scalar(0, 50, 0), cv::Scalar(255, 255, 255), ThreshImage);

		// Get center of the maske out blob using image moments
		cv::Moments moments = cv::moments(ThreshImage);
		cv::Point centerPoint(moments.m10/moments.m00, moments.m01/moments.m00);
		_ball_center = centerPoint;
		cv::circle(ThreshImage, centerPoint, 2, cv::Scalar(128,0,0), -1);

		//_ball_image = ThreshImage;

		cv_bridge::CvImage segmentedBall(std_msgs::Header(), "mono8", ThreshImage);
		image_pub_.publish(segmentedBall.toImageMsg());
	}


};

int main (int argc, char** argv) {
	ros::init(argc, argv, "ball_segment");
	BallSegment bs;
	ros::spin();
	return 0;
}
