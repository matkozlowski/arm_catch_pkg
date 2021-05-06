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
#include <geometry_msgs/Pose.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <xtensor/xarray.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xio.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// Class that uses images and point clouds received from a depth camera
// to determine the position of the center of a red ball
class BallSegment {

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Publisher image_pub_;
	ros::Publisher _ball_pose_pub;

	// Subscribers for the color image and point cloud
	message_filters::Subscriber<sensor_msgs::Image> _im_sub;
	message_filters::Subscriber<PointCloud> _pc_sub;
	message_filters::TimeSynchronizer<sensor_msgs::Image, PointCloud> t_sync;


	public:BallSegment() : it_(nh_), _im_sub(nh_, "/camera/color/image_raw", 1), 
		_pc_sub(nh_, "/camera/depth/points", 1), t_sync(_im_sub, _pc_sub, 10) {

		image_pub_ = it_.advertise("/color_filter/segmented_ball", 1);
		_ball_pose_pub = nh_.advertise<geometry_msgs::Pose>("ball_pose_vision", 1000);

		// Subscribers for color image and point cloud need to be synchronized
		t_sync.registerCallback(&BallSegment::syncCB, this);
	}


	// Callback method for both color image and point cloud data that publishes a topic
	// with the position of the center of the ball
	// Also publishes masked image of ball
	void syncCB(const sensor_msgs::ImageConstPtr& img, const PointCloud::ConstPtr& pcloud){

		cv_bridge::CvImagePtr cv_img;
		try {
			cv_img = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::Mat HSVImage;
		cv::Mat ThreshImage;
		
		// Transform image into HSV
		cv::cvtColor(cv_img->image, HSVImage, CV_BGR2HSV);
		cv::inRange(HSVImage, cv::Scalar(0, 50, 0), cv::Scalar(255, 255, 255), ThreshImage);

		// Get center of the masked out blob using image moments
		// Also draws circle in published image of ball where center is
		cv::Moments moments = cv::moments(ThreshImage);
		cv::Point centerPoint(moments.m10/moments.m00, moments.m01/moments.m00);
		cv::circle(ThreshImage, centerPoint, 2, cv::Scalar(128,0,0), -1);

		// Publish masked image of ball with point at center
		cv_bridge::CvImage segmentedBall(std_msgs::Header(), "mono8", ThreshImage);
		image_pub_.publish(segmentedBall.toImageMsg());


		// Uses point cloud data to determine location of center of ball
		// Ensures that center of ball is within the image
		if(centerPoint.x >= 0 && centerPoint.y >= 0){
			pcl::PointXYZ pc
				_point = pcloud->at(centerPoint.x, centerPoint.y);
			printf("-------CENTER-------\n");
			printf("x: %f, y: %f, z:%f\n", pc_point.x, pc_point.z + 0.25, -pc_point.y + 0.55);
			geometry_msgs::Pose ball_pose;
			ball_pose.position.x = pc_point.x;
			ball_pose.position.y = pc_point.z + 0.25;
			ball_pose.position.z = -pc_point.y + 0.55;
			// Assemble the point matrices
			xt::xarray<double> spX, spY, spZ, A, F; 
			spX = xt::xarray<double>(ball_pose.position.x);
			spY = xt::xarray<double>(ball_pose.position.y);
			spZ = xt::xarray<double>(ball_pose.position.z);
			A = xt::zeros<double>((ball_pose.position.x.length, 4));
			xt::col(A, 0) = ball_pose.position.x * 2;
			xt::col(A, 1) = ball_pose.position.y * 2;
			xt::col(A, 2) = ball_pose.position.z * 2;
			xt::col(A, 3) = 1;
			
			// Assemble the lstqr matrix
			F = xt::zeros<double>((ball_pose.position.x.length, 1));
			xt::col(F, 0) = (spX * spX) + (spY * spY) + (spZ * spZ);
			C, residuals, rank, singval = xt::linalg::lstsq(A, F);
			double t = (C[0] * C[0]) + (C[1] * C[1]) + (C[2] * C[2]) + C[3];	
			double radius = sqrt(t);
			_ball_pose_pub.publish(ball_pose);
		}

		return;
	}

	// UNUSED and INCOMPLETE method that attempts to fit a sphere to point cloud data
	void sphere_fit(const PointCloud::ConstPtr& p_cloud){

		// std::vector<cv::Point> ball_pixels;
		// cv::findNonZero(_ball_image, ball_pixels);
		// double A[ball_pixels.size()][4];
		// double f[ball_pixels.size()];

		// int index = 0;
		// printf("------------POINTS------------\n");
		// for(cv::Point image_point : ball_pixels){
		// 	pcl::PointXYZ pc_point = p_cloud->at(image_point.x, image_point.y);

		// 	if(!std::isnan(pc_point.x) && !std::isnan(pc_point.y) && !std::isnan(pc_point.z)){
		// 		A[index][0] = pc_point.x*2;
		// 		A[index][1] = pc_point.y*2;
		// 		A[index][2] = pc_point.z*2;
		// 		A[index][3] = 1;

		// 		f[index] = pow(pc_point.x, 2) + pow(pc_point.y, 2) + pow(pc_point.z, 2);

		// 		printf("x: %f, y: %f, z:%f\n", pc_point.x, pc_point.y, pc_point.z);
		// 	}
		// 	index++;
		// }
	}
};

int main (int argc, char** argv) {
	ros::init(argc, argv, "ball_segment");
	BallSegment bs;
	ros::spin();
	return 0;
}
