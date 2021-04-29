#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class BallSegment {

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    public:BallSegment() : it_(nh_) {
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &BallSegment::imageSplit, this);
        image_pub_ = it_.advertise("/color_filter/segmented_ball", 1);
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
