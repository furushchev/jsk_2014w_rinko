#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <opencv2/video/tracking.hpp>

class Linear3DKalmanFilter
{
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
//  ros::Rate pub_rate_;
//  ros::Timer pub_timer_;

  cv::KalmanFilter kf_;

public:
  Linear3DKalmanFilter(): nh_(), pnh_("~") {
    kf_ = cv::KalmanFilter(6, 3, 0);
    // init kalman filter
    {
      cv::setIdentity(kf_.measurementMatrix);
      cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(1e-5));
      cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(1e-1));
      cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1));
    }

    // linear control matrix
    float dT = 1.0;
    kf_.transitionMatrix = (cv::Mat_<float>(6, 6) <<
                            1, 0, 0, dT, 0, 0,
                            0, 1, 0, 0, dT, 0,
                            0, 0, 1, 0, 0, dT,
                            0, 0, 0, 1, 0, 0,
                            0, 0, 0, 0, 1, 0,
                            0, 0, 0, 0, 0, 1);

    // init ros
    pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_filtered", 1000);
    sub_ = nh_.subscribe("pose", 1, &Linear3DKalmanFilter::poseCallback, this);
    // double pub_rate_val;
    // pnh_.param<double>("publish_rate", pub_rate_val, 100.0);
    // pub_rate_ = ros::Rate(pub_rate_val);
  };

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    cv::Mat measurement = (cv::Mat_<float>(3, 1) <<
                           msg->pose.position.x,
                           msg->pose.position.y,
                           msg->pose.position.z);
    kf_.correct(measurement);
    cv::Mat prediction = kf_.predict();
    geometry_msgs::PoseStamped pub_msg;
    pub_msg.header = msg->header;
    pub_msg.pose.position.x = prediction.at<float>(0);
    pub_msg.pose.position.y = prediction.at<float>(1);
    pub_msg.pose.position.z = prediction.at<float>(2);
    pub_msg.pose.orientation = msg->pose.orientation;
    pub_.publish(pub_msg);
    ROS_INFO_STREAM("x: " << msg->pose.position.x << " -> " << pub_msg.pose.position.x <<
                    ", y: " << msg->pose.position.y << " -> " << pub_msg.pose.position.y <<
                    ", z: " << msg->pose.position.z << " -> " << pub_msg.pose.position.z);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "linear_3d_kalman_filter");
  Linear3DKalmanFilter k;
  ros::spin();

  return 0;
}
