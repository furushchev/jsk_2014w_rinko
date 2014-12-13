//#include <cmath>
#include <vector>
#include <string>

#include <boost/function.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_2014w_rinko/PSMoveButton.h>

#include <psmove.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#define PSMOVE_FUSION_STEP_EPSILON (.0001)
/* Field of view of the PS Eye */
#define PSEYE_FOV_BLUE_DOT 75
#define PSEYE_FOV_RED_DOT 56

#define NIL (-1)

float qx = 0.f, qy = 0.f, qz = 0.f, qw = 0.f;
bool has_orientation = false;

struct buffered_vector {
public:
  std::vector<float> data;
  int p, n_;
  int length;
  buffered_vector(int n){
    n_ = n;
    data = std::vector<float>();
    data.resize(n);
    this->clear();
  };
  void insert(float v){
    data[p] = v;
    p++;
    if (p >= n_) p = 0;
    if (length < n_) length++;
  }
  float average(){
    if (length <= 0) return 0.f;
    float sum = 0;
    for (int i = 0; i < length; i++) sum += data[i];
    return sum / length;
  }
  float latest(){
    if(length != 0) return data[p];
    else return 0.f;
  }
  void clear(){
    for (int i = 0; i < n_; ++i) data[i] = 0.f;
    length = 0;
    p = 0;
  }
};

struct fusion {
public:
  float width, height;
  glm::mat4 projection, modelview;
  glm::vec4 viewport;

  fusion(float width, float height, float z_near, float z_far): width(width), height(height){
    ROS_INFO_STREAM("fusion initialized");
    projection = glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT, width, height, z_near, z_far);
    viewport = glm::vec4(0., 0., width, height);
    std::cout << "HEY" << std::endl;
  };

  cv::Vec3f get_position(float camX, float camY, float camR){
    float winX = camX;
    float winY = height - camY;
    float winZ = .5; /* start value for binary search */

    float targetWidth = 2.*camR;
    glm::vec3 obj;
    /* Binary search for the best distance based on the current projection */
    float step = .25;

    while (step > PSMOVE_FUSION_STEP_EPSILON) {
      /* Calculate center position of sphere */
      obj = glm::unProject(glm::vec3(winX, winY, winZ),
                           glm::mat4(), projection, viewport);

      /* Project left edge center of sphere */
      glm::vec3 left = glm::project(glm::vec3(obj.x - .5, obj.y, obj.z),
                                    glm::mat4(), projection, viewport);

      /* Project right edge center of sphere */
      glm::vec3 right = glm::project(glm::vec3(obj.x + .5, obj.y, obj.z),
                                     glm::mat4(), projection, viewport);

      float width_ = (right.x - left.x);
      if (width_ > targetWidth) {
        /* Too near */
        winZ += step;
      } else if (width_ < targetWidth) {
        /* Too far away */
        winZ -= step;
      } else {
        /* Perfect fit */
        break;
      }
      step *= .5;
    }
    return cv::Vec3f(obj.x, obj.y, obj.z);
  }
};


void min_max_color(int color, int threshold, int& min_color, int& max_color)
{
  if (color == NIL){
    min_color = 0;
    max_color = 255;
  } else {
    min_color = (color - threshold < 0) ? 0 : color - threshold;
    max_color = (color + threshold > 255) ? 255 : color + threshold;
  }
}

class PSMoveTracker {
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  int color_r_min_, color_r_max_;
  int color_g_min_, color_g_max_;
  int color_b_min_, color_b_max_;

  int color_threshold_;
  int lpf_threshold_;
  int roi_margin_2_;

  buffered_vector* prev_radius_;
  buffered_vector* prev_centroid_x_;
  buffered_vector* prev_centroid_y_;
  bool force_not_roi_;

  fusion* fusion_;
  bool fusion_alloced_;
  double z_near_, z_far_;

  int min_radius_, max_radius_;
  int canny_threshold_, center_detection_threshold_;

  std::string base_frame_id_;

  bool x_mirror_, y_mirror_, z_mirror_;
  int x_mirror_val_, y_mirror_val_, z_mirror_val_;

  bool debug_;

public:
  ros::NodeHandle nh;
  int color_r, color_g, color_b;

  PSMoveTracker(): nh(), nh_("~"), force_not_roi_(false){
    nh_.param<int>("color_r", color_r, NIL);
    nh_.param<int>("color_g", color_g, NIL);
    nh_.param<int>("color_b", color_b, NIL);
    nh_.param<int>("color_threshold" , color_threshold_, 20);

    ROS_INFO_STREAM("R: " << color_r << " G: " << color_g << " B: " << color_b << " TH: " << color_threshold_);

    min_max_color(color_r, color_threshold_, color_r_min_, color_r_max_);
    min_max_color(color_g, color_threshold_, color_g_min_, color_g_max_);
    min_max_color(color_b, color_threshold_, color_b_min_, color_b_max_);

    nh_.param<int>("lpf_threshold", lpf_threshold_, 5);
    nh_.param<int>("roi_margin_2", roi_margin_2_, 50);

    nh_.param<int>("min_radius", min_radius_, 5);
    nh_.param<int>("max_radius", max_radius_, 150);
    nh_.param<int>("canny_threshold", canny_threshold_, 200);
    nh_.param<int>("center_detection_threshold", center_detection_threshold_, 40);

    prev_radius_ = new buffered_vector(lpf_threshold_);
    prev_centroid_x_ = new buffered_vector(lpf_threshold_);
    prev_centroid_y_ = new buffered_vector(lpf_threshold_);

    nh_.param<double>("z_near", z_near_, 1.);
    nh_.param<double>("z_far", z_far_, 1000.);

    nh_.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");

    nh_.param<bool>("x_mirror", x_mirror_, true);
    if (x_mirror_) x_mirror_val_ = -1;
    else x_mirror_val_ = 1;

    nh_.param<bool>("y_mirror", y_mirror_, true);
    if (y_mirror_) y_mirror_val_ = -1;
    else y_mirror_val_ = 1;

    nh_.param<bool>("z_mirror", z_mirror_, true);
    if (z_mirror_) z_mirror_val_ = -1;
    else z_mirror_val_ = 1;


    nh_.param<bool>("debug", debug_, false);

    fusion_alloced_ = false;

    pub_ = nh.advertise<geometry_msgs::PoseStamped>("psmove", 1000);
    sub_ = nh.subscribe("image", 1, &PSMoveTracker::imageCallback, this);
  };

  ~PSMoveTracker(){
    delete prev_radius_;
    delete prev_centroid_x_;
    delete prev_centroid_y_;
    delete fusion_;
  };

  void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    if(!fusion_alloced_){
      fusion_ = new fusion(msg->width, msg->height, z_near_, z_far_);
      fusion_alloced_ = true;
    }

    cv_bridge::CvImageConstPtr cvimg = cv_bridge::toCvShare(msg);

    cv::Mat frame = cvimg->image;
    cv::Mat filtered, roi_image;

    bool is_roi = false;
    cv::Rect roi_rect;
    if (prev_radius_->latest() <= 0 || force_not_roi_){
      roi_image = frame;
      force_not_roi_ = false;
    }
    else {
      // set roi
      is_roi = true;
      int roi_rect_window_2 = prev_radius_->latest() + roi_margin_2_;
      roi_rect = cv::Rect(prev_centroid_x_->latest() - roi_rect_window_2,
                          prev_centroid_y_->latest() - roi_rect_window_2,
                          cv::min(roi_rect_window_2 * 2, frame.size().width),
                          cv::min(roi_rect_window_2 * 2, frame.size().height));
      if (roi_rect.x < 0) roi_rect.x = 0;
      if (roi_rect.y < 0) roi_rect.y = 0;
      if (roi_rect.x + roi_rect.width > frame.size().width) roi_rect.x = frame.size().width - roi_rect.width;
      if (roi_rect.y + roi_rect.height > frame.size().height) roi_rect.y = frame.size().height - roi_rect.height;

      roi_image = frame(roi_rect);

      if (debug_)
        ROS_INFO_STREAM("roi x: " << roi_rect.x << " y: " << roi_rect.y << " w: " << roi_rect.width << " h: " << roi_rect.height);
    }

    // color filter
    cv::inRange(roi_image, cv::Scalar(color_b_min_, color_g_min_, color_r_min_), cv::Scalar(color_b_max_, color_g_max_, color_r_max_), filtered);
    cv::GaussianBlur(filtered, filtered, cv::Size(9,9), 2, 2);

    // circle detection
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(filtered, circles, CV_HOUGH_GRADIENT, 2, roi_image.rows / 4,
                     canny_threshold_, center_detection_threshold_,
                     min_radius_, max_radius_);

    float cent_x, cent_y;
    if (circles.size() <= 0) {
      if (debug_) ROS_INFO_STREAM("no circle detected.");

      force_not_roi_ = true;
      return;
    }

    float max_radius = 0.0f;
    for (size_t i = 0; i < circles.size(); ++i){
      float radius = circles[i][2];
      if (max_radius < radius){
        max_radius = radius;
        cent_x = circles[i][0];
        cent_y = circles[i][1];
      }
    }
    if (is_roi) {
      cent_x += roi_rect.x;
      cent_y += roi_rect.y;
    }
    prev_radius_->insert(max_radius);
    prev_centroid_x_->insert(cent_x);
    prev_centroid_y_->insert(cent_y);

    float avg_radius = prev_radius_->average();
    float avg_cent_x = prev_centroid_x_->average();
    float avg_cent_y = prev_centroid_y_->average();
    if (debug_){
      ROS_INFO_STREAM("x: " << avg_cent_x << " y: " << avg_cent_y << " r: " << avg_radius);
      cv::Mat frame_copy = frame.clone();
      cv::circle( frame_copy, cv::Point(avg_cent_x, avg_cent_y), avg_radius, cv::Scalar(0, 0, 255), 3, 8, 0);
      cv::imshow("debug1", filtered);
      cv::imshow("debug2", frame_copy);
      cvWaitKey(1);
    }

    cv::Point3f pos = fusion_->get_position(avg_cent_x, avg_cent_y, avg_radius);
    if (debug_) ROS_INFO_STREAM("pos x: " << pos.x << " y: " << pos.y << " z: " << pos.z);

    // publish pose
    float intensity = 0.1;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = base_frame_id_;
    pose.pose.position.x = pos.z * intensity * x_mirror_val_;
    pose.pose.position.y = pos.x * intensity * y_mirror_val_;
    pose.pose.position.z = pos.y * intensity * z_mirror_val_;
    if(has_orientation){
      pose.pose.orientation.x = qx;
      pose.pose.orientation.y = qy;
      pose.pose.orientation.z = qz;
      pose.pose.orientation.w = qw;
    }
    pub_.publish(pose);
  }
};

std::vector<unsigned char> get_button_chars(PSMove* move){
  unsigned int b = psmove_get_buttons(move);
  std::vector<unsigned char> ret;
  if(b & Btn_PS) ret.push_back('p');
  if(b & Btn_TRIANGLE) ret.push_back('t');
  if(b & Btn_CIRCLE) ret.push_back('c');
  if(b & Btn_CROSS) ret.push_back('x');
  if(b & Btn_SQUARE) ret.push_back('s');
  if(b & Btn_SELECT) ret.push_back('0');
  if(b & Btn_START) ret.push_back('1');
  if(b & Btn_MOVE) ret.push_back('m');
  if(b & Btn_T) ret.push_back('l');
  return ret;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psmove_tracker");

  PSMoveTracker p;

  PSMove* move_;
  psmove_init(PSMOVE_CURRENT_VERSION);
  move_ = psmove_connect();

  if (!move_){
    ROS_FATAL_STREAM("failed to connect to default move controller.\n" << "Please connect via USB or BT.");
    exit(1);
  }

  char *serial = psmove_get_serial(move_);
  ROS_INFO_STREAM("serial: " << serial);
  free(serial);

  enum PSMove_Connection_Type ctype = psmove_connection_type(move_);
  switch(ctype)
  {
  case Conn_USB:
    ROS_INFO_STREAM("connected via usb");
    break;
  case Conn_Bluetooth:
    ROS_INFO_STREAM("connected via bluetooth");
    break;
  default:
    ROS_INFO_STREAM("unknown connection type");
    break;
  }
  psmove_enable_orientation(move_, PSMove_True);

  ros::Publisher btn_pub = p.nh.advertise<jsk_2014w_rinko::PSMoveButton>("psmove_button", 1000);

  ros::Rate loop_rate_(30);

  while(ros::ok()){
    // get psmove sensor information
    int res = psmove_poll(move_);
    if (res){
      psmove_set_leds(move_, p.color_r, 0, p.color_b);
      psmove_update_leds(move_);
      if (psmove_has_orientation(move_) == PSMove_True)
        has_orientation = true;
      psmove_get_orientation(move_, &qx, &qy, &qz, &qw);

      try {
        std::vector<unsigned char> btn = get_button_chars(move_);
        unsigned char trigger_val = psmove_get_trigger(move_);
        jsk_2014w_rinko::PSMoveButton btn_msg;
        btn_msg.button = btn;
        btn_msg.trigger = trigger_val / 255.0f;
        btn_pub.publish(btn_msg);
      } catch (...) {
        ROS_WARN_STREAM("failed to publish button");
      }

    } else {
//      ROS_WARN_STREAM("failed ay psmove_poll");
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }

  psmove_disconnect(move_);
  return 0;
}
