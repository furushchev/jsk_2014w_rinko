#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <psmove.h>

#include <opencv2/opencv.hpp>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>

#define PSMOVE_FUSION_STEP_EPSILON (.0001)
/* Field of view of the PS Eye */
#define PSEYE_FOV_BLUE_DOT 75
#define PSEYE_FOV_RED_DOT 56


//#define DEBUG

struct buffered_vector {
public:
  std::vector<float> data;
  int p, n_;
  int length;
  buffered_vector(int n){
    n_ = n;
    data = std::vector<float>();
    data.resize(n);
    for (int i = 0; i < n_; ++i) data[i] = 0.f;
    p = 0;
    length = 0;
  };
  void insert(float v){
    data[p] = v;
    p++;
    if (p >= n_) p = 0;
    if (length < 3) length++;
  }
  float average(){
    if (length <= 0) return 0;
    float sum = 0;
    for (int i = 0; i < length; i++) sum += data[i];
    return sum / length;
  }
  float latest(){
    if(length != 0) return data[p];
    else return 0;
  }
};

struct fusion {
public:
  float width, height;
  glm::mat4 projection, modelview;
  glm::vec4 viewport;

  fusion(float width, float height, float z_near, float z_far): width(width), height(height){
    projection = glm::perspectiveFov<float>(PSEYE_FOV_BLUE_DOT, width, height, z_near, z_far);
    viewport = glm::vec4(0., 0., width, height);
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
  min_color = (color - threshold < 0) ? 0 : color - threshold;
  max_color = (color + threshold > 255) ? 255 : color + threshold;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psmove_tracker");

  ros::NodeHandle nh;
  int camera_id = 0;
  nh.getParam("camera_id", camera_id);
  int color_r = 255, color_g = 0, color_b = 255;
  nh.getParam("color_r", color_r);
  nh.getParam("color_g", color_g);
  nh.getParam("color_b", color_b);

  int color_threshold = 20; // 20;
  nh.getParam("color_threshold" , color_threshold);

  ros::Publisher pub = nh.advertise<geometry_msgs::Vector3Stamped>("/psmove_circle", 1000);

  int color_r_min, color_r_max;
  int color_g_min, color_g_max;
  int color_b_min, color_b_max;
  min_max_color(color_r, color_threshold, color_r_min, color_r_max);
  min_max_color(color_g, color_threshold, color_g_min, color_g_max);
  min_max_color(color_b, color_threshold, color_b_min, color_b_max);

  cv::VideoCapture cap(camera_id);
  if(!cap.isOpened()){
    ROS_FATAL_STREAM("failed to open camera");
    return -1;
  }

  //load calib data
  std::string intrinsics_xml = "/home/leus/.psmoveapi/intrinsics.xml";
  nh.getParam("intrinsics_xml", intrinsics_xml);
  cv::FileStorage intr_fs(intrinsics_xml, cv::FileStorage::READ);
  cv::Mat camera_matrix;
  if(!intr_fs.isOpened()) ROS_WARN_STREAM("failed to open intrinsics xml at: " << intrinsics_xml);
  intr_fs["Camera_matrix"] >> camera_matrix;
  intr_fs.release();

  std::string distortion_xml = "/home/leus/.psmoveapi/distortion.xml";
  nh.getParam("distortion_xml", distortion_xml);
  cv::FileStorage dist_fs(distortion_xml, cv::FileStorage::READ);
  cv::Mat dist_coeff;
  if(!dist_fs.isOpened()) ROS_WARN_STREAM("failed to open distortion xml at: " << distortion_xml);
  dist_fs["Distortion"] >> dist_coeff;
  dist_fs.release();
  
  cv::Size captureSize(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT));

  cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeff, captureSize, 1, captureSize, 0);

  ros::Rate loop_rate(1000);

  geometry_msgs::Vector3Stamped msg;

  psmove_init(PSMOVE_CURRENT_VERSION);
  PSMove *move = psmove_connect();
  
  cv::Point2f prev_centroid(0.f,0.f);
  buffered_vector prev_radius(3);
  float roi_margin_2 = 50;
  bool force_not_roi = false;

  fusion fusion(captureSize.width, captureSize.height, 1, 1000);

  while (ros::ok())
  {
    int res = psmove_poll(move);
    if (res)
    {
      psmove_set_leds(move, 255, 0, 255);
      psmove_update_leds(move);
    }


    cv::Mat raw_frame, frame, filtered, roi_image;
    cap >> raw_frame;

    cv::undistort(raw_frame, frame, camera_matrix, dist_coeff, new_camera_matrix);

    // roi
    bool is_roi = false;
    cv::Rect roi_rect;
    if (prev_radius.latest() <= 0 || force_not_roi){
      roi_image = frame;
      force_not_roi = false;
    }
    else {
      is_roi = true;
      roi_rect = cv::Rect(prev_centroid.x - prev_radius.latest() - roi_margin_2,
                        prev_centroid.y - prev_radius.latest() - roi_margin_2,
                        (prev_radius.latest() + roi_margin_2) * 2,
                        (prev_radius.latest() + roi_margin_2) * 2);
      if (roi_rect.x < 0) roi_rect.x = 0;
      if (roi_rect.y < 0) roi_rect.y = 0;
      if (roi_rect.x + roi_rect.width > frame.size().width) roi_rect.x = frame.size().width - roi_rect.width;
      if (roi_rect.y + roi_rect.height > frame.size().height) roi_rect.y = frame.size().height - roi_rect.height;
      roi_image = frame(roi_rect);

#ifdef DEBUG
      ROS_INFO_STREAM("roi x: " << roi_rect.x << " y: " << roi_rect.y << " w: " << roi_rect.width << " h: " << roi_rect.height);
#endif
    }

    // color filter
    cv::inRange(roi_image, cv::Scalar(color_b_min, 0, color_r_min), cv::Scalar(color_b_max, 255, color_r_max), filtered);
    cv::GaussianBlur(filtered, filtered, cv::Size(9,9), 2, 2);

    // circle detection
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(filtered, circles, CV_HOUGH_GRADIENT, 2, roi_image.rows / 4, 200, 40, 5, 200);

    float cent_x, cent_y;
    if (circles.size() <= 0) {
#ifdef DEBUG
      ROS_INFO_STREAM("no circle detected.");
#endif
      force_not_roi = true;
      continue;
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
    prev_radius.insert(max_radius);
    if (is_roi) {
      cent_x += roi_rect.x;
      cent_y += roi_rect.y;
    }
    prev_centroid.x = cent_x;
    prev_centroid.y = cent_y;
    float avg_radius = prev_radius.average();
#ifdef DEBUG
    ROS_INFO_STREAM("x: " << cent_x << " y: " << cent_y << " r: " << avg_radius);
#endif
    cv::circle( frame, cv::Point(cent_x, cent_y), avg_radius, cv::Scalar(0, 0, 255), 3, 8, 0);
#ifdef DEBUG
   cv::imshow("debug1", filtered);
   cv::imshow("debug2", frame);
    cvWaitKey(1);
#endif

    cv::Point3f pos = fusion.get_position(cent_x, cent_y, avg_radius);
#ifdef DEBUG
    ROS_INFO_STREAM("pos x: " << pos.x << " y: " << pos.y << " z: " << pos.z);
#endif

    msg.vector.x = cent_x;
    msg.vector.y = cent_y;
    msg.vector.z = avg_radius;
    pub.publish(msg);

    loop_rate.sleep();
  }

  return 0;
}
