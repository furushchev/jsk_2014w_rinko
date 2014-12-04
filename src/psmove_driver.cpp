#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <psmove.h>
#include <psmove_tracker.h>
#include <psmove_fusion.h>

float deg2rad(int deg)
{
  if (abs(deg) == 180) return -M_PI;
  else return (float)(deg / 180.0f * M_PI);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "psmove_driver");
  ros::NodeHandle nh;
  // ros::Publisher imupub = nh.advertise<sensor_msgs::Imu>("/psmove_imu", 1000);
  // ros::Publisher magpub = nh.advertise<geometry_msgs::Vector3Stamped>("/psmove_mag", 1000);
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/psmove_orientation", 1000);

  if (!psmove_init(PSMOVE_CURRENT_VERSION)) {
    ROS_FATAL_STREAM("Failed to init PSMove API (wrong version?)");
    exit(1);
  }

  int controller_num = psmove_count_connected();
  ROS_INFO_STREAM("connected to " << controller_num << " ps move.");

  PSMove* move = psmove_connect();

  if (!move){
    ROS_FATAL_STREAM("failed to connect to default move controller.\n" << "Please connect via USB or BT.");
    exit(1);
  }

  char *serial = psmove_get_serial(move);
  ROS_INFO_STREAM("serial: " << serial);
  free(serial);

  enum PSMove_Connection_Type ctype = psmove_connection_type(move);
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

//  psmove_set_rate_limition(move,1);
  psmove_enable_orientation(move, PSMove_True);
  enum PSMove_Bool has_calib = psmove_has_calibration(move);
  ROS_INFO_STREAM("has calib? " << has_calib);

  // init tracker
  PSMoveTracker *tracker = psmove_tracker_new();
  psmove_tracker_set_mirror(tracker, PSMove_True);
  psmove_tracker_set_exposure(tracker, Exposure_HIGH);
  while (psmove_tracker_enable_with_color(tracker, move, 255, 0, 255) != Tracker_CALIBRATED);
  enum PSMove_Bool is_auto_update_leds = psmove_tracker_get_auto_update_leds(tracker, move);
  ROS_INFO_STREAM("auto update led: " << (is_auto_update_leds == PSMove_True) ? "enabled" : "disabled");
  unsigned char r, g, b;
  psmove_tracker_get_camera_color(tracker, move, &r, &g, &b);
  ROS_INFO_STREAM("Controller color: " << (int)r << " " << (int)g << " " << (int)b);

  // init fusion
  PSMoveFusion *fusion = psmove_fusion_new(tracker, 1, 1000);

  ros::Rate loop_rate(1000);
  ros::Time prev_time = ros::Time::now();

  float x_ = 0.0f, y_ = 0.0f, z_ = 0.0f;
//  float vx = 0.0f, vy = 0.0f, vz = 0.0f;
  while (ctype != Conn_USB && !(psmove_get_buttons(move) & Btn_PS) && ros::ok())
  {
    int res = psmove_poll(move);
    if (res)
    {
      int intensity = 150;
     psmove_set_leds(move, intensity, 0, intensity);
     psmove_update_leds(move);
      // sensor_msgs::Imu imumsg;
      // geometry_msgs::Vector3Stamped magmsg;
      // int ax, ay, az;
      // int gx, gy, gz;
      // int mx, my, mz;
      // psmove_get_accelerometer(move, &ax, &ay, &az);
      // psmove_get_gyroscope(move, &gx, &gy, &gz);
      // psmove_get_magnetometer(move, &mx, &my, &mz);

      // ROS_INFO_STREAM("ax: " << ax << " ay: " << ay << " az: " << az);
      // ROS_INFO_STREAM("gx: " << gx << " gy: " << gy << " gz: " << gz);
      // ROS_INFO_STREAM("mx: " << mx << " my: " << my << " mz: " << mz);

      // imumsg.linear_acceleration.x = (ax - 4700) * 1e-3;
      // imumsg.linear_acceleration.y = (ay - 4700) * 1e-3;
      // imumsg.linear_acceleration.z = (az - 4700) * 1e-3;
      // imumsg.angular_velocity.x = gx * 1e-3;
      // imumsg.angular_velocity.y = gy * 1e-3;
      // imumsg.angular_velocity.z = gz * 1e-3;
      // magmsg.vector.x = deg2rad(mx);
      // magmsg.vector.y = deg2rad(my);
      // magmsg.vector.z = deg2rad(mz);

      // imupub.publish(imumsg);
      // magpub.publish(magmsg);
//      ROS_INFO_STREAM("has orientation?: " << psmove_has_orientation(move));
      ros::Time now_time = ros::Time::now();
      double dt = (now_time - prev_time).toNSec() * 1e-9;

      psmove_tracker_update_image(tracker);
      psmove_tracker_update(tracker, NULL);

      // int ax, ay, az;
      // float multiplier = 0.001;
      // psmove_get_accelerometer(move, &ax, &ay, &az);
      // ROS_INFO_STREAM("ax: " << ax << " ay: " << ay << " az: " << az);
      // vx += ax * multiplier * dt;
      // vy += ay * multiplier * dt;
      // vz += (az - 4290) * multiplier * dt;
      // ROS_INFO_STREAM("vx: " << vx << " vy: " << vy << " vz: " << vz);
      // x_ += vx * dt;
      // y_ += vy * dt;
      // z_ += vz * dt;

     // psmove_tracker_get_frame(tracker);
     // cv::Ptr<IplImage> iplImage = reinterpret_cast< cv::Ptr<IplImage> >(psmove_tracker_get_frame(tracker));
     // if (iplImage) {
     //   ROS_INFO_STREAM("image");
     //   cv::Mat frame(iplImage);
     //   cv::imshow("debug" , frame);
     // }
      psmove_tracker_annotate(tracker);
      PSMoveTrackerRGBImage raw_image = psmove_tracker_get_image(tracker);
      ROS_INFO_STREAM("width: " << raw_image.width << ", height: " << raw_image.height);
      cv::Mat frame_rgb(cv::Size(raw_image.width, raw_image.height), CV_8UC3, (char*)raw_image.data);
      cv::Mat frame;
      cv::cvtColor(frame_rgb, frame, CV_RGB2BGR);
      cv::imshow("debug" , frame);
      cvWaitKey(1);

      psmove_fusion_get_position(fusion, move, &x_, &y_, &z_);


      //    ROS_INFO_STREAM("x: " << x_ << "y: " << y_ << " z: " << z_);
      float x, y, z, w;
      psmove_get_orientation(move, &w, &x, &y, &z);
      //ROS_INFO_STREAM("x: " << x << " y: " << y << " z: " << z << " w: " << w);
      geometry_msgs::PoseStamped msg;
      float multiplier = 0.1;
      msg.header.frame_id = "torso_lift_link";
      msg.pose.position.x = x_ * multiplier;
      msg.pose.position.y = y_ * multiplier;
      msg.pose.position.z = z_ * multiplier;
      msg.pose.orientation.x = x;
      msg.pose.orientation.y = y;
      msg.pose.orientation.z = z;
      msg.pose.orientation.w = w;
      pub.publish(msg);
      prev_time = now_time;
    }
    loop_rate.sleep();
  }

  psmove_disconnect(move);
  psmove_tracker_free(tracker);
  psmove_fusion_free(fusion);
  ROS_WARN_STREAM("!!exit!!");

  return 0;
}
