#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus-add-srvs "std_srvs")

(ros::roseus "hoge")

(defun hoge ()
  (print "hoge"))

(ros::advertise-service "/test_service" std_srvs::Empty #'(lambda (s)
                                                            (hoge)
                                                            (instance std_srvs::EmptyResponse :init)))

(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
