#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus "psmove_marker_publisher")

(ros::advertise "/psmove_trajectory" visualization_msgs::Marker 5)
(ros::advertise "/psmove_trajectory_current" geometry_msgs::PoseStamped 5)

(ros::rate 30)

(setq i 0)

(setq *multiplier* (ros::get-param "~multiplier" 1.0f))

(defun ros::point->float-vector (point)
  (float-vector (* (send point :x) 1000.0 *multiplier*)
                (* (send point :y) 1000.0 *multiplier*)
                (* (send point :z) 1000.0 *multiplier*)))

(setq *prev-pos* nil)
(setq *write* nil)

(defun cb (msg)
  (let* ((pos (ros::point->float-vector (send msg :pose :position)))
         (line (if *prev-pos* (list pos *prev-pos*) (list pos pos)))
         (m (line->marker-msg line
                              (instance std_msgs::header :init
                                        :stamp (ros::time-now)
                                        :frame_id "/map")
                              :color (float-vector 0.9 0.0 0.9) ;; we lose
                              :id (incf i)
                              :lifetime 3000)))
;;    (send m :pose (send msg :pose))
    (setq *prev-pos* pos)
    (send msg :pose :position (ros::pos->tf-point pos))
    (ros::publish "/psmove_trajectory_current" msg)
    (if (not *write*) (return-from cb nil))
    (ros::publish "/psmove_trajectory" m)
))

(ros::subscribe "/psmove" geometry_msgs::PoseStamped #'cb)
(ros::subscribe "/psmove_button" jsk_2014w_rinko::PSMoveButton #'(lambda (m)
                                                      (if (> (send m :trigger) 0.5)
                                                             (setq *write* t)
                                                        (progn (setq *write* nil)
                                                               (setq *prev-pos* nil)))))

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
