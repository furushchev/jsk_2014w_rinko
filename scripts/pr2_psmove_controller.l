#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pr2_psmove_controller")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(pr2-init)

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))

(defun ros::point->float-vector (point)
  (float-vector (* (send point :x) 1000.0)
                (* (send point :y) 1000.0)
                (* (send point :z) 1000.0)))

(defun ros::quaternion->matrix (quaternion)
  (user::quaternion2matrix
   (float-vector (send quaternion :w)
                 (send quaternion :x)
                 (send quaternion :y) 
                 (send quaternion :z))))

(defun p-cb (msg)
  (let ((c (make-coords
            :pos (ros::point->float-vector (send msg :pose :position))
            :rot (ros::quaternion->matrix (send msg :pose :orientation))))
        (base->torso (send *tfl* :lookup-transform "/base_footprint" "/torso_lift_link" (ros::time 0))))
    (ros::ros-info "psmove: ~A~%" c)
    (ros::ros-info "base->torso: ~A~%" (send base->torso :pos))
    (send c :translate (send base->torso :pos))
    (send *tfb* :send-transform (send c :worldcoords) "/torso_lift_link" "/psmove_target")))


(ros::subscribe "/psmove_orientation" geometry_msgs::PoseStamped #'p-cb)

(ros::rate 100)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
