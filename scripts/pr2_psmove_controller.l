#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pr2_psmove_controller")

(require :pr2-interface "package://pr2eus/pr2-interface.l")

(pr2-init)

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))

(setq *initial-pose* nil)
(setq *move* nil)

(defun ros::point->float-vector (point)
  (float-vector (* (send point :y) 1000.0)
                (* (send point :x) 1000.0)
                (* (send point :z) -1000.0)))

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
        (base->arm (send *tfl* :lookup-transform "/base_footprint" "/l_forearm_link" (ros::time 0)))
        (diff-c (make-cube 100 100 100)))
    (if (not *move*) (return-from p-cb nil))
    (if (not base->arm) (return-from p-cb nil))
    (if (not *initial-pose*)
        (setq *initial-pose* c))
    (send diff-c :transform (send (send *initial-pose* :copy-worldcoords) :transformation c))
    (send diff-c :translate (send base->arm :pos))
    (ros::ros-info "psmove: ~A" c)
    (ros::ros-info "base->arm: ~A" (send base->arm :pos))
    (ros::ros-info "diff-c: ~A" diff-c)
    (ros::publish "/psmove_marker" (cube->marker-msg diff-c
                                                     (instance std_msgs::header :init
                                                               :stamp (ros::time-now)
                                                               :frame_id "/l_forearm_link")
                                                     :color (float-vector 0 1 0)
                                                     :lifetime 300))
    ))

(defun b-cb (msg)
  (let ((b-str (send msg :button))
        (trig (send msg :trigger)))
    (if (> trig 0.5)
        (setq *move* t)
      (setq *move* nil))
    (when (substringp "t" b-str)
      (send *ri* :larm :start-grasp))
    (when (substringp "c" b-str)
      (send *ri* :larm :stop-grasp))))

(ros::subscribe "/psmove" geometry_msgs::PoseStamped #'p-cb)
(ros::subscribe "/psmove_button" jsk_2014w_rinko::PSMoveButton #'b-cb)
(ros::advertise "/psmove_marker" visualization_msgs::Marker 5)

(ros::rate 100)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
