#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus-add-msgs "geometry_msgs")

(ros::roseus "pr2_psmove_controller")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "models/arrow-object.l")

(pr2-init)

(setq *tfb* (instance ros::transform-broadcaster :init))
(setq *tfl* (instance ros::transform-listener :init))


(setq *move* nil)
(setq *initial-move* nil)
(setq *rot-val* 0.0f)
(setq *multiplier* (ros::get-param "~multiplier" 1.0f))
(setq *z-far* (ros::get-param "~z_far" 1.2f))

(defun ros::point->float-vector (point)
  (float-vector (* (send point :x) 1000.0 *multiplier*)
                (* (send point :y) 1000.0 *multiplier*)
                (* (send point :z) 1000.0 *multiplier*)))

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
        (base->arm (send *tfl* :lookup-transform "/base_footprint" "/torso_lift_link" (ros::time 0)))
        (diff-c (make-cube 40 40 40))
        (av-time 500)
        initial->current)
    (if (not base->arm) (return-from p-cb nil))

    (when *initial-move*
      (setq av-time 3000)
      (setq *initial-move* nil))

;;    (send diff-c :transform (send (send *initial-pose* :copy-worldcoords) :transformation c))
    (send diff-c :translate (send base->arm :pos) :world)
    (send diff-c :translate (float-vector (* *z-far* 1000) 0 0) :world)
    (send diff-c :translate (send c :pos) :world)
    (send diff-c :rotate (deg2rad *rot-val*) :z)
;;    (send diff-c :rotate (send c :rot) :world)
;;    (ros::ros-info "psmove: ~A" c)
;;    (ros::ros-info "base->arm: ~A" (send base->arm :pos))
;;    (ros::ros-info "diff-c: ~A" diff-c)
    (ros::publish "/psmove_marker" (cube->marker-msg diff-c
                                                     (instance std_msgs::header :init
                                                               :stamp (ros::time-now)
                                                               :frame_id "/base_footprint")
                                                     :color (float-vector 0 1 0)
                                                     :lifetime 300))
    (when *move*
      (send *pr2* :larm :inverse-kinematics diff-c
            :rotation-axis :x
            :stop 10)
      (send *ri* :larm-angle-vector (send *pr2* :angle-vector) av-time)
;;      (send *ri* :wait-interpolation))
      )
    ))

(defun b-cb (msg)
  (let ((b-str (send msg :button))
        (trig (send msg :trigger)))
    (when (> trig 0.5)
        (setq *move* t)
        (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
        )
    (when (< trig 0.5)
      (setq *initial-move* nil)
      (setq *move* nil))
    (when (substringp "t" b-str)
      (send *ri* :start-grasp :larm))
    (when (substringp "c" b-str)
      (send *ri* :stop-grasp :larm))
    (when (substringp "x" b-str)
      (send *pr2* :larm :wrist-r :joint-angle (+ (send *pr2* :larm :wrist-r :joint-angle)
                                                 50))
      (send *ri* :larm-angle-vector (send *pr2* :angle-vector) 500))
    (when (substringp "s" b-str)
      (setq *rot-val* (- *rot-val* 15)))
))

(ros::subscribe "/psmove" geometry_msgs::PoseStamped #'p-cb)
(ros::subscribe "/psmove_button" jsk_2014w_rinko::PSMoveButton #'b-cb)
(ros::advertise "/psmove_marker" visualization_msgs::Marker 5)

(ros::rate 15)

(while (ros::ok)
  (ros::spin-once)
  (ros::sleep))
