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
(setq *base-frame-id* (ros::get-param "~base_frame_id" "/base_footprint"))
(setq *z-far* (ros::get-param "~z_far" 1.2f))
(setq arm-string (ros::get-param "~use_arm" ":larm"))
(setq *use-arm* (read-from-string arm-string))
(setq *use-arm-av* (read-from-string (concatenate string arm-string "-angle-vector")))
(setq *y-offset* (* (ros::get-param "~y_offset" 0.0f) 1000))
(setq *ns* (ros::get-param "~ns" ""))

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
        (base->arm (send *tfl* :lookup-transform *base-frame-id* "/torso_lift_link" (ros::time 0)))
        (diff-c (make-cube 40 40 40))
        (av-time 500))

    (if (not base->arm)
        (setq base->arm (make-coords :pos (float-vector 0 0 1300))))

    (when *initial-move*
      (setq av-time 3000)
      (setq *initial-move* nil))

    (send diff-c :translate (send base->arm :pos) :world)
    (send diff-c :translate (float-vector (* *z-far* 1000) 0 0) :world)
    (send diff-c :translate (send c :pos) :world)
    (send diff-c :translate (float-vector 0 *y-offset* 0) :world)
    (send diff-c :rotate (deg2rad *rot-val*) :z)
    (ros::publish "/psmove_marker" (instance geometry_msgs::PoseStamped :init
                                             :header (instance std_msgs::header :init
                                                               :stamp (ros::time-now)
                                                               :frame_id *base-frame-id*)
                                             :pose (ros::coords->tf-pose diff-c)))
    (when *move*
      (send *pr2* *use-arm* :inverse-kinematics diff-c
            :rotation-axis :x
            :stop 10)
      (send *ri* *use-arm-av* (send *pr2* :angle-vector) av-time)
      )
    ))

(defun trigger-on ()
  (setq *move* t)
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (setq *initial-move* t))

(defun trigger-off ()
  (setq *move* nil))

(setq *trigger-on* nil)
(setq *reverse* 1)
(defun b-cb (msg)
  (let ((b-str (send msg :button))
        (trig (send msg :trigger)))
    (when (> trig 0.5)
      (when (not *trigger-on*)
        (trigger-on)
        (setq *trigger-on* t)))
    (when (< trig 0.5)
      (when *trigger-on*
        (setq *trigger-on* nil)
        (trigger-off)))
    (when (substringp "t" b-str)
      (send *ri* :start-grasp *use-arm*))
    (when (substringp "c" b-str)
      (send *ri* :stop-grasp *use-arm*))
    (when (substringp "x" b-str)
      (send *pr2* *use-arm* :wrist-r :joint-angle (+ (send *pr2* *use-arm* :wrist-r :joint-angle)
                                                     (* *reverse* 30)))
      (send *ri* *use-arm-av* (send *pr2* :angle-vector) 500))
    (when (substringp "s" b-str)
      (setq *rot-val* (+ *rot-val* (* *reverse* 15))))
    (when (substringp "m" b-str)
      (setq *reverse* (* -1 *reverse*)))
))

(ros::subscribe (concatenate string *ns* "/psmove") geometry_msgs::PoseStamped #'p-cb)
(ros::subscribe (concatenate string *ns* "/psmove_button") jsk_2014w_rinko::PSMoveButton #'b-cb)
(ros::advertise (concatenate string *ns* "/psmove_marker") geometry_msgs::PoseStamped 1)

(ros::rate 30)
(ros::spin)
