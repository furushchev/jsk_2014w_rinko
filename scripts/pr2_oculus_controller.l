#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_2014w_rinko")
(ros::roseus "pr2_oculus_controller")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(pr2-init)

(setq *tfl* (instance ros::transform-listener :init))

(setq *initial-pose* nil)
(setq *prev-time* nil)

(defun on-spin ()
  (let ((map->oculus (send *tfl* :lookup-transform "/map" "/oculus" (ros::time 0)))
        oculus-rot oculus-p oculus-y diff-p diff-y)
    (when map->oculus
      (setq oculus-rot (send (send map->oculus :copy-worldcoords) :locate (float-vector 0 0 0)))
      (format t "rot: ~A~%" (send oculus-rot :rpy-angle))
      (setq oculus-y (caar (send oculus-rot :rpy-angle)))
      (setq oculus-p (cadar (send oculus-rot :rpy-angle)))
      (when (not *initial-pose*)
        (setq *initial-pose* (list oculus-p oculus-y))
        (return-from on-spin nil))
      (setq diff-p (- oculus-p (car *initial-pose*)))
      (setq diff-y (- oculus-y (cadr *initial-pose*)))
      (format t "diff-p: ~A, diff-y: ~A~%" diff-p diff-y)
      (send *pr2* :head :neck-p :joint-angle (rad2deg diff-p))
      (send *pr2* :head :neck-y :joint-angle (rad2deg diff-y))
      (send *ri* :head-angle-vector (send *pr2* :angle-vector) 700))))
#|
(defun on-spin-old ()
  (let ((map->oculus (send *tfl* :lookup-transform "/base_footprint" "/oculus" (ros::time 0)))
        head-v
        diff-time
        base_footprint->torso_lift_link)
    (when (and map->oculus (not *initial-pose*))
      (format t "map->oculus: ~A~%" map->oculus)
      (setq *initial-pose* map->oculus)
      (setq *prev-time* (ros::time-now)))
    (when (and map->oculus *initial-pose*)
      (setq head-v (send (send *initial-pose* :copy-worldcoords) :transformation map->oculus))
      (format t "map->oculus: ~A~%" map->oculus)
      (format t "diff: ~A~%" head-v)
      (send head-v :locate #f(1000 0 0))
;;      (setq torso_lift_link->view_target (send *tfl* :transform-pose "torso_lift_link"  head-v))
;;      (setq base_footprint->torso_lift_link (send *tfl* :lookup-transform "/base_footprint" "/torso_lift_link" (ros::time 0)))
      (if (not base_footprint->torso_lift_link) (setq base_footprint->torso_lift_link (make-coords :pos #f(0 0 1300))))
      (format t "base->torso: ~A~%" base_footprint->torso_lift_link)
      (send head-v :translate (send base_footprint->torso_lift_link :pos) :world)
      (send *tfb* :send-transform (send head-v :worldcoords) "/base_footprint" "/oculus_view_target")
      (setq diff-time (ros::time- (ros::time-now) *prev-time*))
      (setq *prev-time* (ros::time-now))
      (send *pr2* :head :look-at (send head-v :worldpos))
      (send *ri* :head-angle-vector (send *pr2* :angle-vector) 200)
      (format t "head-v: ~A~%" (send head-v :worldcoords)))))
|#

(ros::rate 10)

(while (ros::ok)
  (ros::spin-once)
  (on-spin)
  (ros::sleep))
