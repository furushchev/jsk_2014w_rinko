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
      (send *ri* :head-angle-vector (send *pr2* :angle-vector) 700))
    ))

(ros::advertise-service "/reset_oculus_orientation" std_srvs::Empty #'(lambda ()
                                                                        (setq *initial-pose* nil)
                                                                        (instance std_srvs::EmptyResponse :init)))

(ros::rate 30)

(while (ros::ok)
  (ros::spin-once)
  (on-spin)
  (ros::sleep))
