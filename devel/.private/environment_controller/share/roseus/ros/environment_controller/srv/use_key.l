;; Auto-generated. Do not edit!


(when (boundp 'environment_controller::use_key)
  (if (not (find-package "ENVIRONMENT_CONTROLLER"))
    (make-package "ENVIRONMENT_CONTROLLER"))
  (shadow 'use_key (find-package "ENVIRONMENT_CONTROLLER")))
(unless (find-package "ENVIRONMENT_CONTROLLER::USE_KEY")
  (make-package "ENVIRONMENT_CONTROLLER::USE_KEY"))
(unless (find-package "ENVIRONMENT_CONTROLLER::USE_KEYREQUEST")
  (make-package "ENVIRONMENT_CONTROLLER::USE_KEYREQUEST"))
(unless (find-package "ENVIRONMENT_CONTROLLER::USE_KEYRESPONSE")
  (make-package "ENVIRONMENT_CONTROLLER::USE_KEYRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass environment_controller::use_keyRequest
  :super ros::object
  :slots (_door_loc ))

(defmethod environment_controller::use_keyRequest
  (:init
   (&key
    ((:door_loc __door_loc) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _door_loc __door_loc)
   self)
  (:door_loc
   (&rest __door_loc)
   (if (keywordp (car __door_loc))
       (send* _door_loc __door_loc)
     (progn
       (if __door_loc (setq _door_loc (car __door_loc)))
       _door_loc)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _door_loc
    (send _door_loc :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _door_loc
       (send _door_loc :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _door_loc
     (send _door_loc :deserialize buf ptr-) (incf ptr- (send _door_loc :serialization-length))
   ;;
   self)
  )

(defclass environment_controller::use_keyResponse
  :super ros::object
  :slots (_success ))

(defmethod environment_controller::use_keyResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass environment_controller::use_key
  :super ros::object
  :slots ())

(setf (get environment_controller::use_key :md5sum-) "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(setf (get environment_controller::use_key :datatype-) "environment_controller/use_key")
(setf (get environment_controller::use_key :request) environment_controller::use_keyRequest)
(setf (get environment_controller::use_key :response) environment_controller::use_keyResponse)

(defmethod environment_controller::use_keyRequest
  (:response () (instance environment_controller::use_keyResponse :init)))

(setf (get environment_controller::use_keyRequest :md5sum-) "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(setf (get environment_controller::use_keyRequest :datatype-) "environment_controller/use_keyRequest")
(setf (get environment_controller::use_keyRequest :definition-)
      "geometry_msgs/Point door_loc

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")

(setf (get environment_controller::use_keyResponse :md5sum-) "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(setf (get environment_controller::use_keyResponse :datatype-) "environment_controller/use_keyResponse")
(setf (get environment_controller::use_keyResponse :definition-)
      "geometry_msgs/Point door_loc

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
bool success

")



(provide :environment_controller/use_key "31c1bb88ea2fc30c8bbcc2144b52d6f7")


