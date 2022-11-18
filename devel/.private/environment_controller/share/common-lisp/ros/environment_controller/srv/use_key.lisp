; Auto-generated. Do not edit!


(cl:in-package environment_controller-srv)


;//! \htmlinclude use_key-request.msg.html

(cl:defclass <use_key-request> (roslisp-msg-protocol:ros-message)
  ((door_loc
    :reader door_loc
    :initarg :door_loc
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass use_key-request (<use_key-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <use_key-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'use_key-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name environment_controller-srv:<use_key-request> is deprecated: use environment_controller-srv:use_key-request instead.")))

(cl:ensure-generic-function 'door_loc-val :lambda-list '(m))
(cl:defmethod door_loc-val ((m <use_key-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader environment_controller-srv:door_loc-val is deprecated.  Use environment_controller-srv:door_loc instead.")
  (door_loc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <use_key-request>) ostream)
  "Serializes a message object of type '<use_key-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'door_loc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <use_key-request>) istream)
  "Deserializes a message object of type '<use_key-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'door_loc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<use_key-request>)))
  "Returns string type for a service object of type '<use_key-request>"
  "environment_controller/use_keyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'use_key-request)))
  "Returns string type for a service object of type 'use_key-request"
  "environment_controller/use_keyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<use_key-request>)))
  "Returns md5sum for a message object of type '<use_key-request>"
  "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'use_key-request)))
  "Returns md5sum for a message object of type 'use_key-request"
  "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<use_key-request>)))
  "Returns full string definition for message of type '<use_key-request>"
  (cl:format cl:nil "geometry_msgs/Point door_loc~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'use_key-request)))
  "Returns full string definition for message of type 'use_key-request"
  (cl:format cl:nil "geometry_msgs/Point door_loc~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <use_key-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'door_loc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <use_key-request>))
  "Converts a ROS message object to a list"
  (cl:list 'use_key-request
    (cl:cons ':door_loc (door_loc msg))
))
;//! \htmlinclude use_key-response.msg.html

(cl:defclass <use_key-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass use_key-response (<use_key-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <use_key-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'use_key-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name environment_controller-srv:<use_key-response> is deprecated: use environment_controller-srv:use_key-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <use_key-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader environment_controller-srv:success-val is deprecated.  Use environment_controller-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <use_key-response>) ostream)
  "Serializes a message object of type '<use_key-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <use_key-response>) istream)
  "Deserializes a message object of type '<use_key-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<use_key-response>)))
  "Returns string type for a service object of type '<use_key-response>"
  "environment_controller/use_keyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'use_key-response)))
  "Returns string type for a service object of type 'use_key-response"
  "environment_controller/use_keyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<use_key-response>)))
  "Returns md5sum for a message object of type '<use_key-response>"
  "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'use_key-response)))
  "Returns md5sum for a message object of type 'use_key-response"
  "31c1bb88ea2fc30c8bbcc2144b52d6f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<use_key-response>)))
  "Returns full string definition for message of type '<use_key-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'use_key-response)))
  "Returns full string definition for message of type 'use_key-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <use_key-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <use_key-response>))
  "Converts a ROS message object to a list"
  (cl:list 'use_key-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'use_key)))
  'use_key-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'use_key)))
  'use_key-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'use_key)))
  "Returns string type for a service object of type '<use_key>"
  "environment_controller/use_key")