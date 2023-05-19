; Auto-generated. Do not edit!


(cl:in-package astarfun-msg)


;//! \htmlinclude path_ok.msg.html

(cl:defclass <path_ok> (roslisp-msg-protocol:ros-message)
  ((path_ok
    :reader path_ok
    :initarg :path_ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass path_ok (<path_ok>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <path_ok>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'path_ok)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name astarfun-msg:<path_ok> is deprecated: use astarfun-msg:path_ok instead.")))

(cl:ensure-generic-function 'path_ok-val :lambda-list '(m))
(cl:defmethod path_ok-val ((m <path_ok>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader astarfun-msg:path_ok-val is deprecated.  Use astarfun-msg:path_ok instead.")
  (path_ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <path_ok>) ostream)
  "Serializes a message object of type '<path_ok>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'path_ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <path_ok>) istream)
  "Deserializes a message object of type '<path_ok>"
    (cl:setf (cl:slot-value msg 'path_ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<path_ok>)))
  "Returns string type for a message object of type '<path_ok>"
  "astarfun/path_ok")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'path_ok)))
  "Returns string type for a message object of type 'path_ok"
  "astarfun/path_ok")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<path_ok>)))
  "Returns md5sum for a message object of type '<path_ok>"
  "276f6ed1d160276c34d8bfda3a611a63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'path_ok)))
  "Returns md5sum for a message object of type 'path_ok"
  "276f6ed1d160276c34d8bfda3a611a63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<path_ok>)))
  "Returns full string definition for message of type '<path_ok>"
  (cl:format cl:nil "bool path_ok~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'path_ok)))
  "Returns full string definition for message of type 'path_ok"
  (cl:format cl:nil "bool path_ok~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <path_ok>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <path_ok>))
  "Converts a ROS message object to a list"
  (cl:list 'path_ok
    (cl:cons ':path_ok (path_ok msg))
))
