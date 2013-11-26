; Auto-generated. Do not edit!


(cl:in-package sprint1-msg)


;//! \htmlinclude CVQuery.msg.html

(cl:defclass <CVQuery> (roslisp-msg-protocol:ros-message)
  ((queries
    :reader queries
    :initarg :queries
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass CVQuery (<CVQuery>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CVQuery>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CVQuery)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sprint1-msg:<CVQuery> is deprecated: use sprint1-msg:CVQuery instead.")))

(cl:ensure-generic-function 'queries-val :lambda-list '(m))
(cl:defmethod queries-val ((m <CVQuery>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sprint1-msg:queries-val is deprecated.  Use sprint1-msg:queries instead.")
  (queries m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CVQuery>) ostream)
  "Serializes a message object of type '<CVQuery>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'queries))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'queries))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CVQuery>) istream)
  "Deserializes a message object of type '<CVQuery>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'queries) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'queries)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CVQuery>)))
  "Returns string type for a message object of type '<CVQuery>"
  "sprint1/CVQuery")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CVQuery)))
  "Returns string type for a message object of type 'CVQuery"
  "sprint1/CVQuery")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CVQuery>)))
  "Returns md5sum for a message object of type '<CVQuery>"
  "e4e1d8ac850a18ec9ac2612323094057")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CVQuery)))
  "Returns md5sum for a message object of type 'CVQuery"
  "e4e1d8ac850a18ec9ac2612323094057")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CVQuery>)))
  "Returns full string definition for message of type '<CVQuery>"
  (cl:format cl:nil "int8[] queries~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CVQuery)))
  "Returns full string definition for message of type 'CVQuery"
  (cl:format cl:nil "int8[] queries~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CVQuery>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'queries) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CVQuery>))
  "Converts a ROS message object to a list"
  (cl:list 'CVQuery
    (cl:cons ':queries (queries msg))
))
