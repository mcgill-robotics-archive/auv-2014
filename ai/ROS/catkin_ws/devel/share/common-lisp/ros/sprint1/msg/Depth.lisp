; Auto-generated. Do not edit!


(cl:in-package sprint1-msg)


;//! \htmlinclude Depth.msg.html

(cl:defclass <Depth> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0)
   (raw
    :reader raw
    :initarg :raw
    :type cl:float
    :initform 0.0))
)

(cl:defclass Depth (<Depth>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Depth>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Depth)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sprint1-msg:<Depth> is deprecated: use sprint1-msg:Depth instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Depth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sprint1-msg:data-val is deprecated.  Use sprint1-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'raw-val :lambda-list '(m))
(cl:defmethod raw-val ((m <Depth>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sprint1-msg:raw-val is deprecated.  Use sprint1-msg:raw instead.")
  (raw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Depth>) ostream)
  "Serializes a message object of type '<Depth>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'raw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Depth>) istream)
  "Deserializes a message object of type '<Depth>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'raw) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Depth>)))
  "Returns string type for a message object of type '<Depth>"
  "sprint1/Depth")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Depth)))
  "Returns string type for a message object of type 'Depth"
  "sprint1/Depth")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Depth>)))
  "Returns md5sum for a message object of type '<Depth>"
  "d7776db48db9aec9e30d65f7c8fb2b51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Depth)))
  "Returns md5sum for a message object of type 'Depth"
  "d7776db48db9aec9e30d65f7c8fb2b51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Depth>)))
  "Returns full string definition for message of type '<Depth>"
  (cl:format cl:nil "float64 data~%float64 raw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Depth)))
  "Returns full string definition for message of type 'Depth"
  (cl:format cl:nil "float64 data~%float64 raw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Depth>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Depth>))
  "Converts a ROS message object to a list"
  (cl:list 'Depth
    (cl:cons ':data (data msg))
    (cl:cons ':raw (raw msg))
))
