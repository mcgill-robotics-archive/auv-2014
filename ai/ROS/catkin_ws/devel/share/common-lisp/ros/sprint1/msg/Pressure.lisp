; Auto-generated. Do not edit!


(cl:in-package sprint1-msg)


;//! \htmlinclude Pressure.msg.html

(cl:defclass <Pressure> (roslisp-msg-protocol:ros-message)
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

(cl:defclass Pressure (<Pressure>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Pressure>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Pressure)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sprint1-msg:<Pressure> is deprecated: use sprint1-msg:Pressure instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <Pressure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sprint1-msg:data-val is deprecated.  Use sprint1-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'raw-val :lambda-list '(m))
(cl:defmethod raw-val ((m <Pressure>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sprint1-msg:raw-val is deprecated.  Use sprint1-msg:raw instead.")
  (raw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Pressure>) ostream)
  "Serializes a message object of type '<Pressure>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Pressure>) istream)
  "Deserializes a message object of type '<Pressure>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Pressure>)))
  "Returns string type for a message object of type '<Pressure>"
  "sprint1/Pressure")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Pressure)))
  "Returns string type for a message object of type 'Pressure"
  "sprint1/Pressure")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Pressure>)))
  "Returns md5sum for a message object of type '<Pressure>"
  "d7776db48db9aec9e30d65f7c8fb2b51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Pressure)))
  "Returns md5sum for a message object of type 'Pressure"
  "d7776db48db9aec9e30d65f7c8fb2b51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Pressure>)))
  "Returns full string definition for message of type '<Pressure>"
  (cl:format cl:nil "float64 data~%float64 raw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Pressure)))
  "Returns full string definition for message of type 'Pressure"
  (cl:format cl:nil "float64 data~%float64 raw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Pressure>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Pressure>))
  "Converts a ROS message object to a list"
  (cl:list 'Pressure
    (cl:cons ':data (data msg))
    (cl:cons ':raw (raw msg))
))
