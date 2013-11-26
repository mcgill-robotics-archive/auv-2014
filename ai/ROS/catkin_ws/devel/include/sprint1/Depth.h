/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by genmsg_cpp from file /home/alex/catkin_ws/src/sprint1/msg/Depth.msg
 *
 */


#ifndef SPRINT1_MESSAGE_DEPTH_H
#define SPRINT1_MESSAGE_DEPTH_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace sprint1
{
template <class ContainerAllocator>
struct Depth_
{
  typedef Depth_<ContainerAllocator> Type;

  Depth_()
    : data(0.0)
    , raw(0.0)  {
    }
  Depth_(const ContainerAllocator& _alloc)
    : data(0.0)
    , raw(0.0)  {
    }



   typedef double _data_type;
  _data_type data;

   typedef double _raw_type;
  _raw_type raw;




  typedef boost::shared_ptr< ::sprint1::Depth_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::sprint1::Depth_<ContainerAllocator> const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;

}; // struct Depth_

typedef ::sprint1::Depth_<std::allocator<void> > Depth;

typedef boost::shared_ptr< ::sprint1::Depth > DepthPtr;
typedef boost::shared_ptr< ::sprint1::Depth const> DepthConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::sprint1::Depth_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::sprint1::Depth_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace sprint1

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sprint1': ['/home/alex/catkin_ws/src/sprint1/msg', '/home/alex/catkin_ws/src/sprint1/msg'], 'std_msgs': ['/opt/ros/hydro/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::sprint1::Depth_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::sprint1::Depth_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sprint1::Depth_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::sprint1::Depth_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sprint1::Depth_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::sprint1::Depth_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::sprint1::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d7776db48db9aec9e30d65f7c8fb2b51";
  }

  static const char* value(const ::sprint1::Depth_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd7776db48db9aec9ULL;
  static const uint64_t static_value2 = 0xe30d65f7c8fb2b51ULL;
};

template<class ContainerAllocator>
struct DataType< ::sprint1::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "sprint1/Depth";
  }

  static const char* value(const ::sprint1::Depth_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::sprint1::Depth_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 data\n\
float64 raw\n\
\n\
";
  }

  static const char* value(const ::sprint1::Depth_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::sprint1::Depth_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
      stream.next(m.raw);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Depth_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::sprint1::Depth_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::sprint1::Depth_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<double>::stream(s, indent + "  ", v.data);
    s << indent << "raw: ";
    Printer<double>::stream(s, indent + "  ", v.raw);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SPRINT1_MESSAGE_DEPTH_H
