// Generated by gencpp from file environment_controller/use_keyRequest.msg
// DO NOT EDIT!


#ifndef ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEYREQUEST_H
#define ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEYREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace environment_controller
{
template <class ContainerAllocator>
struct use_keyRequest_
{
  typedef use_keyRequest_<ContainerAllocator> Type;

  use_keyRequest_()
    : door_loc()  {
    }
  use_keyRequest_(const ContainerAllocator& _alloc)
    : door_loc(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _door_loc_type;
  _door_loc_type door_loc;





  typedef boost::shared_ptr< ::environment_controller::use_keyRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::environment_controller::use_keyRequest_<ContainerAllocator> const> ConstPtr;

}; // struct use_keyRequest_

typedef ::environment_controller::use_keyRequest_<std::allocator<void> > use_keyRequest;

typedef boost::shared_ptr< ::environment_controller::use_keyRequest > use_keyRequestPtr;
typedef boost::shared_ptr< ::environment_controller::use_keyRequest const> use_keyRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::environment_controller::use_keyRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::environment_controller::use_keyRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::environment_controller::use_keyRequest_<ContainerAllocator1> & lhs, const ::environment_controller::use_keyRequest_<ContainerAllocator2> & rhs)
{
  return lhs.door_loc == rhs.door_loc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::environment_controller::use_keyRequest_<ContainerAllocator1> & lhs, const ::environment_controller::use_keyRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace environment_controller

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::environment_controller::use_keyRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::environment_controller::use_keyRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::environment_controller::use_keyRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::environment_controller::use_keyRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::environment_controller::use_keyRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::environment_controller::use_keyRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::environment_controller::use_keyRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3ef7d5d942900fab8f329a5d05ac1650";
  }

  static const char* value(const ::environment_controller::use_keyRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3ef7d5d942900fabULL;
  static const uint64_t static_value2 = 0x8f329a5d05ac1650ULL;
};

template<class ContainerAllocator>
struct DataType< ::environment_controller::use_keyRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "environment_controller/use_keyRequest";
  }

  static const char* value(const ::environment_controller::use_keyRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::environment_controller::use_keyRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point door_loc\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::environment_controller::use_keyRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::environment_controller::use_keyRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.door_loc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct use_keyRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::environment_controller::use_keyRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::environment_controller::use_keyRequest_<ContainerAllocator>& v)
  {
    s << indent << "door_loc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.door_loc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ENVIRONMENT_CONTROLLER_MESSAGE_USE_KEYREQUEST_H
