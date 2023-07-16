// Generated by gencpp from file hkcamera_driver/PTZCmd.msg
// DO NOT EDIT!


#ifndef HKCAMERA_DRIVER_MESSAGE_PTZCMD_H
#define HKCAMERA_DRIVER_MESSAGE_PTZCMD_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace hkcamera_driver
{
template <class ContainerAllocator>
struct PTZCmd_
{
  typedef PTZCmd_<ContainerAllocator> Type;

  PTZCmd_()
    : cameraIndex(0)
    , PTZType(0)
    , speed(0)
    , time(0)  {
    }
  PTZCmd_(const ContainerAllocator& _alloc)
    : cameraIndex(0)
    , PTZType(0)
    , speed(0)
    , time(0)  {
  (void)_alloc;
    }



   typedef int32_t _cameraIndex_type;
  _cameraIndex_type cameraIndex;

   typedef int32_t _PTZType_type;
  _PTZType_type PTZType;

   typedef int32_t _speed_type;
  _speed_type speed;

   typedef int32_t _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::hkcamera_driver::PTZCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::hkcamera_driver::PTZCmd_<ContainerAllocator> const> ConstPtr;

}; // struct PTZCmd_

typedef ::hkcamera_driver::PTZCmd_<std::allocator<void> > PTZCmd;

typedef boost::shared_ptr< ::hkcamera_driver::PTZCmd > PTZCmdPtr;
typedef boost::shared_ptr< ::hkcamera_driver::PTZCmd const> PTZCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::hkcamera_driver::PTZCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::hkcamera_driver::PTZCmd_<ContainerAllocator1> & lhs, const ::hkcamera_driver::PTZCmd_<ContainerAllocator2> & rhs)
{
  return lhs.cameraIndex == rhs.cameraIndex &&
    lhs.PTZType == rhs.PTZType &&
    lhs.speed == rhs.speed &&
    lhs.time == rhs.time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::hkcamera_driver::PTZCmd_<ContainerAllocator1> & lhs, const ::hkcamera_driver::PTZCmd_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace hkcamera_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::hkcamera_driver::PTZCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::hkcamera_driver::PTZCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::hkcamera_driver::PTZCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4db5a9253c004fc0a5eb4629bca34970";
  }

  static const char* value(const ::hkcamera_driver::PTZCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4db5a9253c004fc0ULL;
  static const uint64_t static_value2 = 0xa5eb4629bca34970ULL;
};

template<class ContainerAllocator>
struct DataType< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "hkcamera_driver/PTZCmd";
  }

  static const char* value(const ::hkcamera_driver::PTZCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 cameraIndex\n"
"int32 PTZType\n"
"int32 speed\n"
"int32 time\n"
;
  }

  static const char* value(const ::hkcamera_driver::PTZCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cameraIndex);
      stream.next(m.PTZType);
      stream.next(m.speed);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PTZCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::hkcamera_driver::PTZCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::hkcamera_driver::PTZCmd_<ContainerAllocator>& v)
  {
    s << indent << "cameraIndex: ";
    Printer<int32_t>::stream(s, indent + "  ", v.cameraIndex);
    s << indent << "PTZType: ";
    Printer<int32_t>::stream(s, indent + "  ", v.PTZType);
    s << indent << "speed: ";
    Printer<int32_t>::stream(s, indent + "  ", v.speed);
    s << indent << "time: ";
    Printer<int32_t>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // HKCAMERA_DRIVER_MESSAGE_PTZCMD_H
