

#ifndef ROS_FOUR1_MSG_MESSAGE_FOUR1_H
#define ROS_FOUR1_MSG_MESSAGE_FOUR1_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ros_four1_msg
{
template <class ContainerAllocator>
struct four1_
{
  typedef four1_<ContainerAllocator> Type;

  four1_()
    : FLSpeed(0)
    , FRSpeed(0)
    , BLSpeed(0)
    , BRSpeed(0)
    , FLAddEN(0)
    , FRAddEN(0)
    , BLAddEN(0)
    , BRAddEN(0)
    , Voltage(0)
    , State(0)  {
    }
  four1_(const ContainerAllocator& _alloc)
    : FLSpeed(0)
    , FRSpeed(0)
    , BLSpeed(0)
    , BRSpeed(0)
    , FLAddEN(0)
    , FRAddEN(0)
    , BLAddEN(0)
    , BRAddEN(0)
    , Voltage(0)
    , State(0)  {
  (void)_alloc;
    }



   typedef int16_t _FLSpeed_type;
  _FLSpeed_type FLSpeed;

   typedef int16_t _FRSpeed_type;
  _FRSpeed_type FRSpeed;

   typedef int16_t _BLSpeed_type;
  _BLSpeed_type BLSpeed;

   typedef int16_t _BRSpeed_type;
  _BRSpeed_type BRSpeed;

   typedef int16_t _FLAddEN_type;
  _FLAddEN_type FLAddEN;

   typedef int16_t _FRAddEN_type;
  _FRAddEN_type FRAddEN;

   typedef int16_t _BLAddEN_type;
  _BLAddEN_type BLAddEN;

   typedef int16_t _BRAddEN_type;
  _BRAddEN_type BRAddEN;

   typedef int16_t _Voltage_type;
  _Voltage_type Voltage;

   typedef int16_t _State_type;
  _State_type State;





  typedef boost::shared_ptr< ::ros_four1_msg::four1_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ros_four1_msg::four1_<ContainerAllocator> const> ConstPtr;

}; // struct four1_

typedef ::ros_four1_msg::four1_<std::allocator<void> > four1;

typedef boost::shared_ptr< ::ros_four1_msg::four1 > four1Ptr;
typedef boost::shared_ptr< ::ros_four1_msg::four1 const> four1ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ros_four1_msg::four1_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ros_four1_msg::four1_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ros_four1_msg::four1_<ContainerAllocator1> & lhs, const ::ros_four1_msg::four1_<ContainerAllocator2> & rhs)
{
  return lhs.FLSpeed == rhs.FLSpeed &&
    lhs.FRSpeed == rhs.FRSpeed &&
    lhs.BLSpeed == rhs.BLSpeed &&
    lhs.BRSpeed == rhs.BRSpeed &&
    lhs.FLAddEN == rhs.FLAddEN &&
    lhs.FRAddEN == rhs.FRAddEN &&
    lhs.BLAddEN == rhs.BLAddEN &&
    lhs.BRAddEN == rhs.BRAddEN &&
    lhs.Voltage == rhs.Voltage &&
    lhs.State == rhs.State;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ros_four1_msg::four1_<ContainerAllocator1> & lhs, const ::ros_four1_msg::four1_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ros_four1_msg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::ros_four1_msg::four1_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ros_four1_msg::four1_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_four1_msg::four1_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ros_four1_msg::four1_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_four1_msg::four1_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ros_four1_msg::four1_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ros_four1_msg::four1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9ba363db5d7ce11d953645115c5f86be";
  }

  static const char* value(const ::ros_four1_msg::four1_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9ba363db5d7ce11dULL;
  static const uint64_t static_value2 = 0x953645115c5f86beULL;
};

template<class ContainerAllocator>
struct DataType< ::ros_four1_msg::four1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ros_four1_msg/four1";
  }

  static const char* value(const ::ros_four1_msg::four1_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ros_four1_msg::four1_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 FLSpeed	# 左前轮 mm/s\n"
"int16 FRSpeed	# 右前轮\n"
"int16 BLSpeed	# 左后轮\n"
"int16 BRSpeed	# 右后轮\n"
"int16 FLAddEN 	# 左前轮编码线数\n"
"int16 FRAddEN \n"
"int16 BLAddEN 	\n"
"int16 BRAddEN\n"
"int16 Voltage	# 电压 216=21.6v\n"
"int16 State    # \n"
"\n"
"\n"
;
  }

  static const char* value(const ::ros_four1_msg::four1_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ros_four1_msg::four1_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.FLSpeed);
      stream.next(m.FRSpeed);
      stream.next(m.BLSpeed);
      stream.next(m.BRSpeed);
      stream.next(m.FLAddEN);
      stream.next(m.FRAddEN);
      stream.next(m.BLAddEN);
      stream.next(m.BRAddEN);
      stream.next(m.Voltage);
      stream.next(m.State);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct four1_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ros_four1_msg::four1_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ros_four1_msg::four1_<ContainerAllocator>& v)
  {
    s << indent << "FLSpeed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.FLSpeed);
    s << indent << "FRSpeed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.FRSpeed);
    s << indent << "BLSpeed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.BLSpeed);
    s << indent << "BRSpeed: ";
    Printer<int16_t>::stream(s, indent + "  ", v.BRSpeed);
    s << indent << "FLAddEN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.FLAddEN);
    s << indent << "FRAddEN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.FRAddEN);
    s << indent << "BLAddEN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.BLAddEN);
    s << indent << "BRAddEN: ";
    Printer<int16_t>::stream(s, indent + "  ", v.BRAddEN);
    s << indent << "Voltage: ";
    Printer<int16_t>::stream(s, indent + "  ", v.Voltage);
    s << indent << "State: ";
    Printer<int16_t>::stream(s, indent + "  ", v.State);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROS_FOUR1_MSG_MESSAGE_FOUR1_H
