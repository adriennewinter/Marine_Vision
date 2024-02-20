// Generated by gencpp from file vision_msgs/Classification3D.msg
// DO NOT EDIT!


#ifndef VISION_MSGS_MESSAGE_CLASSIFICATION3D_H
#define VISION_MSGS_MESSAGE_CLASSIFICATION3D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <vision_msgs/ObjectHypothesis.h>
#include <sensor_msgs/PointCloud2.h>

namespace vision_msgs
{
template <class ContainerAllocator>
struct Classification3D_
{
  typedef Classification3D_<ContainerAllocator> Type;

  Classification3D_()
    : header()
    , results()
    , source_cloud()  {
    }
  Classification3D_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , results(_alloc)
    , source_cloud(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> >> _results_type;
  _results_type results;

   typedef  ::sensor_msgs::PointCloud2_<ContainerAllocator>  _source_cloud_type;
  _source_cloud_type source_cloud;





  typedef boost::shared_ptr< ::vision_msgs::Classification3D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vision_msgs::Classification3D_<ContainerAllocator> const> ConstPtr;

}; // struct Classification3D_

typedef ::vision_msgs::Classification3D_<std::allocator<void> > Classification3D;

typedef boost::shared_ptr< ::vision_msgs::Classification3D > Classification3DPtr;
typedef boost::shared_ptr< ::vision_msgs::Classification3D const> Classification3DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vision_msgs::Classification3D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vision_msgs::Classification3D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vision_msgs::Classification3D_<ContainerAllocator1> & lhs, const ::vision_msgs::Classification3D_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.results == rhs.results &&
    lhs.source_cloud == rhs.source_cloud;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vision_msgs::Classification3D_<ContainerAllocator1> & lhs, const ::vision_msgs::Classification3D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vision_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::vision_msgs::Classification3D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vision_msgs::Classification3D_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision_msgs::Classification3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vision_msgs::Classification3D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_msgs::Classification3D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vision_msgs::Classification3D_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vision_msgs::Classification3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2c0fe97799b60ee2995363b3fbf44715";
  }

  static const char* value(const ::vision_msgs::Classification3D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2c0fe97799b60ee2ULL;
  static const uint64_t static_value2 = 0x995363b3fbf44715ULL;
};

template<class ContainerAllocator>
struct DataType< ::vision_msgs::Classification3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vision_msgs/Classification3D";
  }

  static const char* value(const ::vision_msgs::Classification3D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vision_msgs::Classification3D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Defines a 3D classification result.\n"
"#\n"
"# This result does not contain any position information. It is designed for\n"
"#   classifiers, which simply provide probabilities given a source image.\n"
"\n"
"Header header\n"
"\n"
"# Class probabilities\n"
"ObjectHypothesis[] results\n"
"\n"
"# The 3D data that generated these results (i.e. region proposal cropped out of\n"
"#   the image). Not required for all detectors, so it may be empty.\n"
"sensor_msgs/PointCloud2 source_cloud\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: vision_msgs/ObjectHypothesis\n"
"# An object hypothesis that contains no position information.\n"
"\n"
"# The unique numeric ID of object detected. To get additional information about\n"
"#   this ID, such as its human-readable name, listeners should perform a lookup\n"
"#   in a metadata database. See vision_msgs/VisionInfo.msg for more detail.\n"
"int64 id\n"
"\n"
"# The probability or confidence value of the detected object. By convention,\n"
"#   this value should lie in the range [0-1].\n"
"float64 score\n"
"================================================================================\n"
"MSG: sensor_msgs/PointCloud2\n"
"# This message holds a collection of N-dimensional points, which may\n"
"# contain additional information such as normals, intensity, etc. The\n"
"# point data is stored as a binary blob, its layout described by the\n"
"# contents of the \"fields\" array.\n"
"\n"
"# The point cloud data may be organized 2d (image-like) or 1d\n"
"# (unordered). Point clouds organized as 2d images may be produced by\n"
"# camera depth sensors such as stereo or time-of-flight.\n"
"\n"
"# Time of sensor data acquisition, and the coordinate frame ID (for 3d\n"
"# points).\n"
"Header header\n"
"\n"
"# 2D structure of the point cloud. If the cloud is unordered, height is\n"
"# 1 and width is the length of the point cloud.\n"
"uint32 height\n"
"uint32 width\n"
"\n"
"# Describes the channels and their layout in the binary data blob.\n"
"PointField[] fields\n"
"\n"
"bool    is_bigendian # Is this data bigendian?\n"
"uint32  point_step   # Length of a point in bytes\n"
"uint32  row_step     # Length of a row in bytes\n"
"uint8[] data         # Actual point data, size is (row_step*height)\n"
"\n"
"bool is_dense        # True if there are no invalid points\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/PointField\n"
"# This message holds the description of one point entry in the\n"
"# PointCloud2 message format.\n"
"uint8 INT8    = 1\n"
"uint8 UINT8   = 2\n"
"uint8 INT16   = 3\n"
"uint8 UINT16  = 4\n"
"uint8 INT32   = 5\n"
"uint8 UINT32  = 6\n"
"uint8 FLOAT32 = 7\n"
"uint8 FLOAT64 = 8\n"
"\n"
"string name      # Name of field\n"
"uint32 offset    # Offset from start of point struct\n"
"uint8  datatype  # Datatype enumeration, see above\n"
"uint32 count     # How many elements in the field\n"
;
  }

  static const char* value(const ::vision_msgs::Classification3D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vision_msgs::Classification3D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.results);
      stream.next(m.source_cloud);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Classification3D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vision_msgs::Classification3D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vision_msgs::Classification3D_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "results[]" << std::endl;
    for (size_t i = 0; i < v.results.size(); ++i)
    {
      s << indent << "  results[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::vision_msgs::ObjectHypothesis_<ContainerAllocator> >::stream(s, indent + "    ", v.results[i]);
    }
    s << indent << "source_cloud: ";
    s << std::endl;
    Printer< ::sensor_msgs::PointCloud2_<ContainerAllocator> >::stream(s, indent + "  ", v.source_cloud);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VISION_MSGS_MESSAGE_CLASSIFICATION3D_H
