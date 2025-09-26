// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from chapt4_interfaces:srv/FaceDetector.idl
// generated code does not contain a copyright notice

#ifndef CHAPT4_INTERFACES__SRV__DETAIL__FACE_DETECTOR__TRAITS_HPP_
#define CHAPT4_INTERFACES__SRV__DETAIL__FACE_DETECTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "chapt4_interfaces/srv/detail/face_detector__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__traits.hpp"

namespace chapt4_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FaceDetector_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: image
  {
    out << "image: ";
    to_flow_style_yaml(msg.image, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaceDetector_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: image
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "image:\n";
    to_block_style_yaml(msg.image, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaceDetector_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chapt4_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chapt4_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chapt4_interfaces::srv::FaceDetector_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  chapt4_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chapt4_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chapt4_interfaces::srv::FaceDetector_Request & msg)
{
  return chapt4_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chapt4_interfaces::srv::FaceDetector_Request>()
{
  return "chapt4_interfaces::srv::FaceDetector_Request";
}

template<>
inline const char * name<chapt4_interfaces::srv::FaceDetector_Request>()
{
  return "chapt4_interfaces/srv/FaceDetector_Request";
}

template<>
struct has_fixed_size<chapt4_interfaces::srv::FaceDetector_Request>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::Image>::value> {};

template<>
struct has_bounded_size<chapt4_interfaces::srv::FaceDetector_Request>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::Image>::value> {};

template<>
struct is_message<chapt4_interfaces::srv::FaceDetector_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace chapt4_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const FaceDetector_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: number
  {
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << ", ";
  }

  // member: use_time
  {
    out << "use_time: ";
    rosidl_generator_traits::value_to_yaml(msg.use_time, out);
    out << ", ";
  }

  // member: top
  {
    if (msg.top.size() == 0) {
      out << "top: []";
    } else {
      out << "top: [";
      size_t pending_items = msg.top.size();
      for (auto item : msg.top) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: right
  {
    if (msg.right.size() == 0) {
      out << "right: []";
    } else {
      out << "right: [";
      size_t pending_items = msg.right.size();
      for (auto item : msg.right) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: bottom
  {
    if (msg.bottom.size() == 0) {
      out << "bottom: []";
    } else {
      out << "bottom: [";
      size_t pending_items = msg.bottom.size();
      for (auto item : msg.bottom) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: left
  {
    if (msg.left.size() == 0) {
      out << "left: []";
    } else {
      out << "left: [";
      size_t pending_items = msg.left.size();
      for (auto item : msg.left) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FaceDetector_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "number: ";
    rosidl_generator_traits::value_to_yaml(msg.number, out);
    out << "\n";
  }

  // member: use_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_time: ";
    rosidl_generator_traits::value_to_yaml(msg.use_time, out);
    out << "\n";
  }

  // member: top
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.top.size() == 0) {
      out << "top: []\n";
    } else {
      out << "top:\n";
      for (auto item : msg.top) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.right.size() == 0) {
      out << "right: []\n";
    } else {
      out << "right:\n";
      for (auto item : msg.right) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bottom
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bottom.size() == 0) {
      out << "bottom: []\n";
    } else {
      out << "bottom:\n";
      for (auto item : msg.bottom) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.left.size() == 0) {
      out << "left: []\n";
    } else {
      out << "left:\n";
      for (auto item : msg.left) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FaceDetector_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace chapt4_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use chapt4_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const chapt4_interfaces::srv::FaceDetector_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  chapt4_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use chapt4_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const chapt4_interfaces::srv::FaceDetector_Response & msg)
{
  return chapt4_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<chapt4_interfaces::srv::FaceDetector_Response>()
{
  return "chapt4_interfaces::srv::FaceDetector_Response";
}

template<>
inline const char * name<chapt4_interfaces::srv::FaceDetector_Response>()
{
  return "chapt4_interfaces/srv/FaceDetector_Response";
}

template<>
struct has_fixed_size<chapt4_interfaces::srv::FaceDetector_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<chapt4_interfaces::srv::FaceDetector_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<chapt4_interfaces::srv::FaceDetector_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<chapt4_interfaces::srv::FaceDetector>()
{
  return "chapt4_interfaces::srv::FaceDetector";
}

template<>
inline const char * name<chapt4_interfaces::srv::FaceDetector>()
{
  return "chapt4_interfaces/srv/FaceDetector";
}

template<>
struct has_fixed_size<chapt4_interfaces::srv::FaceDetector>
  : std::integral_constant<
    bool,
    has_fixed_size<chapt4_interfaces::srv::FaceDetector_Request>::value &&
    has_fixed_size<chapt4_interfaces::srv::FaceDetector_Response>::value
  >
{
};

template<>
struct has_bounded_size<chapt4_interfaces::srv::FaceDetector>
  : std::integral_constant<
    bool,
    has_bounded_size<chapt4_interfaces::srv::FaceDetector_Request>::value &&
    has_bounded_size<chapt4_interfaces::srv::FaceDetector_Response>::value
  >
{
};

template<>
struct is_service<chapt4_interfaces::srv::FaceDetector>
  : std::true_type
{
};

template<>
struct is_service_request<chapt4_interfaces::srv::FaceDetector_Request>
  : std::true_type
{
};

template<>
struct is_service_response<chapt4_interfaces::srv::FaceDetector_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CHAPT4_INTERFACES__SRV__DETAIL__FACE_DETECTOR__TRAITS_HPP_
