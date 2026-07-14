// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/reference_traj.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__TRAITS_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hamr_interfaces/msg/detail/reference_traj__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hamr_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReferenceTraj & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: x_dot
  {
    out << "x_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_dot, out);
    out << ", ";
  }

  // member: y_dot
  {
    out << "y_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_dot, out);
    out << ", ";
  }

  // member: roll_dot
  {
    out << "roll_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_dot, out);
    out << ", ";
  }

  // member: pitch_dot
  {
    out << "pitch_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_dot, out);
    out << ", ";
  }

  // member: yaw_dot
  {
    out << "yaw_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dot, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReferenceTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: x_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.x_dot, out);
    out << "\n";
  }

  // member: y_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.y_dot, out);
    out << "\n";
  }

  // member: roll_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.roll_dot, out);
    out << "\n";
  }

  // member: pitch_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch_dot, out);
    out << "\n";
  }

  // member: yaw_dot
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_dot: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_dot, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReferenceTraj & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hamr_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hamr_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hamr_interfaces::msg::ReferenceTraj & msg,
  std::ostream & out, size_t indentation = 0)
{
  hamr_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hamr_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hamr_interfaces::msg::ReferenceTraj & msg)
{
  return hamr_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hamr_interfaces::msg::ReferenceTraj>()
{
  return "hamr_interfaces::msg::ReferenceTraj";
}

template<>
inline const char * name<hamr_interfaces::msg::ReferenceTraj>()
{
  return "hamr_interfaces/msg/ReferenceTraj";
}

template<>
struct has_fixed_size<hamr_interfaces::msg::ReferenceTraj>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hamr_interfaces::msg::ReferenceTraj>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hamr_interfaces::msg::ReferenceTraj>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__TRAITS_HPP_
