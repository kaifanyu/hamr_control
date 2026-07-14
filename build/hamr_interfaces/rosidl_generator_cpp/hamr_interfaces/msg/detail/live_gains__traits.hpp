// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/live_gains.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__TRAITS_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hamr_interfaces/msg/detail/live_gains__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hamr_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const LiveGains & msg,
  std::ostream & out)
{
  out << "{";
  // member: p_x
  {
    out << "p_x: ";
    rosidl_generator_traits::value_to_yaml(msg.p_x, out);
    out << ", ";
  }

  // member: d_x
  {
    out << "d_x: ";
    rosidl_generator_traits::value_to_yaml(msg.d_x, out);
    out << ", ";
  }

  // member: i_x
  {
    out << "i_x: ";
    rosidl_generator_traits::value_to_yaml(msg.i_x, out);
    out << ", ";
  }

  // member: p_y
  {
    out << "p_y: ";
    rosidl_generator_traits::value_to_yaml(msg.p_y, out);
    out << ", ";
  }

  // member: d_y
  {
    out << "d_y: ";
    rosidl_generator_traits::value_to_yaml(msg.d_y, out);
    out << ", ";
  }

  // member: i_y
  {
    out << "i_y: ";
    rosidl_generator_traits::value_to_yaml(msg.i_y, out);
    out << ", ";
  }

  // member: p_yaw
  {
    out << "p_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.p_yaw, out);
    out << ", ";
  }

  // member: d_yaw
  {
    out << "d_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.d_yaw, out);
    out << ", ";
  }

  // member: i_yaw
  {
    out << "i_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.i_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LiveGains & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: p_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_x: ";
    rosidl_generator_traits::value_to_yaml(msg.p_x, out);
    out << "\n";
  }

  // member: d_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_x: ";
    rosidl_generator_traits::value_to_yaml(msg.d_x, out);
    out << "\n";
  }

  // member: i_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_x: ";
    rosidl_generator_traits::value_to_yaml(msg.i_x, out);
    out << "\n";
  }

  // member: p_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_y: ";
    rosidl_generator_traits::value_to_yaml(msg.p_y, out);
    out << "\n";
  }

  // member: d_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_y: ";
    rosidl_generator_traits::value_to_yaml(msg.d_y, out);
    out << "\n";
  }

  // member: i_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_y: ";
    rosidl_generator_traits::value_to_yaml(msg.i_y, out);
    out << "\n";
  }

  // member: p_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "p_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.p_yaw, out);
    out << "\n";
  }

  // member: d_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "d_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.d_yaw, out);
    out << "\n";
  }

  // member: i_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "i_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.i_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LiveGains & msg, bool use_flow_style = false)
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
  const hamr_interfaces::msg::LiveGains & msg,
  std::ostream & out, size_t indentation = 0)
{
  hamr_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hamr_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hamr_interfaces::msg::LiveGains & msg)
{
  return hamr_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hamr_interfaces::msg::LiveGains>()
{
  return "hamr_interfaces::msg::LiveGains";
}

template<>
inline const char * name<hamr_interfaces::msg::LiveGains>()
{
  return "hamr_interfaces/msg/LiveGains";
}

template<>
struct has_fixed_size<hamr_interfaces::msg::LiveGains>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hamr_interfaces::msg::LiveGains>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hamr_interfaces::msg::LiveGains>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__TRAITS_HPP_
