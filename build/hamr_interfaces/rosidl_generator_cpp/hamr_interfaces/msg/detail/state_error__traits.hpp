// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/state_error.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__TRAITS_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hamr_interfaces/msg/detail/state_error__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hamr_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StateError & msg,
  std::ostream & out)
{
  out << "{";
  // member: err_x
  {
    out << "err_x: ";
    rosidl_generator_traits::value_to_yaml(msg.err_x, out);
    out << ", ";
  }

  // member: err_y
  {
    out << "err_y: ";
    rosidl_generator_traits::value_to_yaml(msg.err_y, out);
    out << ", ";
  }

  // member: err_yaw
  {
    out << "err_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.err_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StateError & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: err_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_x: ";
    rosidl_generator_traits::value_to_yaml(msg.err_x, out);
    out << "\n";
  }

  // member: err_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_y: ";
    rosidl_generator_traits::value_to_yaml(msg.err_y, out);
    out << "\n";
  }

  // member: err_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "err_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.err_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StateError & msg, bool use_flow_style = false)
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
  const hamr_interfaces::msg::StateError & msg,
  std::ostream & out, size_t indentation = 0)
{
  hamr_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hamr_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hamr_interfaces::msg::StateError & msg)
{
  return hamr_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hamr_interfaces::msg::StateError>()
{
  return "hamr_interfaces::msg::StateError";
}

template<>
inline const char * name<hamr_interfaces::msg::StateError>()
{
  return "hamr_interfaces/msg/StateError";
}

template<>
struct has_fixed_size<hamr_interfaces::msg::StateError>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hamr_interfaces::msg::StateError>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hamr_interfaces::msg::StateError>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__TRAITS_HPP_
