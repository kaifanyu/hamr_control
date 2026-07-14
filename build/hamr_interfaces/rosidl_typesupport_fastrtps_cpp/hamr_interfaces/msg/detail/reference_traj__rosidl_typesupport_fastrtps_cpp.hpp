// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

#ifndef HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hamr_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "hamr_interfaces/msg/detail/reference_traj__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace hamr_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_serialize(
  const hamr_interfaces::msg::ReferenceTraj & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hamr_interfaces::msg::ReferenceTraj & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
get_serialized_size(
  const hamr_interfaces::msg::ReferenceTraj & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
max_serialized_size_ReferenceTraj(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_serialize_key(
  const hamr_interfaces::msg::ReferenceTraj & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
get_serialized_size_key(
  const hamr_interfaces::msg::ReferenceTraj & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
max_serialized_size_key_ReferenceTraj(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hamr_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hamr_interfaces, msg, ReferenceTraj)();

#ifdef __cplusplus
}
#endif

#endif  // HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
