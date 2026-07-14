// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice
#ifndef HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hamr_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hamr_interfaces/msg/detail/live_gains__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_serialize_hamr_interfaces__msg__LiveGains(
  const hamr_interfaces__msg__LiveGains * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_deserialize_hamr_interfaces__msg__LiveGains(
  eprosima::fastcdr::Cdr &,
  hamr_interfaces__msg__LiveGains * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t get_serialized_size_hamr_interfaces__msg__LiveGains(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t max_serialized_size_hamr_interfaces__msg__LiveGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_serialize_key_hamr_interfaces__msg__LiveGains(
  const hamr_interfaces__msg__LiveGains * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t get_serialized_size_key_hamr_interfaces__msg__LiveGains(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t max_serialized_size_key_hamr_interfaces__msg__LiveGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hamr_interfaces, msg, LiveGains)();

#ifdef __cplusplus
}
#endif

#endif  // HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
