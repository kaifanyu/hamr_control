// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice
#include "hamr_interfaces/msg/detail/live_gains__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "hamr_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "hamr_interfaces/msg/detail/live_gains__struct.h"
#include "hamr_interfaces/msg/detail/live_gains__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _LiveGains__ros_msg_type = hamr_interfaces__msg__LiveGains;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_serialize_hamr_interfaces__msg__LiveGains(
  const hamr_interfaces__msg__LiveGains * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: p_x
  {
    cdr << ros_message->p_x;
  }

  // Field name: d_x
  {
    cdr << ros_message->d_x;
  }

  // Field name: i_x
  {
    cdr << ros_message->i_x;
  }

  // Field name: p_y
  {
    cdr << ros_message->p_y;
  }

  // Field name: d_y
  {
    cdr << ros_message->d_y;
  }

  // Field name: i_y
  {
    cdr << ros_message->i_y;
  }

  // Field name: p_yaw
  {
    cdr << ros_message->p_yaw;
  }

  // Field name: d_yaw
  {
    cdr << ros_message->d_yaw;
  }

  // Field name: i_yaw
  {
    cdr << ros_message->i_yaw;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_deserialize_hamr_interfaces__msg__LiveGains(
  eprosima::fastcdr::Cdr & cdr,
  hamr_interfaces__msg__LiveGains * ros_message)
{
  // Field name: p_x
  {
    cdr >> ros_message->p_x;
  }

  // Field name: d_x
  {
    cdr >> ros_message->d_x;
  }

  // Field name: i_x
  {
    cdr >> ros_message->i_x;
  }

  // Field name: p_y
  {
    cdr >> ros_message->p_y;
  }

  // Field name: d_y
  {
    cdr >> ros_message->d_y;
  }

  // Field name: i_y
  {
    cdr >> ros_message->i_y;
  }

  // Field name: p_yaw
  {
    cdr >> ros_message->p_yaw;
  }

  // Field name: d_yaw
  {
    cdr >> ros_message->d_yaw;
  }

  // Field name: i_yaw
  {
    cdr >> ros_message->i_yaw;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t get_serialized_size_hamr_interfaces__msg__LiveGains(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LiveGains__ros_msg_type * ros_message = static_cast<const _LiveGains__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: p_x
  {
    size_t item_size = sizeof(ros_message->p_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_x
  {
    size_t item_size = sizeof(ros_message->d_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_x
  {
    size_t item_size = sizeof(ros_message->i_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: p_y
  {
    size_t item_size = sizeof(ros_message->p_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_y
  {
    size_t item_size = sizeof(ros_message->d_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_y
  {
    size_t item_size = sizeof(ros_message->i_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: p_yaw
  {
    size_t item_size = sizeof(ros_message->p_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_yaw
  {
    size_t item_size = sizeof(ros_message->d_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_yaw
  {
    size_t item_size = sizeof(ros_message->i_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t max_serialized_size_hamr_interfaces__msg__LiveGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: p_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: p_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: p_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hamr_interfaces__msg__LiveGains;
    is_plain =
      (
      offsetof(DataType, i_yaw) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
bool cdr_serialize_key_hamr_interfaces__msg__LiveGains(
  const hamr_interfaces__msg__LiveGains * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: p_x
  {
    cdr << ros_message->p_x;
  }

  // Field name: d_x
  {
    cdr << ros_message->d_x;
  }

  // Field name: i_x
  {
    cdr << ros_message->i_x;
  }

  // Field name: p_y
  {
    cdr << ros_message->p_y;
  }

  // Field name: d_y
  {
    cdr << ros_message->d_y;
  }

  // Field name: i_y
  {
    cdr << ros_message->i_y;
  }

  // Field name: p_yaw
  {
    cdr << ros_message->p_yaw;
  }

  // Field name: d_yaw
  {
    cdr << ros_message->d_yaw;
  }

  // Field name: i_yaw
  {
    cdr << ros_message->i_yaw;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t get_serialized_size_key_hamr_interfaces__msg__LiveGains(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LiveGains__ros_msg_type * ros_message = static_cast<const _LiveGains__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: p_x
  {
    size_t item_size = sizeof(ros_message->p_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_x
  {
    size_t item_size = sizeof(ros_message->d_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_x
  {
    size_t item_size = sizeof(ros_message->i_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: p_y
  {
    size_t item_size = sizeof(ros_message->p_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_y
  {
    size_t item_size = sizeof(ros_message->d_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_y
  {
    size_t item_size = sizeof(ros_message->i_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: p_yaw
  {
    size_t item_size = sizeof(ros_message->p_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: d_yaw
  {
    size_t item_size = sizeof(ros_message->d_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: i_yaw
  {
    size_t item_size = sizeof(ros_message->i_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_hamr_interfaces
size_t max_serialized_size_key_hamr_interfaces__msg__LiveGains(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: p_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: p_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: p_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: d_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: i_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hamr_interfaces__msg__LiveGains;
    is_plain =
      (
      offsetof(DataType, i_yaw) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _LiveGains__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const hamr_interfaces__msg__LiveGains * ros_message = static_cast<const hamr_interfaces__msg__LiveGains *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_hamr_interfaces__msg__LiveGains(ros_message, cdr);
}

static bool _LiveGains__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  hamr_interfaces__msg__LiveGains * ros_message = static_cast<hamr_interfaces__msg__LiveGains *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_hamr_interfaces__msg__LiveGains(cdr, ros_message);
}

static uint32_t _LiveGains__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_hamr_interfaces__msg__LiveGains(
      untyped_ros_message, 0));
}

static size_t _LiveGains__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_hamr_interfaces__msg__LiveGains(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LiveGains = {
  "hamr_interfaces::msg",
  "LiveGains",
  _LiveGains__cdr_serialize,
  _LiveGains__cdr_deserialize,
  _LiveGains__get_serialized_size,
  _LiveGains__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LiveGains__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LiveGains,
  get_message_typesupport_handle_function,
  &hamr_interfaces__msg__LiveGains__get_type_hash,
  &hamr_interfaces__msg__LiveGains__get_type_description,
  &hamr_interfaces__msg__LiveGains__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, hamr_interfaces, msg, LiveGains)() {
  return &_LiveGains__type_support;
}

#if defined(__cplusplus)
}
#endif
