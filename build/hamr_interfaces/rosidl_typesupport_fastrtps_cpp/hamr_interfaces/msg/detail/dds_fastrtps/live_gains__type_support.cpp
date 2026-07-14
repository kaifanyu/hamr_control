// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice
#include "hamr_interfaces/msg/detail/live_gains__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hamr_interfaces/msg/detail/live_gains__functions.h"
#include "hamr_interfaces/msg/detail/live_gains__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hamr_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_serialize(
  const hamr_interfaces::msg::LiveGains & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: p_x
  cdr << ros_message.p_x;

  // Member: d_x
  cdr << ros_message.d_x;

  // Member: i_x
  cdr << ros_message.i_x;

  // Member: p_y
  cdr << ros_message.p_y;

  // Member: d_y
  cdr << ros_message.d_y;

  // Member: i_y
  cdr << ros_message.i_y;

  // Member: p_yaw
  cdr << ros_message.p_yaw;

  // Member: d_yaw
  cdr << ros_message.d_yaw;

  // Member: i_yaw
  cdr << ros_message.i_yaw;

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hamr_interfaces::msg::LiveGains & ros_message)
{
  // Member: p_x
  cdr >> ros_message.p_x;

  // Member: d_x
  cdr >> ros_message.d_x;

  // Member: i_x
  cdr >> ros_message.i_x;

  // Member: p_y
  cdr >> ros_message.p_y;

  // Member: d_y
  cdr >> ros_message.d_y;

  // Member: i_y
  cdr >> ros_message.i_y;

  // Member: p_yaw
  cdr >> ros_message.p_yaw;

  // Member: d_yaw
  cdr >> ros_message.d_yaw;

  // Member: i_yaw
  cdr >> ros_message.i_yaw;

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
get_serialized_size(
  const hamr_interfaces::msg::LiveGains & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: p_x
  {
    size_t item_size = sizeof(ros_message.p_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_x
  {
    size_t item_size = sizeof(ros_message.d_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_x
  {
    size_t item_size = sizeof(ros_message.i_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: p_y
  {
    size_t item_size = sizeof(ros_message.p_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_y
  {
    size_t item_size = sizeof(ros_message.d_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_y
  {
    size_t item_size = sizeof(ros_message.i_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: p_yaw
  {
    size_t item_size = sizeof(ros_message.p_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_yaw
  {
    size_t item_size = sizeof(ros_message.d_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_yaw
  {
    size_t item_size = sizeof(ros_message.i_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
max_serialized_size_LiveGains(
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

  // Member: p_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: d_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: i_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: p_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: d_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: i_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: p_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: d_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // Member: i_yaw
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
    using DataType = hamr_interfaces::msg::LiveGains;
    is_plain =
      (
      offsetof(DataType, i_yaw) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
cdr_serialize_key(
  const hamr_interfaces::msg::LiveGains & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: p_x
  cdr << ros_message.p_x;

  // Member: d_x
  cdr << ros_message.d_x;

  // Member: i_x
  cdr << ros_message.i_x;

  // Member: p_y
  cdr << ros_message.p_y;

  // Member: d_y
  cdr << ros_message.d_y;

  // Member: i_y
  cdr << ros_message.i_y;

  // Member: p_yaw
  cdr << ros_message.p_yaw;

  // Member: d_yaw
  cdr << ros_message.d_yaw;

  // Member: i_yaw
  cdr << ros_message.i_yaw;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
get_serialized_size_key(
  const hamr_interfaces::msg::LiveGains & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: p_x
  {
    size_t item_size = sizeof(ros_message.p_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_x
  {
    size_t item_size = sizeof(ros_message.d_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_x
  {
    size_t item_size = sizeof(ros_message.i_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: p_y
  {
    size_t item_size = sizeof(ros_message.p_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_y
  {
    size_t item_size = sizeof(ros_message.d_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_y
  {
    size_t item_size = sizeof(ros_message.i_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: p_yaw
  {
    size_t item_size = sizeof(ros_message.p_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: d_yaw
  {
    size_t item_size = sizeof(ros_message.d_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Member: i_yaw
  {
    size_t item_size = sizeof(ros_message.i_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hamr_interfaces
max_serialized_size_key_LiveGains(
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

  // Member: p_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: d_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: i_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: p_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: d_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: i_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: p_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: d_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: i_yaw
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
    using DataType = hamr_interfaces::msg::LiveGains;
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
  auto typed_message =
    static_cast<const hamr_interfaces::msg::LiveGains *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LiveGains__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hamr_interfaces::msg::LiveGains *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LiveGains__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hamr_interfaces::msg::LiveGains *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LiveGains__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LiveGains(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LiveGains__callbacks = {
  "hamr_interfaces::msg",
  "LiveGains",
  _LiveGains__cdr_serialize,
  _LiveGains__cdr_deserialize,
  _LiveGains__get_serialized_size,
  _LiveGains__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _LiveGains__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LiveGains__callbacks,
  get_message_typesupport_handle_function,
  &hamr_interfaces__msg__LiveGains__get_type_hash,
  &hamr_interfaces__msg__LiveGains__get_type_description,
  &hamr_interfaces__msg__LiveGains__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hamr_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hamr_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<hamr_interfaces::msg::LiveGains>()
{
  return &hamr_interfaces::msg::typesupport_fastrtps_cpp::_LiveGains__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hamr_interfaces, msg, LiveGains)() {
  return &hamr_interfaces::msg::typesupport_fastrtps_cpp::_LiveGains__handle;
}

#ifdef __cplusplus
}
#endif
