// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hamr_interfaces/msg/detail/state_error__functions.h"
#include "hamr_interfaces/msg/detail/state_error__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hamr_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void StateError_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hamr_interfaces::msg::StateError(_init);
}

void StateError_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hamr_interfaces::msg::StateError *>(message_memory);
  typed_message->~StateError();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember StateError_message_member_array[3] = {
  {
    "err_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces::msg::StateError, err_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "err_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces::msg::StateError, err_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "err_yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces::msg::StateError, err_yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers StateError_message_members = {
  "hamr_interfaces::msg",  // message namespace
  "StateError",  // message name
  3,  // number of fields
  sizeof(hamr_interfaces::msg::StateError),
  false,  // has_any_key_member_
  StateError_message_member_array,  // message members
  StateError_init_function,  // function to initialize message memory (memory has to be allocated)
  StateError_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t StateError_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &StateError_message_members,
  get_message_typesupport_handle_function,
  &hamr_interfaces__msg__StateError__get_type_hash,
  &hamr_interfaces__msg__StateError__get_type_description,
  &hamr_interfaces__msg__StateError__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hamr_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hamr_interfaces::msg::StateError>()
{
  return &::hamr_interfaces::msg::rosidl_typesupport_introspection_cpp::StateError_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hamr_interfaces, msg, StateError)() {
  return &::hamr_interfaces::msg::rosidl_typesupport_introspection_cpp::StateError_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
