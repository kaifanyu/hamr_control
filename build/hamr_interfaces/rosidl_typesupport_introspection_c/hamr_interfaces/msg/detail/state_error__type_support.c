// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hamr_interfaces/msg/detail/state_error__rosidl_typesupport_introspection_c.h"
#include "hamr_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hamr_interfaces/msg/detail/state_error__functions.h"
#include "hamr_interfaces/msg/detail/state_error__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hamr_interfaces__msg__StateError__init(message_memory);
}

void hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_fini_function(void * message_memory)
{
  hamr_interfaces__msg__StateError__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_member_array[3] = {
  {
    "err_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__StateError, err_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "err_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__StateError, err_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "err_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__StateError, err_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_members = {
  "hamr_interfaces__msg",  // message namespace
  "StateError",  // message name
  3,  // number of fields
  sizeof(hamr_interfaces__msg__StateError),
  false,  // has_any_key_member_
  hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_member_array,  // message members
  hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_init_function,  // function to initialize message memory (memory has to be allocated)
  hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_type_support_handle = {
  0,
  &hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_members,
  get_message_typesupport_handle_function,
  &hamr_interfaces__msg__StateError__get_type_hash,
  &hamr_interfaces__msg__StateError__get_type_description,
  &hamr_interfaces__msg__StateError__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hamr_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hamr_interfaces, msg, StateError)() {
  if (!hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_type_support_handle.typesupport_identifier) {
    hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hamr_interfaces__msg__StateError__rosidl_typesupport_introspection_c__StateError_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
