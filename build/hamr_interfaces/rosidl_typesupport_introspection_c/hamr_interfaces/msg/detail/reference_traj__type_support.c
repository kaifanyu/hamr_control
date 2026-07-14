// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hamr_interfaces/msg/detail/reference_traj__rosidl_typesupport_introspection_c.h"
#include "hamr_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hamr_interfaces/msg/detail/reference_traj__functions.h"
#include "hamr_interfaces/msg/detail/reference_traj__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hamr_interfaces__msg__ReferenceTraj__init(message_memory);
}

void hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_fini_function(void * message_memory)
{
  hamr_interfaces__msg__ReferenceTraj__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_member_array[10] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, x_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, y_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "roll_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, roll_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pitch_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, pitch_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_dot",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hamr_interfaces__msg__ReferenceTraj, yaw_dot),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_members = {
  "hamr_interfaces__msg",  // message namespace
  "ReferenceTraj",  // message name
  10,  // number of fields
  sizeof(hamr_interfaces__msg__ReferenceTraj),
  false,  // has_any_key_member_
  hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_member_array,  // message members
  hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_init_function,  // function to initialize message memory (memory has to be allocated)
  hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_type_support_handle = {
  0,
  &hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_members,
  get_message_typesupport_handle_function,
  &hamr_interfaces__msg__ReferenceTraj__get_type_hash,
  &hamr_interfaces__msg__ReferenceTraj__get_type_description,
  &hamr_interfaces__msg__ReferenceTraj__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hamr_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hamr_interfaces, msg, ReferenceTraj)() {
  if (!hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_type_support_handle.typesupport_identifier) {
    hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hamr_interfaces__msg__ReferenceTraj__rosidl_typesupport_introspection_c__ReferenceTraj_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
