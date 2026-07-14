// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/state_error.h"


#ifndef HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_H_
#define HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/StateError in the package hamr_interfaces.
typedef struct hamr_interfaces__msg__StateError
{
  double err_x;
  double err_y;
  double err_yaw;
} hamr_interfaces__msg__StateError;

// Struct for a sequence of hamr_interfaces__msg__StateError.
typedef struct hamr_interfaces__msg__StateError__Sequence
{
  hamr_interfaces__msg__StateError * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hamr_interfaces__msg__StateError__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_H_
