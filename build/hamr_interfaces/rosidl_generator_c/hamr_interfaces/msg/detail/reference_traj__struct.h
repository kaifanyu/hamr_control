// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/reference_traj.h"


#ifndef HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__STRUCT_H_
#define HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/ReferenceTraj in the package hamr_interfaces.
typedef struct hamr_interfaces__msg__ReferenceTraj
{
  double x;
  double y;
  double roll;
  double pitch;
  double yaw;
  double x_dot;
  double y_dot;
  double roll_dot;
  double pitch_dot;
  double yaw_dot;
} hamr_interfaces__msg__ReferenceTraj;

// Struct for a sequence of hamr_interfaces__msg__ReferenceTraj.
typedef struct hamr_interfaces__msg__ReferenceTraj__Sequence
{
  hamr_interfaces__msg__ReferenceTraj * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hamr_interfaces__msg__ReferenceTraj__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__STRUCT_H_
