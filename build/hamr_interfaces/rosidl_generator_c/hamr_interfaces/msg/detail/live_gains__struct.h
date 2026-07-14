// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/live_gains.h"


#ifndef HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_H_
#define HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/LiveGains in the package hamr_interfaces.
typedef struct hamr_interfaces__msg__LiveGains
{
  double p_x;
  double d_x;
  double i_x;
  double p_y;
  double d_y;
  double i_y;
  double p_yaw;
  double d_yaw;
  double i_yaw;
} hamr_interfaces__msg__LiveGains;

// Struct for a sequence of hamr_interfaces__msg__LiveGains.
typedef struct hamr_interfaces__msg__LiveGains__Sequence
{
  hamr_interfaces__msg__LiveGains * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hamr_interfaces__msg__LiveGains__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_H_
