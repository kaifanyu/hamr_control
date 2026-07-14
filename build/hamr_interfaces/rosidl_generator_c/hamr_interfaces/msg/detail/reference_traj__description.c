// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

#include "hamr_interfaces/msg/detail/reference_traj__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hamr_interfaces
const rosidl_type_hash_t *
hamr_interfaces__msg__ReferenceTraj__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa1, 0x04, 0xb4, 0xf4, 0xea, 0x74, 0x5a, 0xe2,
      0x19, 0x79, 0x42, 0x30, 0xbc, 0xa8, 0x46, 0xe1,
      0xf8, 0xa6, 0x99, 0xab, 0x38, 0xc6, 0xc2, 0xc7,
      0x5c, 0xb4, 0x48, 0x49, 0x31, 0x44, 0xc1, 0x31,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char hamr_interfaces__msg__ReferenceTraj__TYPE_NAME[] = "hamr_interfaces/msg/ReferenceTraj";

// Define type names, field names, and default values
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__x[] = "x";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__y[] = "y";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__roll[] = "roll";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__pitch[] = "pitch";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__yaw[] = "yaw";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__x_dot[] = "x_dot";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__y_dot[] = "y_dot";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__roll_dot[] = "roll_dot";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__pitch_dot[] = "pitch_dot";
static char hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__yaw_dot[] = "yaw_dot";

static rosidl_runtime_c__type_description__Field hamr_interfaces__msg__ReferenceTraj__FIELDS[] = {
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__x_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__y_dot, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__roll_dot, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__pitch_dot, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__ReferenceTraj__FIELD_NAME__yaw_dot, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
hamr_interfaces__msg__ReferenceTraj__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hamr_interfaces__msg__ReferenceTraj__TYPE_NAME, 33, 33},
      {hamr_interfaces__msg__ReferenceTraj__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x\n"
  "float64 y\n"
  "float64 roll\n"
  "float64 pitch\n"
  "float64 yaw\n"
  "float64 x_dot\n"
  "float64 y_dot\n"
  "float64 roll_dot\n"
  "float64 pitch_dot\n"
  "float64 yaw_dot";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hamr_interfaces__msg__ReferenceTraj__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hamr_interfaces__msg__ReferenceTraj__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 137, 137},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hamr_interfaces__msg__ReferenceTraj__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hamr_interfaces__msg__ReferenceTraj__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
