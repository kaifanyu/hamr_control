// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice

#include "hamr_interfaces/msg/detail/live_gains__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hamr_interfaces
const rosidl_type_hash_t *
hamr_interfaces__msg__LiveGains__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd6, 0x24, 0x29, 0x1a, 0x3c, 0x31, 0x07, 0xa6,
      0x60, 0x3a, 0x95, 0xf9, 0x8b, 0xc8, 0xa0, 0x48,
      0x99, 0xe5, 0x42, 0x01, 0x7b, 0x49, 0xea, 0xdd,
      0x1e, 0x5c, 0x4d, 0x6a, 0x45, 0x4b, 0xcb, 0x92,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char hamr_interfaces__msg__LiveGains__TYPE_NAME[] = "hamr_interfaces/msg/LiveGains";

// Define type names, field names, and default values
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__p_x[] = "p_x";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__d_x[] = "d_x";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__i_x[] = "i_x";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__p_y[] = "p_y";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__d_y[] = "d_y";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__i_y[] = "i_y";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__p_yaw[] = "p_yaw";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__d_yaw[] = "d_yaw";
static char hamr_interfaces__msg__LiveGains__FIELD_NAME__i_yaw[] = "i_yaw";

static rosidl_runtime_c__type_description__Field hamr_interfaces__msg__LiveGains__FIELDS[] = {
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__p_x, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__d_x, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__i_x, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__p_y, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__d_y, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__i_y, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__p_yaw, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__d_yaw, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__LiveGains__FIELD_NAME__i_yaw, 5, 5},
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
hamr_interfaces__msg__LiveGains__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hamr_interfaces__msg__LiveGains__TYPE_NAME, 29, 29},
      {hamr_interfaces__msg__LiveGains__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 p_x \n"
  "float64 d_x \n"
  "float64 i_x \n"
  "float64 p_y \n"
  "float64 d_y \n"
  "float64 i_y \n"
  "float64 p_yaw \n"
  "float64 d_yaw \n"
  "float64 i_yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hamr_interfaces__msg__LiveGains__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hamr_interfaces__msg__LiveGains__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 121, 121},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hamr_interfaces__msg__LiveGains__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hamr_interfaces__msg__LiveGains__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
