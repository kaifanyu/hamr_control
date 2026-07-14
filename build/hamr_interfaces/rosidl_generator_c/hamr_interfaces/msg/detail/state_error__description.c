// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

#include "hamr_interfaces/msg/detail/state_error__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_hamr_interfaces
const rosidl_type_hash_t *
hamr_interfaces__msg__StateError__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x24, 0x01, 0x3a, 0x32, 0xdb, 0x35, 0xf9, 0x63,
      0x39, 0xbf, 0xe2, 0x82, 0x04, 0x08, 0xd1, 0x2c,
      0xa5, 0x58, 0xc7, 0x82, 0xbe, 0xa0, 0x95, 0x61,
      0xd3, 0x43, 0x8c, 0x2e, 0xba, 0x82, 0x2b, 0x06,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char hamr_interfaces__msg__StateError__TYPE_NAME[] = "hamr_interfaces/msg/StateError";

// Define type names, field names, and default values
static char hamr_interfaces__msg__StateError__FIELD_NAME__err_x[] = "err_x";
static char hamr_interfaces__msg__StateError__FIELD_NAME__err_y[] = "err_y";
static char hamr_interfaces__msg__StateError__FIELD_NAME__err_yaw[] = "err_yaw";

static rosidl_runtime_c__type_description__Field hamr_interfaces__msg__StateError__FIELDS[] = {
  {
    {hamr_interfaces__msg__StateError__FIELD_NAME__err_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__StateError__FIELD_NAME__err_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {hamr_interfaces__msg__StateError__FIELD_NAME__err_yaw, 7, 7},
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
hamr_interfaces__msg__StateError__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {hamr_interfaces__msg__StateError__TYPE_NAME, 30, 30},
      {hamr_interfaces__msg__StateError__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 err_x\n"
  "float64 err_y\n"
  "float64 err_yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
hamr_interfaces__msg__StateError__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {hamr_interfaces__msg__StateError__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 43, 43},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
hamr_interfaces__msg__StateError__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *hamr_interfaces__msg__StateError__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
