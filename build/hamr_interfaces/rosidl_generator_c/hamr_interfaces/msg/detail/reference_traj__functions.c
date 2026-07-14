// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice
#include "hamr_interfaces/msg/detail/reference_traj__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hamr_interfaces__msg__ReferenceTraj__init(hamr_interfaces__msg__ReferenceTraj * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // roll
  // pitch
  // yaw
  // x_dot
  // y_dot
  // roll_dot
  // pitch_dot
  // yaw_dot
  return true;
}

void
hamr_interfaces__msg__ReferenceTraj__fini(hamr_interfaces__msg__ReferenceTraj * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // roll
  // pitch
  // yaw
  // x_dot
  // y_dot
  // roll_dot
  // pitch_dot
  // yaw_dot
}

bool
hamr_interfaces__msg__ReferenceTraj__are_equal(const hamr_interfaces__msg__ReferenceTraj * lhs, const hamr_interfaces__msg__ReferenceTraj * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // x_dot
  if (lhs->x_dot != rhs->x_dot) {
    return false;
  }
  // y_dot
  if (lhs->y_dot != rhs->y_dot) {
    return false;
  }
  // roll_dot
  if (lhs->roll_dot != rhs->roll_dot) {
    return false;
  }
  // pitch_dot
  if (lhs->pitch_dot != rhs->pitch_dot) {
    return false;
  }
  // yaw_dot
  if (lhs->yaw_dot != rhs->yaw_dot) {
    return false;
  }
  return true;
}

bool
hamr_interfaces__msg__ReferenceTraj__copy(
  const hamr_interfaces__msg__ReferenceTraj * input,
  hamr_interfaces__msg__ReferenceTraj * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // x_dot
  output->x_dot = input->x_dot;
  // y_dot
  output->y_dot = input->y_dot;
  // roll_dot
  output->roll_dot = input->roll_dot;
  // pitch_dot
  output->pitch_dot = input->pitch_dot;
  // yaw_dot
  output->yaw_dot = input->yaw_dot;
  return true;
}

hamr_interfaces__msg__ReferenceTraj *
hamr_interfaces__msg__ReferenceTraj__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__ReferenceTraj * msg = (hamr_interfaces__msg__ReferenceTraj *)allocator.allocate(sizeof(hamr_interfaces__msg__ReferenceTraj), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hamr_interfaces__msg__ReferenceTraj));
  bool success = hamr_interfaces__msg__ReferenceTraj__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hamr_interfaces__msg__ReferenceTraj__destroy(hamr_interfaces__msg__ReferenceTraj * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hamr_interfaces__msg__ReferenceTraj__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hamr_interfaces__msg__ReferenceTraj__Sequence__init(hamr_interfaces__msg__ReferenceTraj__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__ReferenceTraj * data = NULL;

  if (size) {
    data = (hamr_interfaces__msg__ReferenceTraj *)allocator.zero_allocate(size, sizeof(hamr_interfaces__msg__ReferenceTraj), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hamr_interfaces__msg__ReferenceTraj__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hamr_interfaces__msg__ReferenceTraj__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hamr_interfaces__msg__ReferenceTraj__Sequence__fini(hamr_interfaces__msg__ReferenceTraj__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hamr_interfaces__msg__ReferenceTraj__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hamr_interfaces__msg__ReferenceTraj__Sequence *
hamr_interfaces__msg__ReferenceTraj__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__ReferenceTraj__Sequence * array = (hamr_interfaces__msg__ReferenceTraj__Sequence *)allocator.allocate(sizeof(hamr_interfaces__msg__ReferenceTraj__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hamr_interfaces__msg__ReferenceTraj__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hamr_interfaces__msg__ReferenceTraj__Sequence__destroy(hamr_interfaces__msg__ReferenceTraj__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hamr_interfaces__msg__ReferenceTraj__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hamr_interfaces__msg__ReferenceTraj__Sequence__are_equal(const hamr_interfaces__msg__ReferenceTraj__Sequence * lhs, const hamr_interfaces__msg__ReferenceTraj__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hamr_interfaces__msg__ReferenceTraj__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hamr_interfaces__msg__ReferenceTraj__Sequence__copy(
  const hamr_interfaces__msg__ReferenceTraj__Sequence * input,
  hamr_interfaces__msg__ReferenceTraj__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hamr_interfaces__msg__ReferenceTraj);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hamr_interfaces__msg__ReferenceTraj * data =
      (hamr_interfaces__msg__ReferenceTraj *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hamr_interfaces__msg__ReferenceTraj__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hamr_interfaces__msg__ReferenceTraj__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hamr_interfaces__msg__ReferenceTraj__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
