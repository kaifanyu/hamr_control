// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice
#include "hamr_interfaces/msg/detail/live_gains__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hamr_interfaces__msg__LiveGains__init(hamr_interfaces__msg__LiveGains * msg)
{
  if (!msg) {
    return false;
  }
  // p_x
  // d_x
  // i_x
  // p_y
  // d_y
  // i_y
  // p_yaw
  // d_yaw
  // i_yaw
  return true;
}

void
hamr_interfaces__msg__LiveGains__fini(hamr_interfaces__msg__LiveGains * msg)
{
  if (!msg) {
    return;
  }
  // p_x
  // d_x
  // i_x
  // p_y
  // d_y
  // i_y
  // p_yaw
  // d_yaw
  // i_yaw
}

bool
hamr_interfaces__msg__LiveGains__are_equal(const hamr_interfaces__msg__LiveGains * lhs, const hamr_interfaces__msg__LiveGains * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // p_x
  if (lhs->p_x != rhs->p_x) {
    return false;
  }
  // d_x
  if (lhs->d_x != rhs->d_x) {
    return false;
  }
  // i_x
  if (lhs->i_x != rhs->i_x) {
    return false;
  }
  // p_y
  if (lhs->p_y != rhs->p_y) {
    return false;
  }
  // d_y
  if (lhs->d_y != rhs->d_y) {
    return false;
  }
  // i_y
  if (lhs->i_y != rhs->i_y) {
    return false;
  }
  // p_yaw
  if (lhs->p_yaw != rhs->p_yaw) {
    return false;
  }
  // d_yaw
  if (lhs->d_yaw != rhs->d_yaw) {
    return false;
  }
  // i_yaw
  if (lhs->i_yaw != rhs->i_yaw) {
    return false;
  }
  return true;
}

bool
hamr_interfaces__msg__LiveGains__copy(
  const hamr_interfaces__msg__LiveGains * input,
  hamr_interfaces__msg__LiveGains * output)
{
  if (!input || !output) {
    return false;
  }
  // p_x
  output->p_x = input->p_x;
  // d_x
  output->d_x = input->d_x;
  // i_x
  output->i_x = input->i_x;
  // p_y
  output->p_y = input->p_y;
  // d_y
  output->d_y = input->d_y;
  // i_y
  output->i_y = input->i_y;
  // p_yaw
  output->p_yaw = input->p_yaw;
  // d_yaw
  output->d_yaw = input->d_yaw;
  // i_yaw
  output->i_yaw = input->i_yaw;
  return true;
}

hamr_interfaces__msg__LiveGains *
hamr_interfaces__msg__LiveGains__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__LiveGains * msg = (hamr_interfaces__msg__LiveGains *)allocator.allocate(sizeof(hamr_interfaces__msg__LiveGains), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hamr_interfaces__msg__LiveGains));
  bool success = hamr_interfaces__msg__LiveGains__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hamr_interfaces__msg__LiveGains__destroy(hamr_interfaces__msg__LiveGains * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hamr_interfaces__msg__LiveGains__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hamr_interfaces__msg__LiveGains__Sequence__init(hamr_interfaces__msg__LiveGains__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__LiveGains * data = NULL;

  if (size) {
    data = (hamr_interfaces__msg__LiveGains *)allocator.zero_allocate(size, sizeof(hamr_interfaces__msg__LiveGains), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hamr_interfaces__msg__LiveGains__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hamr_interfaces__msg__LiveGains__fini(&data[i - 1]);
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
hamr_interfaces__msg__LiveGains__Sequence__fini(hamr_interfaces__msg__LiveGains__Sequence * array)
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
      hamr_interfaces__msg__LiveGains__fini(&array->data[i]);
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

hamr_interfaces__msg__LiveGains__Sequence *
hamr_interfaces__msg__LiveGains__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__LiveGains__Sequence * array = (hamr_interfaces__msg__LiveGains__Sequence *)allocator.allocate(sizeof(hamr_interfaces__msg__LiveGains__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hamr_interfaces__msg__LiveGains__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hamr_interfaces__msg__LiveGains__Sequence__destroy(hamr_interfaces__msg__LiveGains__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hamr_interfaces__msg__LiveGains__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hamr_interfaces__msg__LiveGains__Sequence__are_equal(const hamr_interfaces__msg__LiveGains__Sequence * lhs, const hamr_interfaces__msg__LiveGains__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hamr_interfaces__msg__LiveGains__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hamr_interfaces__msg__LiveGains__Sequence__copy(
  const hamr_interfaces__msg__LiveGains__Sequence * input,
  hamr_interfaces__msg__LiveGains__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hamr_interfaces__msg__LiveGains);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hamr_interfaces__msg__LiveGains * data =
      (hamr_interfaces__msg__LiveGains *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hamr_interfaces__msg__LiveGains__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hamr_interfaces__msg__LiveGains__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hamr_interfaces__msg__LiveGains__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
