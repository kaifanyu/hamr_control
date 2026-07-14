// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice
#include "hamr_interfaces/msg/detail/state_error__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hamr_interfaces__msg__StateError__init(hamr_interfaces__msg__StateError * msg)
{
  if (!msg) {
    return false;
  }
  // err_x
  // err_y
  // err_yaw
  return true;
}

void
hamr_interfaces__msg__StateError__fini(hamr_interfaces__msg__StateError * msg)
{
  if (!msg) {
    return;
  }
  // err_x
  // err_y
  // err_yaw
}

bool
hamr_interfaces__msg__StateError__are_equal(const hamr_interfaces__msg__StateError * lhs, const hamr_interfaces__msg__StateError * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // err_x
  if (lhs->err_x != rhs->err_x) {
    return false;
  }
  // err_y
  if (lhs->err_y != rhs->err_y) {
    return false;
  }
  // err_yaw
  if (lhs->err_yaw != rhs->err_yaw) {
    return false;
  }
  return true;
}

bool
hamr_interfaces__msg__StateError__copy(
  const hamr_interfaces__msg__StateError * input,
  hamr_interfaces__msg__StateError * output)
{
  if (!input || !output) {
    return false;
  }
  // err_x
  output->err_x = input->err_x;
  // err_y
  output->err_y = input->err_y;
  // err_yaw
  output->err_yaw = input->err_yaw;
  return true;
}

hamr_interfaces__msg__StateError *
hamr_interfaces__msg__StateError__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__StateError * msg = (hamr_interfaces__msg__StateError *)allocator.allocate(sizeof(hamr_interfaces__msg__StateError), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hamr_interfaces__msg__StateError));
  bool success = hamr_interfaces__msg__StateError__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hamr_interfaces__msg__StateError__destroy(hamr_interfaces__msg__StateError * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hamr_interfaces__msg__StateError__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hamr_interfaces__msg__StateError__Sequence__init(hamr_interfaces__msg__StateError__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__StateError * data = NULL;

  if (size) {
    data = (hamr_interfaces__msg__StateError *)allocator.zero_allocate(size, sizeof(hamr_interfaces__msg__StateError), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hamr_interfaces__msg__StateError__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hamr_interfaces__msg__StateError__fini(&data[i - 1]);
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
hamr_interfaces__msg__StateError__Sequence__fini(hamr_interfaces__msg__StateError__Sequence * array)
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
      hamr_interfaces__msg__StateError__fini(&array->data[i]);
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

hamr_interfaces__msg__StateError__Sequence *
hamr_interfaces__msg__StateError__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hamr_interfaces__msg__StateError__Sequence * array = (hamr_interfaces__msg__StateError__Sequence *)allocator.allocate(sizeof(hamr_interfaces__msg__StateError__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hamr_interfaces__msg__StateError__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hamr_interfaces__msg__StateError__Sequence__destroy(hamr_interfaces__msg__StateError__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hamr_interfaces__msg__StateError__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hamr_interfaces__msg__StateError__Sequence__are_equal(const hamr_interfaces__msg__StateError__Sequence * lhs, const hamr_interfaces__msg__StateError__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hamr_interfaces__msg__StateError__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hamr_interfaces__msg__StateError__Sequence__copy(
  const hamr_interfaces__msg__StateError__Sequence * input,
  hamr_interfaces__msg__StateError__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hamr_interfaces__msg__StateError);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hamr_interfaces__msg__StateError * data =
      (hamr_interfaces__msg__StateError *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hamr_interfaces__msg__StateError__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hamr_interfaces__msg__StateError__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hamr_interfaces__msg__StateError__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
