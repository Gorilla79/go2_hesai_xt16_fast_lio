// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from go2_interfaces:srv/Mode.idl
// generated code does not contain a copyright notice
#include "go2_interfaces/srv/detail/mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `mode`
#include "rosidl_runtime_c/string_functions.h"

bool
go2_interfaces__srv__Mode_Request__init(go2_interfaces__srv__Mode_Request * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    go2_interfaces__srv__Mode_Request__fini(msg);
    return false;
  }
  return true;
}

void
go2_interfaces__srv__Mode_Request__fini(go2_interfaces__srv__Mode_Request * msg)
{
  if (!msg) {
    return;
  }
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
}

bool
go2_interfaces__srv__Mode_Request__are_equal(const go2_interfaces__srv__Mode_Request * lhs, const go2_interfaces__srv__Mode_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  return true;
}

bool
go2_interfaces__srv__Mode_Request__copy(
  const go2_interfaces__srv__Mode_Request * input,
  go2_interfaces__srv__Mode_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  return true;
}

go2_interfaces__srv__Mode_Request *
go2_interfaces__srv__Mode_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Request * msg = (go2_interfaces__srv__Mode_Request *)allocator.allocate(sizeof(go2_interfaces__srv__Mode_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(go2_interfaces__srv__Mode_Request));
  bool success = go2_interfaces__srv__Mode_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
go2_interfaces__srv__Mode_Request__destroy(go2_interfaces__srv__Mode_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    go2_interfaces__srv__Mode_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
go2_interfaces__srv__Mode_Request__Sequence__init(go2_interfaces__srv__Mode_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Request * data = NULL;

  if (size) {
    data = (go2_interfaces__srv__Mode_Request *)allocator.zero_allocate(size, sizeof(go2_interfaces__srv__Mode_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = go2_interfaces__srv__Mode_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        go2_interfaces__srv__Mode_Request__fini(&data[i - 1]);
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
go2_interfaces__srv__Mode_Request__Sequence__fini(go2_interfaces__srv__Mode_Request__Sequence * array)
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
      go2_interfaces__srv__Mode_Request__fini(&array->data[i]);
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

go2_interfaces__srv__Mode_Request__Sequence *
go2_interfaces__srv__Mode_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Request__Sequence * array = (go2_interfaces__srv__Mode_Request__Sequence *)allocator.allocate(sizeof(go2_interfaces__srv__Mode_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = go2_interfaces__srv__Mode_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
go2_interfaces__srv__Mode_Request__Sequence__destroy(go2_interfaces__srv__Mode_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    go2_interfaces__srv__Mode_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
go2_interfaces__srv__Mode_Request__Sequence__are_equal(const go2_interfaces__srv__Mode_Request__Sequence * lhs, const go2_interfaces__srv__Mode_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!go2_interfaces__srv__Mode_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
go2_interfaces__srv__Mode_Request__Sequence__copy(
  const go2_interfaces__srv__Mode_Request__Sequence * input,
  go2_interfaces__srv__Mode_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(go2_interfaces__srv__Mode_Request);
    go2_interfaces__srv__Mode_Request * data =
      (go2_interfaces__srv__Mode_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!go2_interfaces__srv__Mode_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          go2_interfaces__srv__Mode_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!go2_interfaces__srv__Mode_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
go2_interfaces__srv__Mode_Response__init(go2_interfaces__srv__Mode_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    go2_interfaces__srv__Mode_Response__fini(msg);
    return false;
  }
  return true;
}

void
go2_interfaces__srv__Mode_Response__fini(go2_interfaces__srv__Mode_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
go2_interfaces__srv__Mode_Response__are_equal(const go2_interfaces__srv__Mode_Response * lhs, const go2_interfaces__srv__Mode_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
go2_interfaces__srv__Mode_Response__copy(
  const go2_interfaces__srv__Mode_Response * input,
  go2_interfaces__srv__Mode_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

go2_interfaces__srv__Mode_Response *
go2_interfaces__srv__Mode_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Response * msg = (go2_interfaces__srv__Mode_Response *)allocator.allocate(sizeof(go2_interfaces__srv__Mode_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(go2_interfaces__srv__Mode_Response));
  bool success = go2_interfaces__srv__Mode_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
go2_interfaces__srv__Mode_Response__destroy(go2_interfaces__srv__Mode_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    go2_interfaces__srv__Mode_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
go2_interfaces__srv__Mode_Response__Sequence__init(go2_interfaces__srv__Mode_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Response * data = NULL;

  if (size) {
    data = (go2_interfaces__srv__Mode_Response *)allocator.zero_allocate(size, sizeof(go2_interfaces__srv__Mode_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = go2_interfaces__srv__Mode_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        go2_interfaces__srv__Mode_Response__fini(&data[i - 1]);
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
go2_interfaces__srv__Mode_Response__Sequence__fini(go2_interfaces__srv__Mode_Response__Sequence * array)
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
      go2_interfaces__srv__Mode_Response__fini(&array->data[i]);
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

go2_interfaces__srv__Mode_Response__Sequence *
go2_interfaces__srv__Mode_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  go2_interfaces__srv__Mode_Response__Sequence * array = (go2_interfaces__srv__Mode_Response__Sequence *)allocator.allocate(sizeof(go2_interfaces__srv__Mode_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = go2_interfaces__srv__Mode_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
go2_interfaces__srv__Mode_Response__Sequence__destroy(go2_interfaces__srv__Mode_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    go2_interfaces__srv__Mode_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
go2_interfaces__srv__Mode_Response__Sequence__are_equal(const go2_interfaces__srv__Mode_Response__Sequence * lhs, const go2_interfaces__srv__Mode_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!go2_interfaces__srv__Mode_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
go2_interfaces__srv__Mode_Response__Sequence__copy(
  const go2_interfaces__srv__Mode_Response__Sequence * input,
  go2_interfaces__srv__Mode_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(go2_interfaces__srv__Mode_Response);
    go2_interfaces__srv__Mode_Response * data =
      (go2_interfaces__srv__Mode_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!go2_interfaces__srv__Mode_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          go2_interfaces__srv__Mode_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!go2_interfaces__srv__Mode_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
