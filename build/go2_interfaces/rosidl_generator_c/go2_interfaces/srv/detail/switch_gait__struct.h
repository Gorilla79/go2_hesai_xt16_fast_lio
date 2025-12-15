// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from go2_interfaces:srv/SwitchGait.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__SWITCH_GAIT__STRUCT_H_
#define GO2_INTERFACES__SRV__DETAIL__SWITCH_GAIT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'IDLE'.
enum
{
  go2_interfaces__srv__SwitchGait_Request__IDLE = 0l
};

/// Constant 'TROT'.
enum
{
  go2_interfaces__srv__SwitchGait_Request__TROT = 1l
};

/// Constant 'TROT_RUN_NING'.
enum
{
  go2_interfaces__srv__SwitchGait_Request__TROT_RUN_NING = 2l
};

/// Constant 'FORWARD_CLIMBING_MODE'.
enum
{
  go2_interfaces__srv__SwitchGait_Request__FORWARD_CLIMBING_MODE = 3l
};

/// Constant 'REVERSE_CLIMBING_MODE'.
enum
{
  go2_interfaces__srv__SwitchGait_Request__REVERSE_CLIMBING_MODE = 4l
};

// Struct defined in srv/SwitchGait in the package go2_interfaces.
typedef struct go2_interfaces__srv__SwitchGait_Request
{
  int32_t d;
} go2_interfaces__srv__SwitchGait_Request;

// Struct for a sequence of go2_interfaces__srv__SwitchGait_Request.
typedef struct go2_interfaces__srv__SwitchGait_Request__Sequence
{
  go2_interfaces__srv__SwitchGait_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_interfaces__srv__SwitchGait_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

// Struct defined in srv/SwitchGait in the package go2_interfaces.
typedef struct go2_interfaces__srv__SwitchGait_Response
{
  bool success;
  rosidl_runtime_c__String message;
} go2_interfaces__srv__SwitchGait_Response;

// Struct for a sequence of go2_interfaces__srv__SwitchGait_Response.
typedef struct go2_interfaces__srv__SwitchGait_Response__Sequence
{
  go2_interfaces__srv__SwitchGait_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} go2_interfaces__srv__SwitchGait_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GO2_INTERFACES__SRV__DETAIL__SWITCH_GAIT__STRUCT_H_
