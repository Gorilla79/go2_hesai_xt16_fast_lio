// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from go2_interfaces:srv/SwitchGait.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "go2_interfaces/srv/detail/switch_gait__rosidl_typesupport_introspection_c.h"
#include "go2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "go2_interfaces/srv/detail/switch_gait__functions.h"
#include "go2_interfaces/srv/detail/switch_gait__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  go2_interfaces__srv__SwitchGait_Request__init(message_memory);
}

void SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_fini_function(void * message_memory)
{
  go2_interfaces__srv__SwitchGait_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_member_array[1] = {
  {
    "d",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(go2_interfaces__srv__SwitchGait_Request, d),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_members = {
  "go2_interfaces__srv",  // message namespace
  "SwitchGait_Request",  // message name
  1,  // number of fields
  sizeof(go2_interfaces__srv__SwitchGait_Request),
  SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_member_array,  // message members
  SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_type_support_handle = {
  0,
  &SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_go2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Request)() {
  if (!SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_type_support_handle.typesupport_identifier) {
    SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SwitchGait_Request__rosidl_typesupport_introspection_c__SwitchGait_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "go2_interfaces/srv/detail/switch_gait__rosidl_typesupport_introspection_c.h"
// already included above
// #include "go2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "go2_interfaces/srv/detail/switch_gait__functions.h"
// already included above
// #include "go2_interfaces/srv/detail/switch_gait__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  go2_interfaces__srv__SwitchGait_Response__init(message_memory);
}

void SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_fini_function(void * message_memory)
{
  go2_interfaces__srv__SwitchGait_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_member_array[2] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(go2_interfaces__srv__SwitchGait_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(go2_interfaces__srv__SwitchGait_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_members = {
  "go2_interfaces__srv",  // message namespace
  "SwitchGait_Response",  // message name
  2,  // number of fields
  sizeof(go2_interfaces__srv__SwitchGait_Response),
  SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_member_array,  // message members
  SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_type_support_handle = {
  0,
  &SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_go2_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Response)() {
  if (!SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_type_support_handle.typesupport_identifier) {
    SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SwitchGait_Response__rosidl_typesupport_introspection_c__SwitchGait_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "go2_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "go2_interfaces/srv/detail/switch_gait__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_members = {
  "go2_interfaces__srv",  // service namespace
  "SwitchGait",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_Request_message_type_support_handle,
  NULL  // response message
  // go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_Response_message_type_support_handle
};

static rosidl_service_type_support_t go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_type_support_handle = {
  0,
  &go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_go2_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait)() {
  if (!go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_type_support_handle.typesupport_identifier) {
    go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, go2_interfaces, srv, SwitchGait_Response)()->data;
  }

  return &go2_interfaces__srv__detail__switch_gait__rosidl_typesupport_introspection_c__SwitchGait_service_type_support_handle;
}
