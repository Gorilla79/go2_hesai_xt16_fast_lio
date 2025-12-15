// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from go2_interfaces:srv/SwitchJoystick.idl
// generated code does not contain a copyright notice
#include "go2_interfaces/srv/detail/switch_joystick__rosidl_typesupport_fastrtps_cpp.hpp"
#include "go2_interfaces/srv/detail/switch_joystick__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace go2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
cdr_serialize(
  const go2_interfaces::srv::SwitchJoystick_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: flag
  cdr << (ros_message.flag ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  go2_interfaces::srv::SwitchJoystick_Request & ros_message)
{
  // Member: flag
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.flag = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
get_serialized_size(
  const go2_interfaces::srv::SwitchJoystick_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: flag
  {
    size_t item_size = sizeof(ros_message.flag);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
max_serialized_size_SwitchJoystick_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: flag
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _SwitchJoystick_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const go2_interfaces::srv::SwitchJoystick_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SwitchJoystick_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<go2_interfaces::srv::SwitchJoystick_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SwitchJoystick_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const go2_interfaces::srv::SwitchJoystick_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SwitchJoystick_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SwitchJoystick_Request(full_bounded, 0);
}

static message_type_support_callbacks_t _SwitchJoystick_Request__callbacks = {
  "go2_interfaces::srv",
  "SwitchJoystick_Request",
  _SwitchJoystick_Request__cdr_serialize,
  _SwitchJoystick_Request__cdr_deserialize,
  _SwitchJoystick_Request__get_serialized_size,
  _SwitchJoystick_Request__max_serialized_size
};

static rosidl_message_type_support_t _SwitchJoystick_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchJoystick_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace go2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_go2_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<go2_interfaces::srv::SwitchJoystick_Request>()
{
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, go2_interfaces, srv, SwitchJoystick_Request)() {
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace go2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
cdr_serialize(
  const go2_interfaces::srv::SwitchJoystick_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: success
  cdr << (ros_message.success ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  go2_interfaces::srv::SwitchJoystick_Response & ros_message)
{
  // Member: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.success = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
get_serialized_size(
  const go2_interfaces::srv::SwitchJoystick_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: success
  {
    size_t item_size = sizeof(ros_message.success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_go2_interfaces
max_serialized_size_SwitchJoystick_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: success
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _SwitchJoystick_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const go2_interfaces::srv::SwitchJoystick_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SwitchJoystick_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<go2_interfaces::srv::SwitchJoystick_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SwitchJoystick_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const go2_interfaces::srv::SwitchJoystick_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SwitchJoystick_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SwitchJoystick_Response(full_bounded, 0);
}

static message_type_support_callbacks_t _SwitchJoystick_Response__callbacks = {
  "go2_interfaces::srv",
  "SwitchJoystick_Response",
  _SwitchJoystick_Response__cdr_serialize,
  _SwitchJoystick_Response__cdr_deserialize,
  _SwitchJoystick_Response__get_serialized_size,
  _SwitchJoystick_Response__max_serialized_size
};

static rosidl_message_type_support_t _SwitchJoystick_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchJoystick_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace go2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_go2_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<go2_interfaces::srv::SwitchJoystick_Response>()
{
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, go2_interfaces, srv, SwitchJoystick_Response)() {
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace go2_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _SwitchJoystick__callbacks = {
  "go2_interfaces::srv",
  "SwitchJoystick",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, go2_interfaces, srv, SwitchJoystick_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, go2_interfaces, srv, SwitchJoystick_Response)(),
};

static rosidl_service_type_support_t _SwitchJoystick__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SwitchJoystick__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace go2_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_go2_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<go2_interfaces::srv::SwitchJoystick>()
{
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, go2_interfaces, srv, SwitchJoystick)() {
  return &go2_interfaces::srv::typesupport_fastrtps_cpp::_SwitchJoystick__handle;
}

#ifdef __cplusplus
}
#endif
