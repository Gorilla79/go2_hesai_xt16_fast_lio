// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_interfaces:srv/SpeedLevel.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__SPEED_LEVEL__BUILDER_HPP_
#define GO2_INTERFACES__SRV__DETAIL__SPEED_LEVEL__BUILDER_HPP_

#include "go2_interfaces/srv/detail/speed_level__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_SpeedLevel_Request_level
{
public:
  Init_SpeedLevel_Request_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_interfaces::srv::SpeedLevel_Request level(::go2_interfaces::srv::SpeedLevel_Request::_level_type arg)
  {
    msg_.level = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::SpeedLevel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::SpeedLevel_Request>()
{
  return go2_interfaces::srv::builder::Init_SpeedLevel_Request_level();
}

}  // namespace go2_interfaces


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_SpeedLevel_Response_message
{
public:
  explicit Init_SpeedLevel_Response_message(::go2_interfaces::srv::SpeedLevel_Response & msg)
  : msg_(msg)
  {}
  ::go2_interfaces::srv::SpeedLevel_Response message(::go2_interfaces::srv::SpeedLevel_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::SpeedLevel_Response msg_;
};

class Init_SpeedLevel_Response_success
{
public:
  Init_SpeedLevel_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeedLevel_Response_message success(::go2_interfaces::srv::SpeedLevel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SpeedLevel_Response_message(msg_);
  }

private:
  ::go2_interfaces::srv::SpeedLevel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::SpeedLevel_Response>()
{
  return go2_interfaces::srv::builder::Init_SpeedLevel_Response_success();
}

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__SPEED_LEVEL__BUILDER_HPP_
