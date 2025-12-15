// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_interfaces:srv/Euler.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__EULER__BUILDER_HPP_
#define GO2_INTERFACES__SRV__DETAIL__EULER__BUILDER_HPP_

#include "go2_interfaces/srv/detail/euler__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Euler_Request_yaw
{
public:
  explicit Init_Euler_Request_yaw(::go2_interfaces::srv::Euler_Request & msg)
  : msg_(msg)
  {}
  ::go2_interfaces::srv::Euler_Request yaw(::go2_interfaces::srv::Euler_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::Euler_Request msg_;
};

class Init_Euler_Request_pitch
{
public:
  explicit Init_Euler_Request_pitch(::go2_interfaces::srv::Euler_Request & msg)
  : msg_(msg)
  {}
  Init_Euler_Request_yaw pitch(::go2_interfaces::srv::Euler_Request::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Euler_Request_yaw(msg_);
  }

private:
  ::go2_interfaces::srv::Euler_Request msg_;
};

class Init_Euler_Request_roll
{
public:
  Init_Euler_Request_roll()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Euler_Request_pitch roll(::go2_interfaces::srv::Euler_Request::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Euler_Request_pitch(msg_);
  }

private:
  ::go2_interfaces::srv::Euler_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::Euler_Request>()
{
  return go2_interfaces::srv::builder::Init_Euler_Request_roll();
}

}  // namespace go2_interfaces


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Euler_Response_message
{
public:
  explicit Init_Euler_Response_message(::go2_interfaces::srv::Euler_Response & msg)
  : msg_(msg)
  {}
  ::go2_interfaces::srv::Euler_Response message(::go2_interfaces::srv::Euler_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::Euler_Response msg_;
};

class Init_Euler_Response_success
{
public:
  Init_Euler_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Euler_Response_message success(::go2_interfaces::srv::Euler_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Euler_Response_message(msg_);
  }

private:
  ::go2_interfaces::srv::Euler_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::Euler_Response>()
{
  return go2_interfaces::srv::builder::Init_Euler_Response_success();
}

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__EULER__BUILDER_HPP_
