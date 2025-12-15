// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_interfaces:srv/BodyHeight.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__BUILDER_HPP_
#define GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__BUILDER_HPP_

#include "go2_interfaces/srv/detail/body_height__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_BodyHeight_Request_height
{
public:
  Init_BodyHeight_Request_height()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_interfaces::srv::BodyHeight_Request height(::go2_interfaces::srv::BodyHeight_Request::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::BodyHeight_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::BodyHeight_Request>()
{
  return go2_interfaces::srv::builder::Init_BodyHeight_Request_height();
}

}  // namespace go2_interfaces


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_BodyHeight_Response_message
{
public:
  explicit Init_BodyHeight_Response_message(::go2_interfaces::srv::BodyHeight_Response & msg)
  : msg_(msg)
  {}
  ::go2_interfaces::srv::BodyHeight_Response message(::go2_interfaces::srv::BodyHeight_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::BodyHeight_Response msg_;
};

class Init_BodyHeight_Response_success
{
public:
  Init_BodyHeight_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BodyHeight_Response_message success(::go2_interfaces::srv::BodyHeight_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_BodyHeight_Response_message(msg_);
  }

private:
  ::go2_interfaces::srv::BodyHeight_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::BodyHeight_Response>()
{
  return go2_interfaces::srv::builder::Init_BodyHeight_Response_success();
}

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__BUILDER_HPP_
