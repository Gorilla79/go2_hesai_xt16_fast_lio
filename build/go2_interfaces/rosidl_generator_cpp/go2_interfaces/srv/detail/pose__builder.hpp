// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from go2_interfaces:srv/Pose.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__POSE__BUILDER_HPP_
#define GO2_INTERFACES__SRV__DETAIL__POSE__BUILDER_HPP_

#include "go2_interfaces/srv/detail/pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pose_Request_flag
{
public:
  Init_Pose_Request_flag()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_interfaces::srv::Pose_Request flag(::go2_interfaces::srv::Pose_Request::_flag_type arg)
  {
    msg_.flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::Pose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::Pose_Request>()
{
  return go2_interfaces::srv::builder::Init_Pose_Request_flag();
}

}  // namespace go2_interfaces


namespace go2_interfaces
{

namespace srv
{

namespace builder
{

class Init_Pose_Response_success
{
public:
  Init_Pose_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::go2_interfaces::srv::Pose_Response success(::go2_interfaces::srv::Pose_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::go2_interfaces::srv::Pose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::go2_interfaces::srv::Pose_Response>()
{
  return go2_interfaces::srv::builder::Init_Pose_Response_success();
}

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__POSE__BUILDER_HPP_
