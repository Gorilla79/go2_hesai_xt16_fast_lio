// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from go2_interfaces:srv/BodyHeight.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__TRAITS_HPP_
#define GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__TRAITS_HPP_

#include "go2_interfaces/srv/detail/body_height__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::BodyHeight_Request>()
{
  return "go2_interfaces::srv::BodyHeight_Request";
}

template<>
inline const char * name<go2_interfaces::srv::BodyHeight_Request>()
{
  return "go2_interfaces/srv/BodyHeight_Request";
}

template<>
struct has_fixed_size<go2_interfaces::srv::BodyHeight_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<go2_interfaces::srv::BodyHeight_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<go2_interfaces::srv::BodyHeight_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::BodyHeight_Response>()
{
  return "go2_interfaces::srv::BodyHeight_Response";
}

template<>
inline const char * name<go2_interfaces::srv::BodyHeight_Response>()
{
  return "go2_interfaces/srv/BodyHeight_Response";
}

template<>
struct has_fixed_size<go2_interfaces::srv::BodyHeight_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_interfaces::srv::BodyHeight_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_interfaces::srv::BodyHeight_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::BodyHeight>()
{
  return "go2_interfaces::srv::BodyHeight";
}

template<>
inline const char * name<go2_interfaces::srv::BodyHeight>()
{
  return "go2_interfaces/srv/BodyHeight";
}

template<>
struct has_fixed_size<go2_interfaces::srv::BodyHeight>
  : std::integral_constant<
    bool,
    has_fixed_size<go2_interfaces::srv::BodyHeight_Request>::value &&
    has_fixed_size<go2_interfaces::srv::BodyHeight_Response>::value
  >
{
};

template<>
struct has_bounded_size<go2_interfaces::srv::BodyHeight>
  : std::integral_constant<
    bool,
    has_bounded_size<go2_interfaces::srv::BodyHeight_Request>::value &&
    has_bounded_size<go2_interfaces::srv::BodyHeight_Response>::value
  >
{
};

template<>
struct is_service<go2_interfaces::srv::BodyHeight>
  : std::true_type
{
};

template<>
struct is_service_request<go2_interfaces::srv::BodyHeight_Request>
  : std::true_type
{
};

template<>
struct is_service_response<go2_interfaces::srv::BodyHeight_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GO2_INTERFACES__SRV__DETAIL__BODY_HEIGHT__TRAITS_HPP_
