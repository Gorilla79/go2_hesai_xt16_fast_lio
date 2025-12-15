// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from go2_interfaces:srv/Mode.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__MODE__TRAITS_HPP_
#define GO2_INTERFACES__SRV__DETAIL__MODE__TRAITS_HPP_

#include "go2_interfaces/srv/detail/mode__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::Mode_Request>()
{
  return "go2_interfaces::srv::Mode_Request";
}

template<>
inline const char * name<go2_interfaces::srv::Mode_Request>()
{
  return "go2_interfaces/srv/Mode_Request";
}

template<>
struct has_fixed_size<go2_interfaces::srv::Mode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_interfaces::srv::Mode_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_interfaces::srv::Mode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::Mode_Response>()
{
  return "go2_interfaces::srv::Mode_Response";
}

template<>
inline const char * name<go2_interfaces::srv::Mode_Response>()
{
  return "go2_interfaces/srv/Mode_Response";
}

template<>
struct has_fixed_size<go2_interfaces::srv::Mode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<go2_interfaces::srv::Mode_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<go2_interfaces::srv::Mode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<go2_interfaces::srv::Mode>()
{
  return "go2_interfaces::srv::Mode";
}

template<>
inline const char * name<go2_interfaces::srv::Mode>()
{
  return "go2_interfaces/srv/Mode";
}

template<>
struct has_fixed_size<go2_interfaces::srv::Mode>
  : std::integral_constant<
    bool,
    has_fixed_size<go2_interfaces::srv::Mode_Request>::value &&
    has_fixed_size<go2_interfaces::srv::Mode_Response>::value
  >
{
};

template<>
struct has_bounded_size<go2_interfaces::srv::Mode>
  : std::integral_constant<
    bool,
    has_bounded_size<go2_interfaces::srv::Mode_Request>::value &&
    has_bounded_size<go2_interfaces::srv::Mode_Response>::value
  >
{
};

template<>
struct is_service<go2_interfaces::srv::Mode>
  : std::true_type
{
};

template<>
struct is_service_request<go2_interfaces::srv::Mode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<go2_interfaces::srv::Mode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GO2_INTERFACES__SRV__DETAIL__MODE__TRAITS_HPP_
