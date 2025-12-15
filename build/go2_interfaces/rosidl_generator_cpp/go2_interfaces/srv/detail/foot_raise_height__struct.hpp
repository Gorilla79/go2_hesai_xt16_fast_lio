// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from go2_interfaces:srv/FootRaiseHeight.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__FOOT_RAISE_HEIGHT__STRUCT_HPP_
#define GO2_INTERFACES__SRV__DETAIL__FOOT_RAISE_HEIGHT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Request __attribute__((deprecated))
#else
# define DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Request __declspec(deprecated)
#endif

namespace go2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FootRaiseHeight_Request_
{
  using Type = FootRaiseHeight_Request_<ContainerAllocator>;

  explicit FootRaiseHeight_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0.0f;
    }
  }

  explicit FootRaiseHeight_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->height = 0.0f;
    }
  }

  // field types and members
  using _height_type =
    float;
  _height_type height;

  // setters for named parameter idiom
  Type & set__height(
    const float & _arg)
  {
    this->height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Request
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Request
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootRaiseHeight_Request_ & other) const
  {
    if (this->height != other.height) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootRaiseHeight_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootRaiseHeight_Request_

// alias to use template instance with default allocator
using FootRaiseHeight_Request =
  go2_interfaces::srv::FootRaiseHeight_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace go2_interfaces


#ifndef _WIN32
# define DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Response __attribute__((deprecated))
#else
# define DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Response __declspec(deprecated)
#endif

namespace go2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct FootRaiseHeight_Response_
{
  using Type = FootRaiseHeight_Response_<ContainerAllocator>;

  explicit FootRaiseHeight_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit FootRaiseHeight_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Response
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__go2_interfaces__srv__FootRaiseHeight_Response
    std::shared_ptr<go2_interfaces::srv::FootRaiseHeight_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FootRaiseHeight_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const FootRaiseHeight_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FootRaiseHeight_Response_

// alias to use template instance with default allocator
using FootRaiseHeight_Response =
  go2_interfaces::srv::FootRaiseHeight_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace go2_interfaces

namespace go2_interfaces
{

namespace srv
{

struct FootRaiseHeight
{
  using Request = go2_interfaces::srv::FootRaiseHeight_Request;
  using Response = go2_interfaces::srv::FootRaiseHeight_Response;
};

}  // namespace srv

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__FOOT_RAISE_HEIGHT__STRUCT_HPP_
