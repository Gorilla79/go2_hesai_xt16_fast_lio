// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from go2_interfaces:srv/ContinuousGait.idl
// generated code does not contain a copyright notice

#ifndef GO2_INTERFACES__SRV__DETAIL__CONTINUOUS_GAIT__STRUCT_HPP_
#define GO2_INTERFACES__SRV__DETAIL__CONTINUOUS_GAIT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__go2_interfaces__srv__ContinuousGait_Request __attribute__((deprecated))
#else
# define DEPRECATED__go2_interfaces__srv__ContinuousGait_Request __declspec(deprecated)
#endif

namespace go2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContinuousGait_Request_
{
  using Type = ContinuousGait_Request_<ContainerAllocator>;

  explicit ContinuousGait_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flag = false;
    }
  }

  explicit ContinuousGait_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flag = false;
    }
  }

  // field types and members
  using _flag_type =
    bool;
  _flag_type flag;

  // setters for named parameter idiom
  Type & set__flag(
    const bool & _arg)
  {
    this->flag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__go2_interfaces__srv__ContinuousGait_Request
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__go2_interfaces__srv__ContinuousGait_Request
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContinuousGait_Request_ & other) const
  {
    if (this->flag != other.flag) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContinuousGait_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContinuousGait_Request_

// alias to use template instance with default allocator
using ContinuousGait_Request =
  go2_interfaces::srv::ContinuousGait_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace go2_interfaces


#ifndef _WIN32
# define DEPRECATED__go2_interfaces__srv__ContinuousGait_Response __attribute__((deprecated))
#else
# define DEPRECATED__go2_interfaces__srv__ContinuousGait_Response __declspec(deprecated)
#endif

namespace go2_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ContinuousGait_Response_
{
  using Type = ContinuousGait_Response_<ContainerAllocator>;

  explicit ContinuousGait_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ContinuousGait_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__go2_interfaces__srv__ContinuousGait_Response
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__go2_interfaces__srv__ContinuousGait_Response
    std::shared_ptr<go2_interfaces::srv::ContinuousGait_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContinuousGait_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContinuousGait_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContinuousGait_Response_

// alias to use template instance with default allocator
using ContinuousGait_Response =
  go2_interfaces::srv::ContinuousGait_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace go2_interfaces

namespace go2_interfaces
{

namespace srv
{

struct ContinuousGait
{
  using Request = go2_interfaces::srv::ContinuousGait_Request;
  using Response = go2_interfaces::srv::ContinuousGait_Response;
};

}  // namespace srv

}  // namespace go2_interfaces

#endif  // GO2_INTERFACES__SRV__DETAIL__CONTINUOUS_GAIT__STRUCT_HPP_
