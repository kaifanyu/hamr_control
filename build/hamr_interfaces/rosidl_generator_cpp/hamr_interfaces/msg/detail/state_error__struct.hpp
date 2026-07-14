// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/state_error.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hamr_interfaces__msg__StateError __attribute__((deprecated))
#else
# define DEPRECATED__hamr_interfaces__msg__StateError __declspec(deprecated)
#endif

namespace hamr_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct StateError_
{
  using Type = StateError_<ContainerAllocator>;

  explicit StateError_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->err_x = 0.0;
      this->err_y = 0.0;
      this->err_yaw = 0.0;
    }
  }

  explicit StateError_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->err_x = 0.0;
      this->err_y = 0.0;
      this->err_yaw = 0.0;
    }
  }

  // field types and members
  using _err_x_type =
    double;
  _err_x_type err_x;
  using _err_y_type =
    double;
  _err_y_type err_y;
  using _err_yaw_type =
    double;
  _err_yaw_type err_yaw;

  // setters for named parameter idiom
  Type & set__err_x(
    const double & _arg)
  {
    this->err_x = _arg;
    return *this;
  }
  Type & set__err_y(
    const double & _arg)
  {
    this->err_y = _arg;
    return *this;
  }
  Type & set__err_yaw(
    const double & _arg)
  {
    this->err_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hamr_interfaces::msg::StateError_<ContainerAllocator> *;
  using ConstRawPtr =
    const hamr_interfaces::msg::StateError_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hamr_interfaces::msg::StateError_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hamr_interfaces::msg::StateError_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hamr_interfaces__msg__StateError
    std::shared_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hamr_interfaces__msg__StateError
    std::shared_ptr<hamr_interfaces::msg::StateError_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StateError_ & other) const
  {
    if (this->err_x != other.err_x) {
      return false;
    }
    if (this->err_y != other.err_y) {
      return false;
    }
    if (this->err_yaw != other.err_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const StateError_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StateError_

// alias to use template instance with default allocator
using StateError =
  hamr_interfaces::msg::StateError_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hamr_interfaces

#endif  // HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__STRUCT_HPP_
