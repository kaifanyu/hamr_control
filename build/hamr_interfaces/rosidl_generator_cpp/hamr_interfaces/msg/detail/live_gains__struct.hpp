// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/live_gains.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hamr_interfaces__msg__LiveGains __attribute__((deprecated))
#else
# define DEPRECATED__hamr_interfaces__msg__LiveGains __declspec(deprecated)
#endif

namespace hamr_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LiveGains_
{
  using Type = LiveGains_<ContainerAllocator>;

  explicit LiveGains_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p_x = 0.0;
      this->d_x = 0.0;
      this->i_x = 0.0;
      this->p_y = 0.0;
      this->d_y = 0.0;
      this->i_y = 0.0;
      this->p_yaw = 0.0;
      this->d_yaw = 0.0;
      this->i_yaw = 0.0;
    }
  }

  explicit LiveGains_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p_x = 0.0;
      this->d_x = 0.0;
      this->i_x = 0.0;
      this->p_y = 0.0;
      this->d_y = 0.0;
      this->i_y = 0.0;
      this->p_yaw = 0.0;
      this->d_yaw = 0.0;
      this->i_yaw = 0.0;
    }
  }

  // field types and members
  using _p_x_type =
    double;
  _p_x_type p_x;
  using _d_x_type =
    double;
  _d_x_type d_x;
  using _i_x_type =
    double;
  _i_x_type i_x;
  using _p_y_type =
    double;
  _p_y_type p_y;
  using _d_y_type =
    double;
  _d_y_type d_y;
  using _i_y_type =
    double;
  _i_y_type i_y;
  using _p_yaw_type =
    double;
  _p_yaw_type p_yaw;
  using _d_yaw_type =
    double;
  _d_yaw_type d_yaw;
  using _i_yaw_type =
    double;
  _i_yaw_type i_yaw;

  // setters for named parameter idiom
  Type & set__p_x(
    const double & _arg)
  {
    this->p_x = _arg;
    return *this;
  }
  Type & set__d_x(
    const double & _arg)
  {
    this->d_x = _arg;
    return *this;
  }
  Type & set__i_x(
    const double & _arg)
  {
    this->i_x = _arg;
    return *this;
  }
  Type & set__p_y(
    const double & _arg)
  {
    this->p_y = _arg;
    return *this;
  }
  Type & set__d_y(
    const double & _arg)
  {
    this->d_y = _arg;
    return *this;
  }
  Type & set__i_y(
    const double & _arg)
  {
    this->i_y = _arg;
    return *this;
  }
  Type & set__p_yaw(
    const double & _arg)
  {
    this->p_yaw = _arg;
    return *this;
  }
  Type & set__d_yaw(
    const double & _arg)
  {
    this->d_yaw = _arg;
    return *this;
  }
  Type & set__i_yaw(
    const double & _arg)
  {
    this->i_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hamr_interfaces::msg::LiveGains_<ContainerAllocator> *;
  using ConstRawPtr =
    const hamr_interfaces::msg::LiveGains_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hamr_interfaces::msg::LiveGains_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hamr_interfaces::msg::LiveGains_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hamr_interfaces__msg__LiveGains
    std::shared_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hamr_interfaces__msg__LiveGains
    std::shared_ptr<hamr_interfaces::msg::LiveGains_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LiveGains_ & other) const
  {
    if (this->p_x != other.p_x) {
      return false;
    }
    if (this->d_x != other.d_x) {
      return false;
    }
    if (this->i_x != other.i_x) {
      return false;
    }
    if (this->p_y != other.p_y) {
      return false;
    }
    if (this->d_y != other.d_y) {
      return false;
    }
    if (this->i_y != other.i_y) {
      return false;
    }
    if (this->p_yaw != other.p_yaw) {
      return false;
    }
    if (this->d_yaw != other.d_yaw) {
      return false;
    }
    if (this->i_yaw != other.i_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const LiveGains_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LiveGains_

// alias to use template instance with default allocator
using LiveGains =
  hamr_interfaces::msg::LiveGains_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hamr_interfaces

#endif  // HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__STRUCT_HPP_
