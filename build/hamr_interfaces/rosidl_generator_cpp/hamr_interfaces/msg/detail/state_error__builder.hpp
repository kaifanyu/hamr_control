// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hamr_interfaces:msg/StateError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/state_error.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__BUILDER_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hamr_interfaces/msg/detail/state_error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hamr_interfaces
{

namespace msg
{

namespace builder
{

class Init_StateError_err_yaw
{
public:
  explicit Init_StateError_err_yaw(::hamr_interfaces::msg::StateError & msg)
  : msg_(msg)
  {}
  ::hamr_interfaces::msg::StateError err_yaw(::hamr_interfaces::msg::StateError::_err_yaw_type arg)
  {
    msg_.err_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hamr_interfaces::msg::StateError msg_;
};

class Init_StateError_err_y
{
public:
  explicit Init_StateError_err_y(::hamr_interfaces::msg::StateError & msg)
  : msg_(msg)
  {}
  Init_StateError_err_yaw err_y(::hamr_interfaces::msg::StateError::_err_y_type arg)
  {
    msg_.err_y = std::move(arg);
    return Init_StateError_err_yaw(msg_);
  }

private:
  ::hamr_interfaces::msg::StateError msg_;
};

class Init_StateError_err_x
{
public:
  Init_StateError_err_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StateError_err_y err_x(::hamr_interfaces::msg::StateError::_err_x_type arg)
  {
    msg_.err_x = std::move(arg);
    return Init_StateError_err_y(msg_);
  }

private:
  ::hamr_interfaces::msg::StateError msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hamr_interfaces::msg::StateError>()
{
  return hamr_interfaces::msg::builder::Init_StateError_err_x();
}

}  // namespace hamr_interfaces

#endif  // HAMR_INTERFACES__MSG__DETAIL__STATE_ERROR__BUILDER_HPP_
