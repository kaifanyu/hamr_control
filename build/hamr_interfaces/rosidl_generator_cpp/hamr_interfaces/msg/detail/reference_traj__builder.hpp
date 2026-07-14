// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hamr_interfaces:msg/ReferenceTraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/reference_traj.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__BUILDER_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hamr_interfaces/msg/detail/reference_traj__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hamr_interfaces
{

namespace msg
{

namespace builder
{

class Init_ReferenceTraj_yaw_dot
{
public:
  explicit Init_ReferenceTraj_yaw_dot(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  ::hamr_interfaces::msg::ReferenceTraj yaw_dot(::hamr_interfaces::msg::ReferenceTraj::_yaw_dot_type arg)
  {
    msg_.yaw_dot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_pitch_dot
{
public:
  explicit Init_ReferenceTraj_pitch_dot(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_yaw_dot pitch_dot(::hamr_interfaces::msg::ReferenceTraj::_pitch_dot_type arg)
  {
    msg_.pitch_dot = std::move(arg);
    return Init_ReferenceTraj_yaw_dot(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_roll_dot
{
public:
  explicit Init_ReferenceTraj_roll_dot(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_pitch_dot roll_dot(::hamr_interfaces::msg::ReferenceTraj::_roll_dot_type arg)
  {
    msg_.roll_dot = std::move(arg);
    return Init_ReferenceTraj_pitch_dot(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_y_dot
{
public:
  explicit Init_ReferenceTraj_y_dot(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_roll_dot y_dot(::hamr_interfaces::msg::ReferenceTraj::_y_dot_type arg)
  {
    msg_.y_dot = std::move(arg);
    return Init_ReferenceTraj_roll_dot(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_x_dot
{
public:
  explicit Init_ReferenceTraj_x_dot(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_y_dot x_dot(::hamr_interfaces::msg::ReferenceTraj::_x_dot_type arg)
  {
    msg_.x_dot = std::move(arg);
    return Init_ReferenceTraj_y_dot(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_yaw
{
public:
  explicit Init_ReferenceTraj_yaw(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_x_dot yaw(::hamr_interfaces::msg::ReferenceTraj::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_ReferenceTraj_x_dot(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_pitch
{
public:
  explicit Init_ReferenceTraj_pitch(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_yaw pitch(::hamr_interfaces::msg::ReferenceTraj::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_ReferenceTraj_yaw(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_roll
{
public:
  explicit Init_ReferenceTraj_roll(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_pitch roll(::hamr_interfaces::msg::ReferenceTraj::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_ReferenceTraj_pitch(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_y
{
public:
  explicit Init_ReferenceTraj_y(::hamr_interfaces::msg::ReferenceTraj & msg)
  : msg_(msg)
  {}
  Init_ReferenceTraj_roll y(::hamr_interfaces::msg::ReferenceTraj::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_ReferenceTraj_roll(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

class Init_ReferenceTraj_x
{
public:
  Init_ReferenceTraj_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferenceTraj_y x(::hamr_interfaces::msg::ReferenceTraj::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_ReferenceTraj_y(msg_);
  }

private:
  ::hamr_interfaces::msg::ReferenceTraj msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hamr_interfaces::msg::ReferenceTraj>()
{
  return hamr_interfaces::msg::builder::Init_ReferenceTraj_x();
}

}  // namespace hamr_interfaces

#endif  // HAMR_INTERFACES__MSG__DETAIL__REFERENCE_TRAJ__BUILDER_HPP_
