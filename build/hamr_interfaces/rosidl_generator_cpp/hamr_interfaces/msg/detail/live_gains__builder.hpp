// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hamr_interfaces:msg/LiveGains.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "hamr_interfaces/msg/live_gains.hpp"


#ifndef HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__BUILDER_HPP_
#define HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hamr_interfaces/msg/detail/live_gains__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hamr_interfaces
{

namespace msg
{

namespace builder
{

class Init_LiveGains_i_yaw
{
public:
  explicit Init_LiveGains_i_yaw(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  ::hamr_interfaces::msg::LiveGains i_yaw(::hamr_interfaces::msg::LiveGains::_i_yaw_type arg)
  {
    msg_.i_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_d_yaw
{
public:
  explicit Init_LiveGains_d_yaw(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_i_yaw d_yaw(::hamr_interfaces::msg::LiveGains::_d_yaw_type arg)
  {
    msg_.d_yaw = std::move(arg);
    return Init_LiveGains_i_yaw(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_p_yaw
{
public:
  explicit Init_LiveGains_p_yaw(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_d_yaw p_yaw(::hamr_interfaces::msg::LiveGains::_p_yaw_type arg)
  {
    msg_.p_yaw = std::move(arg);
    return Init_LiveGains_d_yaw(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_i_y
{
public:
  explicit Init_LiveGains_i_y(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_p_yaw i_y(::hamr_interfaces::msg::LiveGains::_i_y_type arg)
  {
    msg_.i_y = std::move(arg);
    return Init_LiveGains_p_yaw(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_d_y
{
public:
  explicit Init_LiveGains_d_y(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_i_y d_y(::hamr_interfaces::msg::LiveGains::_d_y_type arg)
  {
    msg_.d_y = std::move(arg);
    return Init_LiveGains_i_y(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_p_y
{
public:
  explicit Init_LiveGains_p_y(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_d_y p_y(::hamr_interfaces::msg::LiveGains::_p_y_type arg)
  {
    msg_.p_y = std::move(arg);
    return Init_LiveGains_d_y(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_i_x
{
public:
  explicit Init_LiveGains_i_x(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_p_y i_x(::hamr_interfaces::msg::LiveGains::_i_x_type arg)
  {
    msg_.i_x = std::move(arg);
    return Init_LiveGains_p_y(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_d_x
{
public:
  explicit Init_LiveGains_d_x(::hamr_interfaces::msg::LiveGains & msg)
  : msg_(msg)
  {}
  Init_LiveGains_i_x d_x(::hamr_interfaces::msg::LiveGains::_d_x_type arg)
  {
    msg_.d_x = std::move(arg);
    return Init_LiveGains_i_x(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

class Init_LiveGains_p_x
{
public:
  Init_LiveGains_p_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LiveGains_d_x p_x(::hamr_interfaces::msg::LiveGains::_p_x_type arg)
  {
    msg_.p_x = std::move(arg);
    return Init_LiveGains_d_x(msg_);
  }

private:
  ::hamr_interfaces::msg::LiveGains msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hamr_interfaces::msg::LiveGains>()
{
  return hamr_interfaces::msg::builder::Init_LiveGains_p_x();
}

}  // namespace hamr_interfaces

#endif  // HAMR_INTERFACES__MSG__DETAIL__LIVE_GAINS__BUILDER_HPP_
