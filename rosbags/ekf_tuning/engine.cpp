// Deterministic batch adapter around the unmodified robot_localization EKF.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "robot_localization/ekf.hpp"
#include "robot_localization/filter_common.hpp"

namespace {
using robot_localization::Ekf;
using robot_localization::Measurement;
constexpr int kSize = 15;
constexpr int kTwoD[] = {2, 3, 4, 8, 9, 10, 14};

Measurement measurement(const char *name) {
  Measurement m;
  m.topic_name_ = name;
  m.measurement_ = Eigen::VectorXd::Zero(kSize);
  m.covariance_ = Eigen::MatrixXd::Zero(kSize, kSize);
  m.update_vector_.assign(kSize, false);
  for (int state : kTwoD) {
    m.covariance_(state, state) = 1e-6;
    m.update_vector_[state] = true;
  }
  return m;
}

void enable(Measurement &m, int state, double variance) {
  m.update_vector_[state] = true;
  m.covariance_(state, state) = variance;
}

double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}
}

// Events: t, sensor (0=wheel,1=IMU), vx, vy, yaw, wz, ax, ay.
// Variances: wheel vx,vy,yaw,wz; IMU yaw,wz,ax,ay.
// Masks: four wheel fields, then four IMU fields, in variance order.
// Output: all 15 states after each original input message.
extern "C" int hamr_ekf_run(const double *events, int64_t count,
  const double *q_diag, const double *p_diag, const double *r_diag,
  const int32_t *mask, int32_t relative_imu, int32_t relative_wheel,
  int32_t dynamic_q, double *output)
{
  try {
    Ekf ekf;
    Eigen::MatrixXd q = Eigen::MatrixXd::Zero(kSize, kSize);
    Eigen::MatrixXd p = Eigen::MatrixXd::Zero(kSize, kSize);
    for (int j = 0; j < kSize; ++j) {
      q(j, j) = q_diag[j]; p(j, j) = p_diag[j];
    }
    ekf.setProcessNoiseCovariance(q);
    ekf.setEstimateErrorCovariance(p);
    ekf.setUseDynamicProcessNoiseCovariance(dynamic_q != 0);
    Measurement wheel_pose = measurement("wheel_pose");
    Measurement wheel_twist = measurement("wheel_twist");
    Measurement imu_pose = measurement("imu_pose");
    Measurement imu_twist = measurement("imu_twist");
    Measurement imu_accel = measurement("imu_accel");
    if (mask[0]) enable(wheel_twist, 6, r_diag[0]);
    if (mask[1]) enable(wheel_twist, 7, r_diag[1]);
    if (mask[2]) enable(wheel_pose, 5, r_diag[2]);
    if (mask[3]) enable(wheel_twist, 11, r_diag[3]);
    if (mask[4]) enable(imu_pose, 5, r_diag[4]);
    if (mask[5]) enable(imu_twist, 11, r_diag[5]);
    if (mask[6]) enable(imu_accel, 12, r_diag[6]);
    if (mask[7]) enable(imu_accel, 13, r_diag[7]);
    double first_imu_yaw = std::numeric_limits<double>::quiet_NaN();
    double first_wheel_yaw = std::numeric_limits<double>::quiet_NaN();
    for (int64_t i = 0; i < count; ++i) {
      const double *row = events + 8 * i;
      // Offset by 1 second so zero-relative-time inputs are valid ROS stamps.
      rclcpp::Time stamp(static_cast<int64_t>(std::llround((row[0] + 1.0) * 1e9)), RCL_ROS_TIME);
      if (row[1] == 0.0) {
        if (!std::isfinite(first_wheel_yaw)) first_wheel_yaw = row[4];
        if (mask[2]) {
          wheel_pose.time_ = stamp;
          wheel_pose.measurement_[5] = wrap(row[4] - (relative_wheel ? first_wheel_yaw : 0.0));
          ekf.processMeasurement(wheel_pose);
        }
        if (mask[0] || mask[1] || mask[3]) {
          wheel_twist.time_ = stamp;
          wheel_twist.measurement_[6] = row[2];
          wheel_twist.measurement_[7] = row[3];
          wheel_twist.measurement_[11] = row[5];
          ekf.processMeasurement(wheel_twist);
        }
      } else {
        if (!std::isfinite(first_imu_yaw)) first_imu_yaw = row[4];
        // Match RosFilter::imuCallback's separate pose/twist/acceleration updates.
        if (mask[4]) {
          imu_pose.time_ = stamp;
          imu_pose.measurement_[5] = wrap(row[4] - (relative_imu ? first_imu_yaw : 0.0));
          ekf.processMeasurement(imu_pose);
        }
        if (mask[5]) {
          imu_twist.time_ = stamp;
          imu_twist.measurement_[11] = row[5];
          ekf.processMeasurement(imu_twist);
        }
        if (mask[6] || mask[7]) {
          imu_accel.time_ = stamp;
          imu_accel.measurement_[12] = row[6];
          imu_accel.measurement_[13] = row[7];
          ekf.processMeasurement(imu_accel);
        }
      }
      const auto &state = ekf.getState();
      std::copy(state.data(), state.data() + kSize, output + kSize * i);
    }
    return 0;
  } catch (...) {
    return -1;
  }
}
