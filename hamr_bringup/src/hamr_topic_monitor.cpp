// Passive live health. Count serialized receipts at source rate; decode only
// the latest measurement at the dashboard rate to keep Pi overhead small.
#include <chrono>
#include <memory>
#include <set>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <yaml-cpp/yaml.h>

#include "topic_health.hpp"

namespace hamr_health {
inline double monotonic_now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<typename Message>
Message decode(const rclcpp::SerializedMessage & serialized) {
  Message message;
  rclcpp::Serialization<Message>().deserialize_message(&serialized, &message);
  return message;
}

template<typename Vector>
bool finite_xyz(const Vector & v) {return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);}

std::string measurement_error(const std::string & type, const rclcpp::SerializedMessage & data) {
  bool valid = true;
  if (type == "sensor_msgs/msg/JointState") {
    auto msg = decode<sensor_msgs::msg::JointState>(data);
    if (msg.position.empty()) {return "Empty joint position";}
    valid = std::all_of(msg.position.begin(), msg.position.end(), [](double v) {return std::isfinite(v);}) &&
      std::all_of(msg.velocity.begin(), msg.velocity.end(), [](double v) {return std::isfinite(v);});
  } else if (type == "nav_msgs/msg/Odometry") {
    auto msg = decode<nav_msgs::msg::Odometry>(data);
    valid = finite_xyz(msg.pose.pose.position) && finite_xyz(msg.pose.pose.orientation) &&
      std::isfinite(msg.pose.pose.orientation.w) && finite_xyz(msg.twist.twist.linear) && finite_xyz(msg.twist.twist.angular);
  } else if (type == "sensor_msgs/msg/Imu") {
    auto msg = decode<sensor_msgs::msg::Imu>(data);
    valid = finite_xyz(msg.angular_velocity) && finite_xyz(msg.linear_acceleration) &&
      (msg.orientation_covariance[0] == -1 || (finite_xyz(msg.orientation) && std::isfinite(msg.orientation.w)));
  } else if (type == "sensor_msgs/msg/MagneticField") {
    valid = finite_xyz(decode<sensor_msgs::msg::MagneticField>(data).magnetic_field);
  } else if (type == "std_msgs/msg/Float64MultiArray") {
    auto msg = decode<std_msgs::msg::Float64MultiArray>(data);
    valid = std::all_of(msg.data.begin(), msg.data.end(), [](double v) {return std::isfinite(v);});
  }
  return valid ? "" : "Non-finite measurement";
}

class TopicMonitor : public rclcpp::Node {
public:
  TopicMonitor() : Node("hamr_topic_monitor", rclcpp::NodeOptions().start_parameter_services(false)) {
    auto config = YAML::LoadFile(declare_parameter<std::string>("config_file", ""));
    auto hz = positive(config["publish_hz"].as<double>(), "publish_hz");
    auto window = positive(config["rate_window_s"].as<double>(), "rate_window_s");
    auto timeout = positive(config["caster_diagnostic_timeout_s"].as<double>(), "caster_diagnostic_timeout_s");
    auto qos = rclcpp::QoS(10).best_effort();
    std::set<std::string> names;
    for (const auto & item : config["streams"]) {
      auto state = std::make_shared<StreamState>(item["name"].as<std::string>(), item["topic"].as<std::string>(),
        item["type"].as<std::string>(), item["timeout_s"].as<double>(), window, monotonic_now());
      if (state->name.empty() || !names.insert(state->name).second) {
        throw std::invalid_argument("streams must have unique, nonempty names");
      }
      const auto index = streams_.size();
      streams_.push_back(state);
      latest_.emplace_back();
      subscriptions_.push_back(create_generic_subscription(state->topic, state->type, qos,
        [this, state, index](std::shared_ptr<rclcpp::SerializedMessage> message) {
          state->receive(monotonic_now());
          latest_[index] = std::move(message);
        }));
    }
    if (streams_.empty()) {throw std::invalid_argument("No streams configured");}
    for (const auto * wheel : {"front_left", "front_right"}) {
      auto report = std::make_shared<CasterReport>(wheel, timeout);
      casters_.emplace(wheel, report);
      caster_subscriptions_.push_back(create_subscription<std_msgs::msg::String>(
        std::string("/caster_encoder/") + wheel + "/diagnostics", qos,
        [report](std_msgs::msg::String::ConstSharedPtr msg) {report->receive(msg->data, monotonic_now());}));
    }
    auto topic = config["diagnostics_topic"].as<std::string>();
    publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(topic, 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz), [this]() {publish();});
    RCLCPP_INFO(get_logger(), "Monitoring %zu streams -> %s at %.1f Hz", streams_.size(), topic.c_str(), hz);
  }

private:
  void publish() {
    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = now();
    const auto receipt_now = monotonic_now();
    for (size_t index = 0; index < streams_.size(); ++index) {
      auto & state = *streams_[index];
      if (latest_[index]) {
        try {state.error = measurement_error(state.type, *latest_[index]);}
        catch (const std::exception & ex) {state.error = std::string("Cannot decode measurement: ") + ex.what();}
        latest_[index].reset();
      }
      auto report = state.snapshot(receipt_now);
      for (const auto & [wheel, caster] : casters_) {
        auto prefix = "caster/" + wheel + "/";
        if (state.name.rfind(prefix, 0) == 0) {report = caster->decorate(state.name.substr(prefix.size()), report, receipt_now);}
      }
      Status status;
      status.level = report.level;
      status.name = state.name;
      status.hardware_id = "hamr";
      status.message = report.message;
      for (const auto & [key, value] : report.values) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key; entry.value = value;
        status.values.push_back(std::move(entry));
      }
      output.status.push_back(std::move(status));
    }
    publisher_->publish(output);
  }

  std::vector<std::shared_ptr<StreamState>> streams_;
  std::vector<std::shared_ptr<rclcpp::SerializedMessage>> latest_;
  std::map<std::string, std::shared_ptr<CasterReport>> casters_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> caster_subscriptions_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace hamr_health

#ifndef HAMR_TOPIC_MONITOR_NO_MAIN
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hamr_health::TopicMonitor>());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("hamr_topic_monitor"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
