#include <gtest/gtest.h>
#include <limits>

#define HAMR_TOPIC_MONITOR_NO_MAIN
#include "../src/hamr_topic_monitor.cpp"

using namespace hamr_health;

StreamState state() {return StreamState("caster/front_left/H1", "/example", "sensor_msgs/msg/JointState", 1.0, 5.0, 10.0);}

Json report(uint32_t flags = 1 | 2 | 4 | 16 | 64 | 1024) {
  return {
    {"wheel_id", "front_left"},
    {"nodes", {{{"role", "H1"}, {"device_id", "test"}, {"boot_id", "boot"},
      {"never_seen", false}, {"stale", false}, {"inventory_verified", true}, {"metadata_received", true}}}},
    {"streams", {{{"role", "H1"}, {"device_id", "test"}, {"boot_id", "boot"},
      {"current_boot", true}, {"last_flags", flags}}}},
  };
}

CasterReport caster(const Json & value) {
  CasterReport result("front_left", 2.5);
  result.receive(value.dump(), 10.0);
  return result;
}

template<typename Message>
rclcpp::SerializedMessage serialize(const Message & message) {
  rclcpp::SerializedMessage result;
  rclcpp::Serialization<Message>().serialize_message(&message, &result);
  return result;
}

TEST(TopicHealth, MissingDropoutAndRecovery) {
  auto stream = state();
  EXPECT_EQ(stream.snapshot(10.0).level, Status::STALE);
  EXPECT_EQ(stream.snapshot(10.0).message, "Never received");
  stream.receive(10.1);
  EXPECT_EQ(stream.snapshot(10.2).level, Status::OK);
  EXPECT_EQ(stream.snapshot(11.2).level, Status::STALE);
  EXPECT_DOUBLE_EQ(std::stod(stream.snapshot(16.0).values.at("observed_hz")), 0.0);
  stream.receive(16.1);
  EXPECT_EQ(stream.snapshot(16.2).level, Status::OK);
}

TEST(TopicHealth, RateUsesReceiptsEvenWhenValuesAreConstant) {
  auto stream = state();
  for (int i = 0; i < 100; ++i) {stream.receive(10.01 + i / 100.0);}
  EXPECT_NEAR(std::stod(stream.snapshot(11.0).values.at("observed_hz")), 100.0, 0.01);
  EXPECT_EQ(stream.total, 100u);
}

TEST(TopicHealth, RejectInvalidTimeouts) {
  for (auto timeout : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(StreamState("x", "/x", "x", timeout, 5.0, 10.0), std::invalid_argument);
    EXPECT_THROW(CasterReport("front_left", timeout), std::invalid_argument);
  }
}

TEST(TopicHealth, FirmwareFaultOverridesMissingJointOutput) {
  for (auto flags : {0u, 1u, 3u | 128u, 3u | 4096u}) {
    auto health = caster(report(flags));
    EXPECT_EQ(health.decorate("H1", state().snapshot(10.1), 10.1).level, Status::ERROR);
  }
}

TEST(TopicHealth, AdvancingSequenceStatusOverridesCachedJoint) {
  auto stream = state(); stream.receive(10.0);
  auto value = report(); value["nodes"][0]["stale"] = true;
  EXPECT_EQ(caster(value).decorate("H1", stream.snapshot(10.1), 10.1).level, Status::STALE);
}

TEST(TopicHealth, HealthyCasterNeedsFreshJointAndFreshDiagnosticReport) {
  auto stream = state(); stream.receive(10.0);
  auto health = caster(report());
  EXPECT_EQ(health.decorate("H1", stream.snapshot(10.1), 10.1).level, Status::OK);
  EXPECT_EQ(health.decorate("H1", stream.snapshot(11.1), 11.1).level, Status::STALE);
  stream.receive(13.0);
  EXPECT_EQ(health.decorate("H1", stream.snapshot(13.1), 13.1).level, Status::WARN);
}

TEST(TopicHealth, UnverifiedInventoryAndContinuityAreWarnings) {
  auto stream = state(); stream.receive(10.0);
  auto value = report(3 | 256); value["nodes"][0]["inventory_verified"] = false;
  auto result = caster(value).decorate("H1", stream.snapshot(10.1), 10.1);
  EXPECT_EQ(result.level, Status::WARN);
  EXPECT_EQ(result.message.find("Receiving;"), 0u);
  EXPECT_EQ(result.values.at("continuity_lost"), "true");
  EXPECT_EQ(result.values.at("inventory_verified"), "false");
}

TEST(TopicHealth, OldBootFaultIsNotReused) {
  auto value = report();
  auto old = value["streams"][0];
  old["boot_id"] = "old"; old["current_boot"] = false; old["last_flags"] = 0;
  value["streams"].insert(value["streams"].begin(), old);
  auto stream = state(); stream.receive(10.0);
  EXPECT_EQ(caster(value).decorate("H1", stream.snapshot(10.1), 10.1).level, Status::OK);
}

TEST(TopicHealth, MalformedReportCannotLeaveCachedOK) {
  for (const auto * text : {"{", "null", "[]", "{\"wheel_id\":\"front_right\"}",
    "{\"wheel_id\":\"front_left\",\"nodes\":[null],\"streams\":[]}"})
  {
    auto health = caster(report()); health.receive(text, 10.1);
    auto stream = state(); stream.receive(10.1);
    EXPECT_EQ(health.decorate("H1", stream.snapshot(10.2), 10.2).level, Status::WARN);
  }
}

TEST(TopicHealth, ZeroJointValuesAreValidAndMissingVelocityIsAllowed) {
  sensor_msgs::msg::JointState message;
  EXPECT_EQ(measurement_error("sensor_msgs/msg/JointState", serialize(message)), "Empty joint position");
  message.position = {0.0};
  EXPECT_EQ(measurement_error("sensor_msgs/msg/JointState", serialize(message)), "");
  message.position = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_EQ(measurement_error("sensor_msgs/msg/JointState", serialize(message)), "Non-finite measurement");
}

TEST(TopicHealth, InvalidImuAndOdometryAreReported) {
  sensor_msgs::msg::Imu imu;
  imu.angular_velocity.z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(measurement_error("sensor_msgs/msg/Imu", serialize(imu)), "Non-finite measurement");
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(measurement_error("nav_msgs/msg/Odometry", serialize(odom)), "Non-finite measurement");
}

TEST(TopicHealth, EveryDiagnosticLevelSerializes) {
  for (const auto level : {Status::OK, Status::WARN, Status::ERROR, Status::STALE}) {
    Status status;
    status.level = level;
    status.name = "caster/front_left/H1";
    EXPECT_EQ(decode<Status>(serialize(status)), status);
  }
}
