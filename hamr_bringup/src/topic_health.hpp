#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <nlohmann/json.hpp>

namespace hamr_health {
using Json = nlohmann::json;
using Status = diagnostic_msgs::msg::DiagnosticStatus;

inline double positive(double value, const char * name) {
  if (!std::isfinite(value) || value <= 0) {
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
  }
  return value;
}

struct Report {
  uint8_t level = Status::OK;
  std::string message = "Receiving";
  std::map<std::string, std::string> values;
};

struct StreamState {
  std::string name, topic, type, error;
  double timeout, window, started;
  std::optional<double> last;
  std::deque<double> receipts;
  uint64_t total = 0;

  StreamState(std::string name_, std::string topic_, std::string type_,
              double timeout_, double window_, double now)
  : name(std::move(name_)), topic(std::move(topic_)), type(std::move(type_)),
    timeout(positive(timeout_, "timeout_s")), window(positive(window_, "rate_window_s")), started(now) {}

  void prune(double now) {
    while (!receipts.empty() && receipts.front() < now - window) {receipts.pop_front();}
  }

  void receive(double now) {
    last = now;
    ++total;
    receipts.push_back(now);
    prune(now);
    if (receipts.size() > 20000) {receipts.pop_front();}
  }

  Report snapshot(double now) {
    prune(now);
    Report result;
    result.values = {
      {"topic", topic}, {"age_s", last ? std::to_string(std::max(0.0, now - *last)) : "never"},
      {"observed_hz", std::to_string(receipts.size() / std::min(window, std::max(0.001, now - started)))},
      {"received_messages", std::to_string(total)}, {"timeout_s", std::to_string(timeout)},
      {"freshness_basis", "host receipt; not sensor acquisition rate or accuracy"},
    };
    if (!last) {result.level = Status::STALE; result.message = "Never received";}
    else if (now - *last > timeout) {result.level = Status::STALE; result.message = "No recent messages";}
    else if (!error.empty()) {result.level = Status::ERROR; result.message = error;}
    return result;
  }
};

inline const std::vector<std::string> flag_names = {
  "ready", "sample_valid", "ppr_verified", "phase_reference_valid", "unwrap_continuous",
  "clock_sync_valid", "velocity_valid", "parity_error", "continuity_lost", "index_valid",
  "rate_bound_verified", "acquisition_overrun", "spi_io_error",
};

struct CasterReport {
  std::string wheel, error;
  double timeout;
  std::optional<double> last;
  Json data;

  CasterReport(std::string wheel_, double timeout_)
  : wheel(std::move(wheel_)), timeout(positive(timeout_, "caster_diagnostic_timeout_s")) {}

  void receive(const std::string & text, double now) {
    last = now;
    data = nullptr;
    try {
      auto parsed = Json::parse(text);
      if (!parsed.is_object() || parsed.value("wheel_id", "") != wheel ||
        !parsed.contains("nodes") || !parsed.at("nodes").is_array() ||
        !parsed.contains("streams") || !parsed.at("streams").is_array())
      {throw std::invalid_argument("Wrong wheel or malformed report");}
      for (const auto * key : {"nodes", "streams"}) {
        for (const auto & entry : parsed.at(key)) {
          if (!entry.is_object()) {throw std::invalid_argument("Malformed entry");}
        }
      }
      data = std::move(parsed);
      error.clear();
    } catch (const std::exception & ex) {error = ex.what();}
  }

  Report decorate(const std::string & role, Report result, double now) const {
    auto mark = [&result](uint8_t level, const std::string & text) {
        result.level = level; result.message = text; return result;
      };
    if (!last || now - *last > timeout || data.is_null()) {
      auto reason = error.empty() ? "Caster diagnostics unavailable" : "Invalid caster diagnostics: " + error;
      result.values["caster_diagnostics"] = reason;
      return result.level == Status::OK ? mark(Status::WARN, reason) : result;
    }
    try {
      const auto & nodes = data.at("nodes");
      auto node = std::find_if(nodes.begin(), nodes.end(), [&](const auto & n) {return n.value("role", "") == role;});
      if (node == nodes.end()) {return mark(Status::STALE, "Role absent from caster diagnostics");}
      for (const auto * key : {"device_id", "source_ip", "inventory_verified", "metadata_received", "sample_age_s"}) {
        result.values[key] = node->contains(key) ? node->at(key).dump() : "null";
      }
      result.values["rejected_packets"] = data.value("rejected_packets", Json(0)).dump();
      result.values["last_rejection"] = data.value("last_rejection", Json("")).dump();
      if (node->value("never_seen", true) || node->value("stale", true)) {
        return mark(Status::STALE, "No advancing caster samples");
      }
      const auto & streams = data.at("streams");
      auto stream = std::find_if(streams.begin(), streams.end(), [&](const auto & s) {
          return s.value("role", "") == role && s.value("device_id", Json()) == node->value("device_id", Json()) &&
                 s.value("boot_id", Json()) == node->value("boot_id", Json()) && s.value("current_boot", false);
        });
      if (stream == streams.end() || !stream->contains("last_flags") || !stream->at("last_flags").is_number_integer()) {
        return mark(Status::WARN, "Caster validity flags unavailable");
      }
      auto flags = stream->at("last_flags").get<uint32_t>();
      auto bit = [flags](unsigned index) {return (flags & (1u << index)) != 0;};
      for (size_t index = 0; index < flag_names.size(); ++index) {
        result.values[flag_names[index]] = bit(index) ? "true" : "false";
      }
      result.values["missing_samples_in_recording"] = stream->value("missing_samples_in_recording", Json(0)).dump();
      result.values["firmware_diagnostics"] = stream->value("firmware_diagnostics", Json::object()).dump();
      if (!bit(0) || !bit(1) || bit(7) || bit(12)) {
        return mark(Status::ERROR, "Caster reports invalid sample / hardware fault");
      }
      if (result.level != Status::OK) {return result;}
      std::vector<std::string> warnings;
      if (bit(8) || !bit(4)) {warnings.emplace_back("angle continuity unverified");}
      if (bit(11)) {warnings.emplace_back("acquisition overrun");}
      if (!node->value("inventory_verified", false)) {warnings.emplace_back("device inventory unverified");}
      if ((role == "H1" || role == "H2") && !bit(2)) {warnings.emplace_back("PPR unverified");}
      if (role == "CENTER" && !bit(10)) {warnings.emplace_back("rate bound unverified");}
      if (!warnings.empty()) {
        result.level = Status::WARN;
        for (const auto & warning : warnings) {result.message += "; " + warning;}
      }
      return result;
    } catch (const std::exception & ex) {
      return mark(Status::WARN, std::string("Invalid caster diagnostics: ") + ex.what());
    }
  }
};
}  // namespace hamr_health
