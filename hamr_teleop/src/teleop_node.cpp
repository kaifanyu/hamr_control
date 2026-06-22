#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <string>
#include <thread>

class Teleoperation : public rclcpp::Node {
public:
  Teleoperation()
  : Node("teleop_node")
  {
    mix_scale_ = this->declare_parameter<double>("mix_scale", 0.8);
    max_rpm_cmd_ = this->declare_parameter<double>("max_rpm_cmd", 30.0);
    turret_max_rad_s_ = this->declare_parameter<double>("turret_max_rad_s", 2.0);
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 100.0);
    stale_timeout_s_ = this->declare_parameter<double>("stale_timeout_s", 100.0);
    ema_tau_s_ = this->declare_parameter<double>("ema_tau_s", 0.05);

    const double rpm_to_rad_s = 2.0 * M_PI / 60.0;
    max_rad_s_ = max_rpm_cmd_ * rpm_to_rad_s;

    auto qos = rclcpp::QoS(1).best_effort();
    pub_left_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel/cmd_vel", qos);
    pub_right_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel/cmd_vel", qos);
    pub_turret_ = this->create_publisher<std_msgs::msg::Float64>("/turret/cmd_vel", qos);

    running_ = true;
    keyboard_reader_ = std::thread([this] { this->keyboard_loop(); });

    timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(1e6 / std::max(1.0, publish_rate_hz_))),
      std::bind(&Teleoperation::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "keyboard teleop ready: W/S forward/back, A/D turn, Q/E turret, Space stop "
      "(max_rpm=%.1f, max_rad_s=%.3f)",
      max_rpm_cmd_, max_rad_s_);
  }

  ~Teleoperation() override
  {
    running_ = false;
    if (keyboard_reader_.joinable()) {
      keyboard_reader_.join();
    }
    publish_zero();
  }

private:
  class TerminalMode {
  public:
    explicit TerminalMode(rclcpp::Logger logger)
    : logger_(logger)
    {
      if (!isatty(STDIN_FILENO)) {
        RCLCPP_WARN(logger_, "stdin is not a TTY; keyboard controls are disabled");
        return;
      }

      if (tcgetattr(STDIN_FILENO, &original_termios_) != 0) {
        RCLCPP_WARN(logger_, "could not read terminal settings; keyboard controls are disabled");
        return;
      }

      original_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
      if (original_flags_ < 0) {
        RCLCPP_WARN(logger_, "could not read stdin flags; keyboard controls are disabled");
        return;
      }

      termios raw = original_termios_;
      raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;

      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
        RCLCPP_WARN(logger_, "could not switch terminal to keyboard mode");
        return;
      }

      if (fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK) != 0) {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        RCLCPP_WARN(logger_, "could not make stdin nonblocking");
        return;
      }

      active_ = true;
    }

    ~TerminalMode()
    {
      restore();
    }

    bool active() const
    {
      return active_;
    }

    void restore()
    {
      if (!active_) {
        return;
      }
      tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
      fcntl(STDIN_FILENO, F_SETFL, original_flags_);
      active_ = false;
    }

  private:
    rclcpp::Logger logger_;
    termios original_termios_{};
    int original_flags_{0};
    bool active_{false};
  };

  static inline double now_steady()
  {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
  }

  void keyboard_loop()
  {
    TerminalMode terminal(this->get_logger());
    if (!terminal.active()) {
      return;
    }

    RCLCPP_INFO(this->get_logger(), "keyboard input active; press Space for full stop");

    while (running_) {
      pollfd input{};
      input.fd = STDIN_FILENO;
      input.events = POLLIN;

      const int ready = poll(&input, 1, 100);
      if (ready <= 0 || !(input.revents & POLLIN)) {
        continue;
      }

      char key = '\0';
      while (read(STDIN_FILENO, &key, 1) == 1) {
        handle_key(key);
      }
    }
  }

  void handle_key(char key)
  {
    const char lower_key = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));

    switch (lower_key) {
      case 'w':
        set_command(1.0, 0.0, 0.0);
        break;
      case 's':
        set_command(-1.0, 0.0, 0.0);
        break;
      case 'a':
        set_command(0.0, -1.0, 0.0);
        break;
      case 'd':
        set_command(0.0, 1.0, 0.0);
        break;
      case 'q':
        set_command(0.0, 0.0, 1.0);
        break;
      case 'e':
        set_command(0.0, 0.0, -1.0);
        break;
      case ' ':
        set_command(0.0, 0.0, 0.0);
        stop_requested_ = true;
        break;
      default:
        break;
    }
  }

  void set_command(double forward, double turn, double turret)
  {
    forward_axis_ = forward;
    turn_axis_ = turn;
    turret_axis_ = turret;
    last_key_time_ = now_steady();
  }

  void on_timer()
  {
    double forward = forward_axis_.load();
    double turn = turn_axis_.load();
    double turret = turret_axis_.load();

    const double tnow = now_steady();
    if ((tnow - last_key_time_.load()) > stale_timeout_s_) {
      forward = 0.0;
      turn = 0.0;
      turret = 0.0;
    }

    forward *= mix_scale_;
    turn *= mix_scale_;

    const double left_cmd = (forward + turn) * max_rad_s_;
    const double right_cmd = (forward - turn) * max_rad_s_;
    const double turret_cmd = turret * turret_max_rad_s_;

    const double dt = 1.0 / std::max(1.0, publish_rate_hz_);
    const double a = (ema_tau_s_ > 1e-4) ? dt / (ema_tau_s_ + dt) : 1.0;

    if (stop_requested_.exchange(false)) {
      left_f_ = 0.0;
      right_f_ = 0.0;
      turret_f_ = 0.0;
    } else {
      left_f_ += a * (left_cmd - left_f_);
      right_f_ += a * (right_cmd - right_f_);
      turret_f_ += a * (turret_cmd - turret_f_);
    }

    publish_command(left_f_, right_f_, turret_f_);
  }

  void publish_command(double left, double right, double turret)
  {
    std_msgs::msg::Float64 msg;
    msg.data = left;
    pub_left_->publish(msg);
    msg.data = right;
    pub_right_->publish(msg);
    msg.data = turret;
    pub_turret_->publish(msg);
  }

  void publish_zero()
  {
    if (pub_left_ && pub_right_ && pub_turret_) {
      publish_command(0.0, 0.0, 0.0);
    }
  }

  double mix_scale_{0.8};
  double max_rpm_cmd_{30.0};
  double max_rad_s_{0.0};
  double turret_max_rad_s_{2.0};
  double publish_rate_hz_{100.0};
  double stale_timeout_s_{100.0};
  double ema_tau_s_{0.05};

  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<double> forward_axis_{0.0};
  std::atomic<double> turn_axis_{0.0};
  std::atomic<double> turret_axis_{0.0};
  std::atomic<double> last_key_time_{0.0};
  std::thread keyboard_reader_;
  rclcpp::TimerBase::SharedPtr timer_;

  double left_f_{0.0};
  double right_f_{0.0};
  double turret_f_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_left_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_right_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_turret_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleoperation>());
  rclcpp::shutdown();
  return 0;
}
