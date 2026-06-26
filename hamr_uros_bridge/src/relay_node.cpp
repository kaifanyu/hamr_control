#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <cerrno>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

using namespace std::chrono_literals;

namespace {

constexpr uint16_t MAGIC = 0xCAFE;
constexpr uint16_t VER   = 1;
constexpr uint16_t TYPE_CMD3 = 0x0011;
constexpr uint16_t TYPE_ENC  = 0x0003; // ESP->PC: encoder ticks (L,R,T)
constexpr uint16_t TYPE_IMU  = 0x0004; // ESP->PC: IMU data (roll/pitch/yaw, accel, gyro)
constexpr uint16_t TYPE_IMU_EXT = 0x0005; // ESP->PC: IMU + magnetometer + calibration

// Sign corrections applied once at unpack time in rx_loop().
// TICK_SIGN: ESP firmware negates wheel commands internally and its encoder
// ISRs match that convention, so raw ticks arrive negated relative to robot
// motion. The 2026-06-11 Vicon comparison showed the relay-published IMU yaw
// rate was already inverted relative to REP-103, so leave IMU yaw/gz unchanged
// here and handle wheel-odom yaw convention in holonomic_odom_node.
constexpr int32_t TICK_SIGN = 1;
constexpr float   YAW_SIGN  = 1.0f;

// Wire packet (packed)
#pragma pack(push,1)
struct PacketCmd3 {
  uint16_t magic;     // 0xCAFE
  uint16_t ver;       // 1
  uint16_t type;      // 0x0011
  uint32_t seq;
  uint64_t t_tx_ns;   // host monotonic send time
  float left;
  float right;
  float turret;
  uint16_t crc16;     // CRC32 folded to 16 bits
};
#pragma pack(pop)

static_assert(sizeof(PacketCmd3) == 2+2+2+4+8+4+4+4+2, "Packet size mismatch");

#pragma pack(push,1)
struct PacketEnc {
  uint16_t magic;     // 0xCAFE
  uint16_t ver;       // 1
  uint16_t type;      // 0x0003
  uint32_t seq;
  uint64_t t_tx_ns;   // device side send timestamp (ns)
  int32_t  ticksL;    // left encoder ticks
  int32_t  ticksR;    // right encoder ticks
  int32_t  ticksT;    // turret encoder ticks
  uint16_t crc16;     // CRC32 folded to 16 bits
};
#pragma pack(pop)

static_assert(sizeof(PacketEnc) == 32, "PacketEnc size mismatch");

// IMU Packet — must match IMUPacket in ESP main.cpp exactly (56 bytes)
#pragma pack(push,1)
struct PacketIMU {
  uint16_t magic;    // 0xCAFE
  uint16_t ver;      // 1
  uint16_t type;     // 0x0004
  uint32_t seq;
  uint64_t t_tx_ns;  // device side send timestamp (ns)
  float roll;        // rad, BNO055 Euler Y, wrapped to ±π
  float pitch;       // rad, BNO055 Euler Z, wrapped to ±π
  float yaw;         // rad, BNO055 Euler X (heading), wrapped to ±π
  float ax, ay, az;  // m/s², linear accel (gravity already removed by BNO055)
  float gx, gy, gz;  // rad/s, angular velocity
  uint16_t crc16;    // CRC32 folded to 16 bits
};
#pragma pack(pop)

static_assert(sizeof(PacketIMU) == 2+2+2+4+8+4+4+4+4+4+4+4+4+4+2, "PacketIMU size mismatch");

// Extended IMU Packet — must match IMUPacketExt in ESP main.cpp exactly (72 bytes)
#pragma pack(push,1)
struct PacketIMUExt {
  uint16_t magic;    // 0xCAFE
  uint16_t ver;      // 1
  uint16_t type;     // 0x0005
  uint32_t seq;
  uint64_t t_tx_ns;
  float roll, pitch, yaw;  // rad
  float ax, ay, az;        // m/s²
  float gx, gy, gz;        // rad/s
  float mx, my, mz;        // µT, raw magnetometer
  uint8_t cal_sys, cal_gyro, cal_accel, cal_mag; // BNO055 calibration 0..3
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(PacketIMUExt) == 2+2+2+4+8+4+4+4+4+4+4+4+4+4+4+4+4+1+1+1+1+2, "PacketIMUExt size mismatch");

// Simple CRC32 -> fold to 16 bits 
uint16_t crc16_fold(const uint8_t* data, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; ++i) {
    c ^= data[i];
    for (int k = 0; k < 8; ++k) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  c ^= 0xFFFFFFFFu;
  return static_cast<uint16_t>(c & 0xFFFFu);
}

speed_t baud_to_speed_t(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return B115200;
  }
}

class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  bool open(const std::string& dev, int baud) {
    close();
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      perror("open serial");
      return false;
    }

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      perror("tcgetattr");
      close();
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;            // 1 stop
    tio.c_cflag &= ~PARENB;            // no parity
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;                // 8 data bits

    const speed_t spd = baud_to_speed_t(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    tio.c_cc[VMIN]  = 0;   // non-blocking read
    tio.c_cc[VTIME] = 0;   // no inter-char timer

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      perror("tcsetattr");
      close();
      return false;
    }

    // clear buffers
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool write_all(const uint8_t* data, size_t size) {
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < size) {
      ssize_t n = ::write(fd_, data + sent, size - sent);
      if (n > 0) {
        sent += static_cast<size_t>(n);
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Brief sleep to yield
        ::usleep(100);
      } else {
        perror("serial write");
        return false;
      }
    }
    return true;
  }

  int fd() const { return fd_; }

private:
  int fd_ = -1;
};

} // namespace

class RelayNode : public rclcpp::Node {
public:
  RelayNode() : Node("hamr_uros_bridge") {
    // Parameters
    serial_port_  = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_         = this->declare_parameter<int>("baud", 460800);
    tx_rate_hz_   = this->declare_parameter<double>("tx_rate_hz", 100.0);
    command_timeout_s_ = std::max(
      0.01, this->declare_parameter<double>("command_timeout_s", 0.25));
    shutdown_zero_packets_ = static_cast<int>(std::max<int64_t>(
      1, this->declare_parameter<int64_t>("shutdown_zero_packets", 10)));
    imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", "imu_link");
    turret_command_sign_ = this->declare_parameter<double>("turret_command_sign", -1.0);

    // Open serial
    if (!serial_.open(serial_port_, baud_)) {
      RCLCPP_FATAL(get_logger(), "Failed to open serial %s @ %d", serial_port_.c_str(), baud_);
      throw std::runtime_error("serial open failed");
    }
    RCLCPP_INFO(
      get_logger(),
      "Serial open: %s @ %d; turret command sign: %.1f; command timeout: %.3fs",
      serial_port_.c_str(), baud_, turret_command_sign_, command_timeout_s_);

    // Subscriptions
    using std::placeholders::_1;
    sub_left_ = create_subscription<std_msgs::msg::Float64>(
      "/left_wheel/cmd_vel", rclcpp::QoS(1).best_effort(),
      std::bind(&RelayNode::left_cb, this, _1));
    sub_right_ = create_subscription<std_msgs::msg::Float64>(
      "/right_wheel/cmd_vel", rclcpp::QoS(1).best_effort(),
      std::bind(&RelayNode::right_cb, this, _1));
    sub_turret_ = create_subscription<std_msgs::msg::Float64>(
      "/turret/cmd_vel", rclcpp::QoS(1).best_effort(),
      std::bind(&RelayNode::turret_cb, this, _1));

    // Publishers (raw ticks)
    pub_ticks_l_ = create_publisher<std_msgs::msg::Int32>("/left_wheel/encoder_ticks", 10);
    pub_ticks_r_ = create_publisher<std_msgs::msg::Int32>("/right_wheel/encoder_ticks", 10);
    pub_ticks_t_ = create_publisher<std_msgs::msg::Int32>("/turret/encoder_ticks", 10);

    // IMU publisher
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    // Raw magnetometer (distortion analysis / offline calibration) and BNO055 calib levels
    pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    pub_calib_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/imu/calib_status", 10);

    // Timer for TX
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, tx_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RelayNode::tx_tick, this));

    // Start RX thread
    running_ = true;
    reader_ = std::thread([this]{ rx_loop(); });
  }

  ~RelayNode() override {
    if (timer_) timer_->cancel();

    // The MCU may retain its last velocity command, so explicitly overwrite
    // it before closing the serial port. Repetition makes a partially written
    // final packet or a short scheduling delay harmless.
    left_ = 0.0f;
    right_ = 0.0f;
    turret_ = 0.0f;
    for (int i = 0; i < shutdown_zero_packets_; ++i) {
      send_command(0.0f, 0.0f, 0.0f);
      std::this_thread::sleep_for(2ms);
    }

    running_ = false;
    if (reader_.joinable()) reader_.join();
  }

private:
  static int64_t steady_now_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void left_cb(const std_msgs::msg::Float64 & msg) {
    left_ = static_cast<float>(msg.data);
    last_left_cmd_ns_ = steady_now_ns();
  }

  void right_cb(const std_msgs::msg::Float64 & msg) {
    right_ = static_cast<float>(msg.data);
    last_right_cmd_ns_ = steady_now_ns();
  }

  void turret_cb(const std_msgs::msg::Float64 & msg)
  {
    turret_ = static_cast<float>(turret_command_sign_ * msg.data);
    last_turret_cmd_ns_ = steady_now_ns();
  }

  void tx_tick() {
    const int64_t now_ns = steady_now_ns();
    const int64_t timeout_ns = static_cast<int64_t>(command_timeout_s_ * 1e9);
    const auto fresh = [now_ns, timeout_ns](int64_t last_ns) {
      return last_ns > 0 && now_ns >= last_ns && (now_ns - last_ns) <= timeout_ns;
    };

    const bool left_fresh = fresh(last_left_cmd_ns_.load(std::memory_order_relaxed));
    const bool right_fresh = fresh(last_right_cmd_ns_.load(std::memory_order_relaxed));
    const bool turret_fresh = fresh(last_turret_cmd_ns_.load(std::memory_order_relaxed));

    const float left = left_fresh ? left_.load(std::memory_order_relaxed) : 0.0f;
    const float right = right_fresh ? right_.load(std::memory_order_relaxed) : 0.0f;
    const float turret = turret_fresh ? turret_.load(std::memory_order_relaxed) : 0.0f;

    if ((!left_fresh && left_.load(std::memory_order_relaxed) != 0.0f) ||
        (!right_fresh && right_.load(std::memory_order_relaxed) != 0.0f) ||
        (!turret_fresh && turret_.load(std::memory_order_relaxed) != 0.0f)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Actuator command timed out; sending zero for stale channels");
    }

    send_command(left, right, turret);
  }

  bool send_command(float left, float right, float turret) {
    PacketCmd3 pkt{};
    pkt.magic   = MAGIC;
    pkt.ver     = VER;
    pkt.type    = TYPE_CMD3;
    pkt.seq     = ++seq_;
    pkt.t_tx_ns = static_cast<uint64_t>(steady_now_ns());
    pkt.left    = left;
    pkt.right   = right;
    pkt.turret  = turret;

    pkt.crc16 = crc16_fold(reinterpret_cast<const uint8_t*>(&pkt), sizeof(PacketCmd3) - 2);

    const bool ok = serial_.write_all(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write failed");
    }
    return ok;
  }

  // Convert BNO055 Euler (roll, pitch, yaw) to quaternion using ZYX convention.
  // BNO055 NDOF gives absolute orientation; yaw is magnetic heading.
  void euler_to_quat(float roll, float pitch, float yaw,
                     double& qx, double& qy, double& qz, double& qw) {
    double cr = std::cos(roll  * 0.5);
    double sr = std::sin(roll  * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cy = std::cos(yaw   * 0.5);
    double sy = std::sin(yaw   * 0.5);
    qw = cr * cp * cy + sr * sp * sy;
    qx = sr * cp * cy - cr * sp * sy;
    qy = cr * sp * cy + sr * cp * sy;
    qz = cr * cp * sy - sr * sp * cy;
  }

  void publish_imu_data(const PacketIMU& p) {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp    = rclcpp::Clock(RCL_ROS_TIME).now();
    msg.header.frame_id = imu_frame_id_;

    // Orientation from BNO055 fusion (absolute, magnetometer-referenced)
    double qx, qy, qz, qw;
    euler_to_quat(p.roll, p.pitch, p.yaw, qx, qy, qz, qw);
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    // BNO055 NDOF heading accuracy ~2°, roll/pitch ~1°
    msg.orientation_covariance = {
      0.0003, 0,      0,
      0,      0.0003, 0,
      0,      0,      0.0009
    };

    // Angular velocity from BNO055 gyroscope (rad/s)
    msg.angular_velocity.x = static_cast<double>(p.gx);
    msg.angular_velocity.y = static_cast<double>(p.gy);
    msg.angular_velocity.z = static_cast<double>(p.gz);
    msg.angular_velocity_covariance = {
      0.0002, 0,      0,
      0,      0.0002, 0,
      0,      0,      0.0002
    };

    // Linear acceleration from BNO055 VECTOR_LINEARACCEL (gravity already removed)
    msg.linear_acceleration.x = static_cast<double>(p.ax);
    msg.linear_acceleration.y = static_cast<double>(p.ay);
    msg.linear_acceleration.z = static_cast<double>(p.az);
    msg.linear_acceleration_covariance = {
      0.01, 0,    0,
      0,    0.01, 0,
      0,    0,    0.01
    };

    pub_imu_->publish(msg);
  }

  // Publish raw magnetometer (µT → Tesla) for distortion analysis / offline calibration.
  void publish_imu_mag(const PacketIMUExt& p) {
    auto m = sensor_msgs::msg::MagneticField();
    m.header.stamp    = rclcpp::Clock(RCL_ROS_TIME).now();
    m.header.frame_id = imu_frame_id_;
    m.magnetic_field.x = static_cast<double>(p.mx) * 1e-6;
    m.magnetic_field.y = static_cast<double>(p.my) * 1e-6;
    m.magnetic_field.z = static_cast<double>(p.mz) * 1e-6;
    pub_mag_->publish(m);
  }

  // Publish BNO055 calibration levels [sys, gyro, accel, mag], each 0..3.
  void publish_calib_status(const PacketIMUExt& p) {
    auto c = std_msgs::msg::UInt8MultiArray();
    c.data = {p.cal_sys, p.cal_gyro, p.cal_accel, p.cal_mag};
    pub_calib_->publish(c);
  }

  // --- RX loop: parse encoder packets from ESP and publish ---
  void rx_loop() {
    std::vector<uint8_t> buf;
    buf.reserve(512);

    while (rclcpp::ok() && running_) {
      // Non-blocking read from serial
      uint8_t tmp[128];
      ssize_t n = ::read(serial_.fd(), tmp, sizeof(tmp));
      if (n > 0) {
        buf.insert(buf.end(), tmp, tmp + n);
      } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        std::this_thread::sleep_for(1ms);
      } else if (n < 0) {
        // unexpected error: brief backoff
        std::this_thread::sleep_for(5ms);
      }

      // Parse frames
      for (;;) {
        if (buf.size() < 6) break;

        // resync to MAGIC
        uint16_t magic = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        if (magic != MAGIC) { buf.erase(buf.begin()); continue; }

        if (buf.size() < 6) break;
        uint16_t ver  = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint16_t type = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        if (ver != VER) { buf.erase(buf.begin()); continue; }

        size_t need = 0;
        if (type == TYPE_ENC) {
          need = sizeof(PacketEnc);
        } else if (type == TYPE_IMU) {
          need = sizeof(PacketIMU);
        } else if (type == TYPE_IMU_EXT) {
          need = sizeof(PacketIMUExt);
        } else {
          // Unknown type → drop 1 byte and resync
          buf.erase(buf.begin());
          continue;
        }

        if (buf.size() < need) break;

        if (type == TYPE_ENC) {
          PacketEnc p{};
          std::memcpy(&p, buf.data(), need);

          uint16_t calc = crc16_fold(reinterpret_cast<const uint8_t*>(&p), need - 2);
          if (calc != p.crc16) {
            buf.erase(buf.begin());
            continue;
          }

          std_msgs::msg::Int32 mL; mL.data = TICK_SIGN * p.ticksL; pub_ticks_l_->publish(mL);
          std_msgs::msg::Int32 mR; mR.data = TICK_SIGN * p.ticksR; pub_ticks_r_->publish(mR);
          std_msgs::msg::Int32 mT; mT.data = p.ticksT; pub_ticks_t_->publish(mT);
        } else if (type == TYPE_IMU) {
          PacketIMU p{};
          std::memcpy(&p, buf.data(), need);

          uint16_t calc = crc16_fold(reinterpret_cast<const uint8_t*>(&p), need - 2);
          if (calc != p.crc16) {
            buf.erase(buf.begin());
            continue;
          }

          p.yaw *= YAW_SIGN;
          p.gz  *= YAW_SIGN;
          publish_imu_data(p);
        } else if (type == TYPE_IMU_EXT) {
          PacketIMUExt pe{};
          std::memcpy(&pe, buf.data(), need);

          uint16_t calc = crc16_fold(reinterpret_cast<const uint8_t*>(&pe), need - 2);
          if (calc != pe.crc16) {
            buf.erase(buf.begin());
            continue;
          }

          pe.yaw *= YAW_SIGN;
          pe.gz  *= YAW_SIGN;

          // Republish the standard /imu/data from the shared orientation/accel/gyro fields...
          PacketIMU base{};
          base.roll = pe.roll; base.pitch = pe.pitch; base.yaw = pe.yaw;
          base.ax = pe.ax; base.ay = pe.ay; base.az = pe.az;
          base.gx = pe.gx; base.gy = pe.gy; base.gz = pe.gz;
          publish_imu_data(base);

          // ...plus the new raw-magnetometer and calibration-status topics.
          publish_imu_mag(pe);
          publish_calib_status(pe);
        }

        // consume this frame
        buf.erase(buf.begin(), buf.begin() + need);
      }
    }
  }

  // Params
  std::string serial_port_;
  int baud_;
  double tx_rate_hz_;
  double command_timeout_s_;
  int shutdown_zero_packets_;
  double turret_command_sign_;
  std::string imu_frame_id_;

  // Serial
  SerialPort serial_;

  // State (atomic so callbacks + timer are safe)
  std::atomic<float> left_{0.0f}, right_{0.0f}, turret_{0.0f};
  std::atomic<int64_t> last_left_cmd_ns_{0};
  std::atomic<int64_t> last_right_cmd_ns_{0};
  std::atomic<int64_t> last_turret_cmd_ns_{0};
  uint32_t seq_{0};

  // ROS
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_left_, sub_right_, sub_turret_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_ticks_l_, pub_ticks_r_, pub_ticks_t_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_calib_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Reader thread
  std::thread reader_;
  std::atomic<bool> running_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<RelayNode>());
  } catch (const std::exception& e) {
    fprintf(stderr, "RelayNode exception: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
