// Wheeltec N100 IMU driver (FDILink protocol) for Qmini.
//
// Self-contained: opens the serial port via raw termios (no external serial
// library), runs a byte-synced frame parser on a dedicated read thread, and
// publishes sensor_msgs/Imu on /imu/data. Kept in its own process so a stalled
// IMU port can never block the realtime motor bus (qmini_hardware).
//
// What it publishes per AHRS frame:
//   orientation         <- AHRS fused quaternion (optionally ROS-frame rotated)
//   angular_velocity    <- IMU raw gyroscope (rad/s)        [policy imu_ang_vel]
//   linear_acceleration <- IMU raw accelerometer (m/s^2)    [policy gravity src]
//
// FRAME ALIGNMENT IS NOT FINAL HERE. The Isaac Lab policy expects the IMU frame
// to match the sim mount: ImuCfg offset pos=(-0.04718, 0.0663, 0.11094),
// rot=(1,0,0,0) identity relative to base_link. Verifying/fixing that mapping
// is exactly what M4 calibration does. This driver publishes the sensor's own
// frame; the M5 observation assembler applies the final base_link transform.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "qmini_imu/fdilink_protocol.hpp"

namespace qmini_imu
{

using namespace std::chrono_literals;
namespace fd = fdilink;

// Map a numeric baud rate to the termios Bxxx constant. Only the rates the
// N100 can be configured for are listed; default firmware is 921600.
speed_t to_baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B0;
  }
}

// Quaternions are (w, x, y, z). Hamilton product a ⊗ b.
std::array<double, 4> quat_mul(const std::array<double, 4> & a, const std::array<double, 4> & b)
{
  return {
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
}

// Rotate vector v by quaternion q: v' = R(q) v.
std::array<double, 3> quat_rotate(const std::array<double, 4> & q, const std::array<double, 3> & v)
{
  const double w = q[0];
  const std::array<double, 3> u = {q[1], q[2], q[3]};
  const double uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
  const double uv = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const std::array<double, 3> cross = {
    u[1] * v[2] - u[2] * v[1],
    u[2] * v[0] - u[0] * v[2],
    u[0] * v[1] - u[1] * v[0]};
  const double s = w * w - uu;
  std::array<double, 3> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = s * v[i] + 2.0 * uv * u[i] + 2.0 * w * cross[i];
  }
  return out;
}

class ImuDriverNode : public rclcpp::Node
{
public:
  ImuDriverNode()
  : Node("imu_driver_node")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_rate_ = declare_parameter<int>("baud_rate", 921600);
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
    auto imu_topic = declare_parameter<std::string>("imu_topic", "imu/data");
    // device_type==1 in the vendor driver rotates the AHRS quaternion 180 deg
    // about Y to land in a ROS-standard frame. Default true reproduces the
    // known-working vendor output; M4 confirms it against the base_link mount.
    apply_ros_transform_ = declare_parameter<bool>("apply_ros_transform", true);
    // Mount rotation R (chip/sensor frame -> base_link), as a quaternion
    // [w,x,y,z]. Applied uniformly to orientation (post-multiply), gyro, and
    // accel so all three land in base_link. Default identity; this robot's
    // verified mount is 180 deg about Y = [0,0,1,0] (the official SDK's
    // (-1,+1,-1) sign flip on x,z) — set in config/imu.yaml. M4 records it.
    {
      auto m = declare_parameter<std::vector<double>>(
        "mount_rotation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
      if (m.size() != 4) {
        RCLCPP_FATAL(get_logger(), "mount_rotation must be 4 numbers [w,x,y,z]");
        throw std::runtime_error("mount_rotation size");
      }
      double n = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2] + m[3] * m[3]);
      if (n < 1e-9) {n = 1.0;}
      mount_rotation_ = {m[0] / n, m[1] / n, m[2] / n, m[3] / n};
    }
    // Covariance diagonals (0 = "unknown" per REP-145; we publish small values
    // so downstream filters don't treat the data as garbage). Tune in M4.
    gyro_cov_ = declare_parameter<double>("gyro_covariance", 0.01);
    accel_cov_ = declare_parameter<double>("accel_covariance", 0.05);
    orient_cov_ = declare_parameter<double>("orientation_covariance", 0.01);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SensorDataQoS());

    running_ = true;
    read_thread_ = std::thread(&ImuDriverNode::run, this);
  }

  ~ImuDriverNode() override
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

private:
  // --- serial -------------------------------------------------------------
  bool open_port()
  {
    fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Cannot open IMU serial port '%s': %s", serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    const speed_t baud = to_baud_constant(baud_rate_);
    if (baud == B0) {
      RCLCPP_FATAL(get_logger(), "Unsupported baud rate %d", baud_rate_);
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcgetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    cfmakeraw(&tty);           // 8N1, no flow control, no echo, no signal chars
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    // Blocking read with a 0.2 s inter-byte timeout so the thread can notice
    // shutdown and port loss instead of blocking forever.
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 2;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "tcsetattr failed: %s", std::strerror(errno));
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    tcflush(fd_, TCIFLUSH);
    RCLCPP_INFO(get_logger(), "Opened IMU serial port %s @ %d baud", serial_port_.c_str(), baud_rate_);
    return true;
  }

  // Read exactly n bytes (looping over partial reads). Returns false on EOF,
  // error, or shutdown.
  bool read_exact(uint8_t * buf, size_t n)
  {
    size_t got = 0;
    while (got < n && running_) {
      ssize_t r = ::read(fd_, buf + got, n - got);
      if (r < 0) {
        if (errno == EINTR) {continue;}
        return false;
      }
      if (r == 0) {continue;}  // VTIME timeout, no bytes yet — keep waiting
      got += static_cast<size_t>(r);
    }
    return got == n;
  }

  // --- frame parsing ------------------------------------------------------
  void run()
  {
    while (running_) {
      if (fd_ < 0 && !open_port()) {
        std::this_thread::sleep_for(1s);
        continue;
      }
      if (!parse_one_frame()) {
        // read error / port lost — close and retry from open_port().
        if (fd_ >= 0) {::close(fd_); fd_ = -1;}
      }
    }
  }

  // Returns false only on a serial read failure (triggers reopen). A bad/
  // dropped frame just resyncs and returns true.
  bool parse_one_frame()
  {
    // 1. sync to the 0xFC start byte
    uint8_t b = 0;
    if (!read_exact(&b, 1)) {return false;}
    if (b != fd::kFrameHead) {return true;}

    // 2. read the rest of the 7-byte header
    std::array<uint8_t, fd::kHeaderBytes> hdr{};
    hdr[0] = fd::kFrameHead;
    if (!read_exact(hdr.data() + 1, fd::kHeaderBytes - 1)) {return false;}

    const uint8_t data_type = hdr[1];
    const uint8_t data_size = hdr[2];
    const uint8_t header_crc8 = hdr[4];
    const uint16_t header_crc16 = (static_cast<uint16_t>(hdr[5]) << 8) | hdr[6];

    // 3. validate header CRC-8 over the first 4 bytes
    if (fd::crc8(hdr.data(), 4) != header_crc8) {
      ++crc8_errors_;
      return true;
    }

    // 4. only IMU / AHRS frames carry what we need; skip others by length
    const bool is_imu = (data_type == fd::kTypeImu && data_size == fd::kImuLen);
    const bool is_ahrs = (data_type == fd::kTypeAhrs && data_size == fd::kAhrsLen);

    // 5. read payload + frame_end byte
    std::array<uint8_t, 256> payload{};
    if (!read_exact(payload.data(), data_size)) {return false;}
    uint8_t frame_end = 0;
    if (!read_exact(&frame_end, 1)) {return false;}

    if (!is_imu && !is_ahrs) {return true;}  // unknown type, validly skipped

    // 6. validate payload CRC-16 and frame end
    if (fd::crc16(payload.data(), data_size) != header_crc16) {
      ++crc16_errors_;
      return true;
    }
    if (frame_end != fd::kFrameEnd) {return true;}

    // 7. dispatch
    if (is_imu) {
      std::memcpy(&imu_pkt_, payload.data(), sizeof(fd::ImuPacket));
      have_imu_ = true;
    } else {  // AHRS frame — the "complete" frame, so publish here
      fd::AhrsPacket ahrs;
      std::memcpy(&ahrs, payload.data(), sizeof(fd::AhrsPacket));
      publish_imu(ahrs);
    }
    return true;
  }

  void publish_imu(const fd::AhrsPacket & ahrs)
  {
    sensor_msgs::msg::Imu msg;
    // Stamp at receive time in the driver layer — calibration latency numbers
    // (M4) are only as good as this timestamp; do not re-stamp downstream.
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    // 1. orientation in the sensor's ROS frame (optional vendor device_type==1
    //    180-deg-about-Y on the quaternion).
    std::array<double, 4> q_sensor;  // (w,x,y,z)
    if (apply_ros_transform_) {
      q_sensor = {-ahrs.qy, ahrs.qz, ahrs.qw, -ahrs.qx};
    } else {
      q_sensor = {ahrs.qw, ahrs.qx, ahrs.qy, ahrs.qz};
    }

    // 2. raw gyro/accel in the sensor frame.
    std::array<double, 3> gyro, accel;
    if (have_imu_) {
      gyro = {imu_pkt_.gyroscope_x, imu_pkt_.gyroscope_y, imu_pkt_.gyroscope_z};
      accel = {imu_pkt_.accelerometer_x, imu_pkt_.accelerometer_y, imu_pkt_.accelerometer_z};
    } else {
      // No IMU frame yet — fall back to AHRS body rates; accel stays zero.
      gyro = {ahrs.roll_speed, ahrs.pitch_speed, ahrs.heading_speed};
      accel = {0.0, 0.0, 0.0};
    }

    // 3. apply the mount rotation R (base_link <- sensor) uniformly: orientation
    //    post-multiplied so quat_rotate_inverse downstream yields base-frame
    //    gravity; gyro/accel rotated by R. Default R is identity.
    const auto q_base = quat_mul(q_sensor, mount_rotation_);
    gyro = quat_rotate(mount_rotation_, gyro);
    accel = quat_rotate(mount_rotation_, accel);

    msg.orientation.w = q_base[0];
    msg.orientation.x = q_base[1];
    msg.orientation.y = q_base[2];
    msg.orientation.z = q_base[3];
    msg.angular_velocity.x = gyro[0];
    msg.angular_velocity.y = gyro[1];
    msg.angular_velocity.z = gyro[2];
    msg.linear_acceleration.x = accel[0];
    msg.linear_acceleration.y = accel[1];
    msg.linear_acceleration.z = accel[2];

    msg.orientation_covariance[0] = orient_cov_;
    msg.orientation_covariance[4] = orient_cov_;
    msg.orientation_covariance[8] = orient_cov_;
    msg.angular_velocity_covariance[0] = gyro_cov_;
    msg.angular_velocity_covariance[4] = gyro_cov_;
    msg.angular_velocity_covariance[8] = gyro_cov_;
    msg.linear_acceleration_covariance[0] = accel_cov_;
    msg.linear_acceleration_covariance[4] = accel_cov_;
    msg.linear_acceleration_covariance[8] = accel_cov_;

    imu_pub_->publish(msg);
  }

  // params
  std::string serial_port_;
  int baud_rate_{921600};
  std::string frame_id_;
  bool apply_ros_transform_{true};
  std::array<double, 4> mount_rotation_{1.0, 0.0, 0.0, 0.0};  // [w,x,y,z], base<-sensor
  double gyro_cov_{0.01};
  double accel_cov_{0.05};
  double orient_cov_{0.01};

  // serial + thread
  int fd_{-1};
  std::atomic<bool> running_{false};
  std::thread read_thread_;

  // latest IMU frame (gyro/accel), merged into each AHRS publish
  fd::ImuPacket imu_pkt_{};
  bool have_imu_{false};

  // diagnostics
  uint64_t crc8_errors_{0};
  uint64_t crc16_errors_{0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace qmini_imu

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<qmini_imu::ImuDriverNode>());
  rclcpp::shutdown();
  return 0;
}
