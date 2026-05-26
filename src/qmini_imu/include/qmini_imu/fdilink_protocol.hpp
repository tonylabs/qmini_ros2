// FDILink serial protocol for the Wheeltec N100 IMU.
//
// Frame layout (little-endian, byte-packed):
//
//   [0] header_start  = 0xFC
//   [1] data_type     = 0x40 IMU | 0x41 AHRS | 0x42 INSGPS
//   [2] data_size     = payload length (0x38=56 IMU, 0x30=48 AHRS)
//   [3] serial_num    = rolling sequence counter (for drop detection)
//   [4] header_crc8   = CRC-8 over bytes [0..3]
//   [5] header_crc16_h \  CRC-16/CCITT over the payload bytes only
//   [6] header_crc16_l /  (big-endian split: h<<8 | l)
//   [7 .. 7+data_size-1] payload
//   [last] frame_end  = 0xFD
//
// The N100 streams IMU frames (raw gyro/accel/mag) and AHRS frames (fused
// orientation + body rates) interleaved. We assemble one sensor_msgs/Imu per
// AHRS frame using the most recent IMU frame's gyro/accel.
//
// Struct field order/units are taken verbatim from the vendor's
// fdilink_data_struct.h (see qmini_imu README for provenance).
#ifndef QMINI_IMU__FDILINK_PROTOCOL_HPP_
#define QMINI_IMU__FDILINK_PROTOCOL_HPP_

#include <cstdint>

namespace qmini_imu::fdilink
{

constexpr uint8_t kFrameHead = 0xFC;
constexpr uint8_t kFrameEnd = 0xFD;

constexpr uint8_t kTypeImu = 0x40;
constexpr uint8_t kTypeAhrs = 0x41;
constexpr uint8_t kTypeInsgps = 0x42;

constexpr uint8_t kImuLen = 0x38;     // 56
constexpr uint8_t kAhrsLen = 0x30;    // 48
constexpr uint8_t kInsgpsLen = 0x54;  // 84

constexpr int kHeaderBytes = 7;   // start, type, size, sn, crc8, crc16_h, crc16_l

#pragma pack(push, 1)
struct ImuPacket
{
  float gyroscope_x;          // rad/s
  float gyroscope_y;          // rad/s
  float gyroscope_z;          // rad/s
  float accelerometer_x;      // m/s^2
  float accelerometer_y;      // m/s^2
  float accelerometer_z;      // m/s^2
  float magnetometer_x;       // mG
  float magnetometer_y;       // mG
  float magnetometer_z;       // mG
  float imu_temperature;      // C
  float pressure;             // Pa
  float pressure_temperature; // C
  int64_t timestamp;          // us (device clock — NOT used for ROS stamp)
};

struct AhrsPacket
{
  float roll_speed;     // rad/s
  float pitch_speed;    // rad/s
  float heading_speed;  // rad/s
  float roll;           // rad
  float pitch;          // rad
  float heading;        // rad
  float qw;             // quaternion (w, x, y, z)
  float qx;
  float qy;
  float qz;
  int64_t timestamp;    // us (device clock)
};
#pragma pack(pop)

static_assert(sizeof(ImuPacket) == kImuLen, "ImuPacket must be 56 bytes packed");
static_assert(sizeof(AhrsPacket) == kAhrsLen, "AhrsPacket must be 48 bytes packed");

// CRC-8 (vendor table) over `len` bytes — used on the 4-byte header prefix.
uint8_t crc8(const uint8_t * p, uint8_t len);
// CRC-16/CCITT (vendor table) over `len` bytes — used on the payload.
uint16_t crc16(const uint8_t * p, uint8_t len);

}  // namespace qmini_imu::fdilink

#endif  // QMINI_IMU__FDILINK_PROTOCOL_HPP_
