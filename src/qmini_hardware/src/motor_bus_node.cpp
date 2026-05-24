// Copyright 2026 Tony Wang
// SPDX-License-Identifier: BSD-3-Clause
//
// qmini_hardware / motor_bus_node — M1 (read-only "torque-disabled" mode).
//
// Layout: ONE node, ONE polling thread per RS-485 channel. Each thread owns
// its own SerialPort (SerialPort is not thread-safe). A wall timer in the
// main thread snapshots each channel's most-recent MotorData and publishes
// a single aggregated sensor_msgs/JointState at the configured rate.
//
// In M1 the node sends all-zeros commands (mode=FOC, kp=kd=q=dq=tau=0): the
// motor freewheels but still reports its encoder, so a human pushing the
// joint by hand will see /joint_states update. No MotorCommand handling
// yet — that arrives in M2 once the safety/PD path is wired.
//
// Gear ratio + zero offset: the SDK reports motor-side q / dq / tau. The URDF
// and the Isaac Lab cfg work in joint-side radians. The GO-M8010-6 has only a
// rotor-side encoder, so the joint zero is unknown at power-on and must be
// calibrated (homing). We convert at the message boundary, PER JOINT:
//   joint_q   = motor_q  / ratio[i] - offset[i]
//   joint_dq  = motor_dq / ratio[i]
//   joint_tau = motor_tau * ratio[i]      (torque amplifies through a reducer)
// where ratio[i] is the per-joint reduction (hip-roll has a 2nd gear stage:
// 6.33*3 = 18.99; all other joints 6.33) and offset[i] is the homing offset
// captured at the all-joints-zero jig pose (offset = motor_q / ratio there).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "qmini_msgs/msg/safety_heartbeat.hpp"
#include "qmini_msgs/msg/motion_gate.hpp"

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include <yaml-cpp/yaml.h>

namespace {

// Canonical joint ordering — matches the URDF and the policy's action vector.
// Do not reorder without retraining the policy.
constexpr std::array<const char*, 10> kJointOrder = {
    "hip_yaw_l",   "hip_roll_l",  "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
    "hip_yaw_r",   "hip_roll_r",  "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
};

struct MotorSlot {
  std::string joint;
  int id;
  double ratio{6.33};              // per-joint reduction (hip-roll = 18.99)
  std::atomic<double> offset{0.0}; // homing offset [rad, joint-side]
  MotorData latest;                // protected by mutex
  std::mutex mu;
  std::atomic<bool> has_fresh{false};
};

struct Channel {
  std::string name;
  std::string port;
  int baud_rate;
  std::unique_ptr<SerialPort> serial;
  std::vector<std::unique_ptr<MotorSlot>> motors;
  std::atomic<bool> port_ok{false};
  std::atomic<bool> running{false};
  std::thread poll_thread;
};

}  // namespace

class MotorBusNode : public rclcpp::Node {
 public:
  MotorBusNode() : Node("motor_bus_node") {
    declare_parameter<std::string>("layout_file", "");
    declare_parameter<double>("min_poll_period_us", 500.0);   // ~2 kHz max
    declare_parameter<bool>("allow_missing_channels", true);
    // A motor that fails this many consecutive polls is marked ABSENT and no
    // longer polled (silences the SDK timeout spam and removes its per-poll
    // stall). It is re-probed every motor_reprobe_period_s so reconnecting it
    // recovers automatically — this is what makes partial bring-up (Phase B/C)
    // and mid-run motor loss survivable.
    declare_parameter<int>("motor_absent_threshold", 3);
    declare_parameter<double>("motor_reprobe_period_s", 2.0);
    // File holding the per-joint homing offsets (joint name -> offset rad).
    // Loaded at startup if present; (re)written by the capture_home_offsets
    // service. Empty = default share-path location.
    declare_parameter<std::string>("offsets_file", "");

    gear_ratio_ = queryGearRatio(MotorType::GO_M8010_6);
    if (gear_ratio_ <= 0.0f) {
      RCLCPP_FATAL(get_logger(), "queryGearRatio returned %.3f — refusing to start.",
                   gear_ratio_);
      throw std::runtime_error("Bad gear ratio from Unitree SDK.");
    }
    RCLCPP_INFO(get_logger(), "GO-M8010-6 base gear ratio = %.4f (per-joint "
                "ratios may differ; hip-roll has a 2nd stage = %.4f).",
                gear_ratio_, gear_ratio_ * 3.0);

    const std::string layout_file = resolve_layout_path();
    load_layout(layout_file);   // sets per-motor ratio (default = gear_ratio_)
    load_offsets();             // applies homing offsets if a file exists

    build_joint_index();
    open_channels();
    start_polling_threads();

    rclcpp::QoS hb_qos(1);  hb_qos.reliable();
    rclcpp::QoS gate_qos(1); gate_qos.reliable().transient_local();

    hb_sub_ = create_subscription<qmini_msgs::msg::SafetyHeartbeat>(
        "safety/heartbeat", hb_qos,
        [this](const qmini_msgs::msg::SafetyHeartbeat::SharedPtr) {
          last_hb_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
        });
    gate_sub_ = create_subscription<qmini_msgs::msg::MotionGate>(
        "safety/motion_gate", gate_qos,
        [this](const qmini_msgs::msg::MotionGate::SharedPtr m) {
          motion_enabled_.store(
              m->state == qmini_msgs::msg::MotionGate::STATE_ENABLED,
              std::memory_order_relaxed);
          RCLCPP_INFO(get_logger(),
                      "MotionGate -> state=%u (enabled=%s) reason=\"%s\"",
                      m->state, motion_enabled_ ? "yes" : "no",
                      m->reason.c_str());
        });

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SystemDefaultsQoS());

    // Homing service: hold the robot at the all-joints-zero jig pose, then call
    // this. It sets each present motor's offset = motor_q / ratio and persists
    // the result. No torque is involved — purely a read+record operation.
    capture_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/capture_home_offsets",
        std::bind(&MotorBusNode::on_capture_offsets, this,
                  std::placeholders::_1, std::placeholders::_2));

    const double pub_rate_hz = publish_rate_hz_;
    const auto period_ns = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / std::max(1.0, pub_rate_hz)));
    publish_timer_ = create_wall_timer(
        period_ns, std::bind(&MotorBusNode::publish_joint_states, this));

    RCLCPP_INFO(get_logger(),
                "motor_bus_node up: %zu channels, %zu motors, /joint_states @ %.1f Hz.",
                channels_.size(), motor_count(), pub_rate_hz);
  }

  ~MotorBusNode() override {
    for (auto& ch : channels_) {
      ch.running.store(false, std::memory_order_relaxed);
      if (ch.poll_thread.joinable()) ch.poll_thread.join();
    }
  }

 private:
  std::string resolve_layout_path() {
    auto p = get_parameter("layout_file").as_string();
    if (!p.empty()) return p;
    // Default: installed share path
    return ament_index_cpp_share_dir() + "/config/motor_layout.yaml";
  }

  // ament_index_cpp would be the proper dependency, but we'd rather not pull
  // it in for one path lookup. The launch file passes layout_file explicitly,
  // so this fallback is only for "ros2 run" sanity.
  std::string ament_index_cpp_share_dir() {
    const char* prefix = std::getenv("AMENT_PREFIX_PATH");
    if (!prefix) return "src/qmini_hardware";  // dev fallback
    // AMENT_PREFIX_PATH is colon-separated; the first entry is this overlay.
    std::string p(prefix);
    const auto colon = p.find(':');
    if (colon != std::string::npos) p = p.substr(0, colon);
    return p + "/share/qmini_hardware";
  }

  void load_layout(const std::string& path) {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
      RCLCPP_FATAL(get_logger(), "Failed to load motor_layout.yaml at %s: %s",
                   path.c_str(), e.what());
      throw;
    }
    publish_rate_hz_ = root["joint_state_publish_rate_hz"].as<double>(200.0);
    const int default_baud = root["default_baud_rate"].as<int>(4000000);

    const auto& chans = root["channels"];
    if (!chans || !chans.IsSequence() || chans.size() != 4) {
      RCLCPP_FATAL(get_logger(),
                   "Expected exactly 4 channels in motor_layout.yaml, got %zu.",
                   chans ? chans.size() : 0);
      throw std::runtime_error("motor_layout.yaml malformed.");
    }
    // Channel holds std::atomic + std::thread (non-movable, non-copyable), so
    // it can't live in a std::vector (reserve/realloc need a move). std::deque
    // never relocates existing elements, so emplace_back works without one.
    for (const auto& cn : chans) {
      Channel& ch = channels_.emplace_back();
      ch.name = cn["name"].as<std::string>();
      ch.port = cn["port"].as<std::string>();
      ch.baud_rate = cn["baud_rate"].as<int>(default_baud);
      for (const auto& mn : cn["motors"]) {
        auto slot = std::make_unique<MotorSlot>();
        slot->joint = mn["joint"].as<std::string>();
        slot->id = mn["id"].as<int>();
        // Per-joint reduction. Defaults to the SDK base ratio (6.33); hip-roll
        // joints carry a 2nd gear stage and set gear_ratio: 18.99 in the YAML.
        slot->ratio = mn["gear_ratio"].as<double>(gear_ratio_);
        if (slot->ratio <= 0.0) {
          RCLCPP_FATAL(get_logger(), "Bad gear_ratio %.3f for joint %s.",
                       slot->ratio, slot->joint.c_str());
          throw std::runtime_error("Bad gear_ratio in motor_layout.yaml.");
        }
        ch.motors.push_back(std::move(slot));
      }
    }
  }

  // Resolve the offsets-file path: explicit param, else the share-dir default.
  std::string resolve_offsets_path() {
    auto p = get_parameter("offsets_file").as_string();
    if (!p.empty()) return p;
    return ament_index_cpp_share_dir() + "/config/joint_offsets.yaml";
  }

  // Load homing offsets (joint name -> rad) if the file exists. Missing file
  // or missing joints leave offsets at 0.0 (uncalibrated — the node still runs
  // so the bus is testable, but absolute joint angles are not yet meaningful).
  void load_offsets() {
    const std::string path = resolve_offsets_path();
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const std::exception&) {
      RCLCPP_WARN(get_logger(),
                  "No homing offsets at %s — joint angles are UNCALIBRATED. "
                  "Hold the all-joints-zero jig pose and call "
                  "~/capture_home_offsets.", path.c_str());
      return;
    }
    const auto& offs = root["offsets"];
    size_t applied = 0;
    for (auto& ch : channels_) {
      for (auto& m : ch.motors) {
        if (offs && offs[m->joint]) {
          m->offset.store(offs[m->joint].as<double>(), std::memory_order_relaxed);
          ++applied;
        }
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu/10 homing offsets from %s.",
                applied, path.c_str());
  }

  void build_joint_index() {
    joint_to_chan_motor_.clear();
    for (size_t ci = 0; ci < channels_.size(); ++ci) {
      for (size_t mi = 0; mi < channels_[ci].motors.size(); ++mi) {
        const auto& j = channels_[ci].motors[mi]->joint;
        if (joint_to_chan_motor_.count(j)) {
          RCLCPP_FATAL(get_logger(),
                       "Duplicate joint name in motor_layout.yaml: %s", j.c_str());
          throw std::runtime_error("duplicate joint name");
        }
        joint_to_chan_motor_[j] = {ci, mi};
      }
    }
    for (const auto* name : kJointOrder) {
      if (!joint_to_chan_motor_.count(name)) {
        RCLCPP_FATAL(get_logger(),
                     "URDF-canonical joint missing from motor_layout.yaml: %s", name);
        throw std::runtime_error("missing joint in layout");
      }
    }
  }

  void open_channels() {
    const bool tolerate_missing = get_parameter("allow_missing_channels").as_bool();
    for (auto& ch : channels_) {
      try {
        ch.serial = std::make_unique<SerialPort>(
            ch.port, /*recvLength=*/16,
            static_cast<uint32_t>(ch.baud_rate));
        ch.port_ok = true;
        RCLCPP_INFO(get_logger(), "[%s] opened %s @ %d baud (%zu motors).",
                    ch.name.c_str(), ch.port.c_str(), ch.baud_rate,
                    ch.motors.size());
      } catch (const std::exception& e) {
        if (!tolerate_missing) {
          RCLCPP_FATAL(get_logger(), "[%s] open failed: %s. Aborting.",
                       ch.name.c_str(), e.what());
          throw;
        }
        RCLCPP_WARN(get_logger(),
                    "[%s] could not open %s: %s — channel marked DOWN, "
                    "node will keep running so the rest of the bus is testable.",
                    ch.name.c_str(), ch.port.c_str(), e.what());
        ch.port_ok = false;
      }
    }
  }

  void start_polling_threads() {
    const double min_period_us = get_parameter("min_poll_period_us").as_double();
    const int absent_threshold = static_cast<int>(
        std::max<int64_t>(1, get_parameter("motor_absent_threshold").as_int()));
    const double reprobe_s =
        std::max(0.1, get_parameter("motor_reprobe_period_s").as_double());
    for (auto& ch : channels_) {
      if (!ch.port_ok) continue;
      ch.running = true;
      ch.poll_thread = std::thread(
          &MotorBusNode::poll_loop, this, std::ref(ch), min_period_us,
          absent_threshold, reprobe_s);
    }
  }

  // Per-channel polling. All M1 commands are zero — motor freewheels but
  // reports state. Future milestones swap in real commands gated by the
  // motion_enabled_ flag.
  void poll_loop(Channel& ch, double min_period_us, int absent_threshold,
                 double reprobe_period_s) {
    const size_t n = ch.motors.size();
    std::vector<MotorCmd> cmds(n);
    std::vector<MotorData> datas(n);
    const auto mode_foc = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
    for (size_t i = 0; i < n; ++i) {
      cmds[i].motorType = MotorType::GO_M8010_6;
      datas[i].motorType = MotorType::GO_M8010_6;
      cmds[i].id = static_cast<unsigned short>(ch.motors[i]->id);
      cmds[i].mode = static_cast<unsigned short>(mode_foc);
      cmds[i].q = 0.0f;
      cmds[i].dq = 0.0f;
      cmds[i].tau = 0.0f;
      cmds[i].kp = 0.0f;
      cmds[i].kd = 0.0f;
    }

    // Per-motor presence tracking (thread-local to this channel's poll thread).
    std::vector<int> fail_streak(n, 0);
    std::vector<bool> absent(n, false);
    const auto reprobe_period = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(reprobe_period_s));
    std::vector<std::chrono::steady_clock::time_point> next_probe(
        n, std::chrono::steady_clock::now());

    const auto min_period = std::chrono::microseconds(
        static_cast<int>(std::max(50.0, min_period_us)));

    while (rclcpp::ok() && ch.running.load(std::memory_order_relaxed)) {
      const auto t0 = std::chrono::steady_clock::now();

      // Poll each motor independently so one silent motor neither blanks the
      // rest of the channel nor stalls the loop on every cycle.
      for (size_t i = 0; i < n; ++i) {
        auto& slot = *ch.motors[i];
        if (absent[i] && t0 < next_probe[i]) {
          continue;  // marked absent, not yet time to re-probe
        }

        const bool ok = ch.serial->sendRecv(&cmds[i], &datas[i]) &&
                        datas[i].correct;
        if (ok) {
          {
            std::lock_guard<std::mutex> lk(slot.mu);
            slot.latest = datas[i];
          }
          slot.has_fresh.store(true, std::memory_order_relaxed);
          if (absent[i]) {
            RCLCPP_INFO(get_logger(), "[%s] motor id=%d reconnected.",
                        ch.name.c_str(), ch.motors[i]->id);
          }
          absent[i] = false;
          fail_streak[i] = 0;
        } else {
          slot.has_fresh.store(false, std::memory_order_relaxed);
          if (!absent[i] && ++fail_streak[i] >= absent_threshold) {
            absent[i] = true;
            RCLCPP_WARN(get_logger(),
                        "[%s] motor id=%d silent for %d polls — marking ABSENT "
                        "(re-probing every %.1fs). Connect+power it to recover.",
                        ch.name.c_str(), ch.motors[i]->id, absent_threshold,
                        reprobe_period_s);
          }
          next_probe[i] = t0 + reprobe_period;  // throttle re-probe of a silent motor
        }
      }

      const auto elapsed = std::chrono::steady_clock::now() - t0;
      if (elapsed < min_period) std::this_thread::sleep_for(min_period - elapsed);
    }
  }

  void publish_joint_states() {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.header.frame_id = "";  // joint state has no frame

    msg.name.reserve(kJointOrder.size());
    msg.position.reserve(kJointOrder.size());
    msg.velocity.reserve(kJointOrder.size());
    msg.effort.reserve(kJointOrder.size());

    for (const auto* joint_name : kJointOrder) {
      msg.name.emplace_back(joint_name);
      auto [ci, mi] = joint_to_chan_motor_.at(joint_name);
      auto& slot = *channels_[ci].motors[mi];

      MotorData snapshot;
      bool fresh;
      {
        std::lock_guard<std::mutex> lk(slot.mu);
        snapshot = slot.latest;
        fresh = slot.has_fresh.load(std::memory_order_relaxed);
      }

      if (!fresh || !snapshot.correct || !channels_[ci].port_ok) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        msg.position.push_back(nan);
        msg.velocity.push_back(nan);
        msg.effort.push_back(nan);
        continue;
      }
      const double ratio = slot.ratio;
      const double offset = slot.offset.load(std::memory_order_relaxed);
      msg.position.push_back(snapshot.q / ratio - offset);
      msg.velocity.push_back(snapshot.dq / ratio);
      msg.effort.push_back(snapshot.tau * ratio);
    }
    joint_state_pub_->publish(msg);
  }

  // Homing: capture offsets at the all-joints-zero jig pose. offset = q/ratio
  // so that joint_q reads ~0 in that pose. Only present (fresh+correct) motors
  // are captured; the rest keep their previous offset and are reported back.
  void on_capture_offsets(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
    std::vector<std::string> captured, missing;
    for (auto& ch : channels_) {
      for (auto& m : ch.motors) {
        MotorData snap;
        bool fresh;
        {
          std::lock_guard<std::mutex> lk(m->mu);
          snap = m->latest;
          fresh = m->has_fresh.load(std::memory_order_relaxed);
        }
        if (fresh && snap.correct) {
          m->offset.store(snap.q / m->ratio, std::memory_order_relaxed);
          captured.push_back(m->joint);
        } else {
          missing.push_back(m->joint);
        }
      }
    }
    const std::string path = resolve_offsets_path();
    const bool wrote = save_offsets(path);

    std::string msg = "Captured " + std::to_string(captured.size()) + "/10";
    if (!missing.empty()) {
      msg += "; SKIPPED (not responding):";
      for (auto& j : missing) msg += " " + j;
    }
    msg += wrote ? ("; saved to " + path) : ("; FAILED to write " + path);
    resp->success = missing.empty() && wrote;
    resp->message = msg;
    RCLCPP_INFO(get_logger(), "capture_home_offsets: %s", msg.c_str());
  }

  bool save_offsets(const std::string& path) {
    std::ofstream f(path, std::ios::trunc);
    if (!f) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s for writing.", path.c_str());
      return false;
    }
    f << "# Qmini joint zero offsets [rad, joint-side], captured at the\n"
         "# all-joints-zero jig pose via ~/capture_home_offsets.\n"
         "#   joint_q = motor_q / gear_ratio - offset\n"
         "offsets:\n"
      << std::fixed << std::setprecision(6);
    for (const auto* j : kJointOrder) {
      double off = 0.0;
      auto it = joint_to_chan_motor_.find(j);
      if (it != joint_to_chan_motor_.end()) {
        auto [ci, mi] = it->second;
        off = channels_[ci].motors[mi]->offset.load(std::memory_order_relaxed);
      }
      f << "  " << j << ": " << off << "\n";
    }
    return static_cast<bool>(f);
  }

  size_t motor_count() const {
    size_t n = 0;
    for (const auto& ch : channels_) n += ch.motors.size();
    return n;
  }

  // --- state ---
  std::deque<Channel> channels_;
  std::unordered_map<std::string, std::pair<size_t, size_t>> joint_to_chan_motor_;
  double publish_rate_hz_{200.0};
  float gear_ratio_{0.0f};

  std::atomic<int64_t> last_hb_ns_{0};
  std::atomic<bool> motion_enabled_{false};

  rclcpp::Subscription<qmini_msgs::msg::SafetyHeartbeat>::SharedPtr hb_sub_;
  rclcpp::Subscription<qmini_msgs::msg::MotionGate>::SharedPtr gate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotorBusNode>());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "[motor_bus_node] fatal: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
