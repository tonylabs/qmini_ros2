// Copyright 2026 Tony Wang
// SPDX-License-Identifier: BSD-3-Clause
//
// qmini_safety node.
//
// Responsibilities:
//   - Publish SafetyHeartbeat at 50 Hz (RELIABLE) so qmini_hardware can run.
//   - Subscribe to /joy and run the deadman + watchdog state machine.
//   - Publish a latched MotionGate (RELIABLE + TRANSIENT_LOCAL) on change.
//
// M2 state machine (deliberately simple; full two-tier latch + L2+R2 release
// gesture and battery monitoring are M6):
//
//   no /joy ever seen .......................... BOOTING       (torque off)
//   /joy stale (> joy_timeout_s) ............... HARD_STOPPED  (torque off, latched)
//   /joy fresh, deadman held ................... ENABLED       (motion permitted)
//   /joy fresh, deadman released ............... SOFT_STOPPED  (hold position)
//
// The HARD_STOPPED latch (set on /joy loss, e.g. a Bluetooth dropout) clears
// only once /joy is fresh again AND the deadman is released — so a momentary
// link blip can never silently re-energize the robot while the buttons are
// still held.
//
// Deadman = ALL buttons in `deadman_buttons` held simultaneously. Default
// [4, 5] = LB + RB on an Xbox-layout pad (a two-handed gesture).
//
// IMPORTANT: this is the SOLE publisher of SafetyHeartbeat and MotionGate.
// qmini_hardware and qmini_controllers trust only this node.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "qmini_msgs/msg/safety_heartbeat.hpp"
#include "qmini_msgs/msg/motion_gate.hpp"

using namespace std::chrono_literals;
using qmini_msgs::msg::MotionGate;

class SafetyNode : public rclcpp::Node {
 public:
  SafetyNode() : Node("qmini_safety") {
    // --- parameters ---
    declare_parameter<std::vector<int64_t>>("deadman_buttons", {4, 5});
    declare_parameter<double>("joy_timeout_s", 0.1);   // 100 ms watchdog
    declare_parameter<double>("eval_rate_hz", 50.0);

    deadman_buttons_ = get_parameter("deadman_buttons").as_integer_array();
    joy_timeout_ns_ = static_cast<int64_t>(
        get_parameter("joy_timeout_s").as_double() * 1e9);
    const double eval_hz = get_parameter("eval_rate_hz").as_double();

    // --- QoS ---
    // Heartbeat: RELIABLE depth 1, NOT transient_local (consumer wants the
    // latest, not a stale buffered one at subscribe time).
    auto hb_qos = rclcpp::QoS(1).reliable();
    // Gate: RELIABLE + TRANSIENT_LOCAL so late subscribers immediately learn
    // the current safety state — the canonical latched-state pattern.
    auto gate_qos = rclcpp::QoS(1).reliable().transient_local();
    // /joy: sensor data, BEST_EFFORT depth 1 (joy_node default). We do our own
    // staleness watchdog rather than relying on delivery guarantees.
    auto joy_qos = rclcpp::SensorDataQoS();

    heartbeat_pub_ = create_publisher<qmini_msgs::msg::SafetyHeartbeat>(
        "safety/heartbeat", hb_qos);
    gate_pub_ = create_publisher<MotionGate>("safety/motion_gate", gate_qos);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", joy_qos,
        std::bind(&SafetyNode::on_joy, this, std::placeholders::_1));

    // Publish the initial latched gate: BOOTING (no /joy seen yet → torque off).
    state_ = MotionGate::STATE_BOOTING;
    publish_gate(state_, "Booting: waiting for first /joy message.");

    heartbeat_timer_ = create_wall_timer(
        20ms, std::bind(&SafetyNode::publish_heartbeat, this));
    eval_timer_ = create_wall_timer(
        std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, eval_hz))),
        std::bind(&SafetyNode::evaluate, this));

    std::string btns;
    for (auto b : deadman_buttons_) btns += std::to_string(b) + " ";
    RCLCPP_INFO(get_logger(),
                "qmini_safety started — heartbeat 50 Hz, /joy watchdog %.0f ms, "
                "deadman buttons [ %s] (all required).",
                joy_timeout_ns_ / 1e6, btns.c_str());
  }

 private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg) {
    last_joy_ns_.store(now().nanoseconds(), std::memory_order_relaxed);

    // Deadman = every configured button index present and pressed.
    bool held = !deadman_buttons_.empty();
    for (int64_t idx : deadman_buttons_) {
      if (idx < 0 || static_cast<size_t>(idx) >= msg->buttons.size() ||
          msg->buttons[idx] == 0) {
        held = false;
        break;
      }
    }
    deadman_held_.store(held, std::memory_order_relaxed);
  }

  void evaluate() {
    const int64_t now_ns = now().nanoseconds();
    const int64_t last_joy = last_joy_ns_.load(std::memory_order_relaxed);
    const bool ever_seen_joy = last_joy != 0;
    const bool joy_fresh = ever_seen_joy && (now_ns - last_joy) <= joy_timeout_ns_;
    const bool deadman = deadman_held_.load(std::memory_order_relaxed);

    uint8_t next = state_;
    std::string reason;

    if (!ever_seen_joy) {
      next = MotionGate::STATE_BOOTING;
      reason = "Booting: waiting for first /joy message.";
    } else if (!joy_fresh) {
      next = MotionGate::STATE_HARD_STOPPED;
      hard_latched_ = true;
      reason = "HARD STOP: /joy stale (controller link lost).";
    } else if (hard_latched_) {
      if (!deadman) {
        hard_latched_ = false;
        next = MotionGate::STATE_SOFT_STOPPED;
        reason = "Link restored, deadman released — cleared to SOFT stop.";
      } else {
        next = MotionGate::STATE_HARD_STOPPED;
        reason = "HARD STOP latched: release deadman to clear.";
      }
    } else if (deadman) {
      next = MotionGate::STATE_ENABLED;
      reason = "Deadman held — motion ENABLED.";
    } else {
      next = MotionGate::STATE_SOFT_STOPPED;
      reason = "Deadman released — SOFT stop (holding position).";
    }

    if (next != state_) {
      state_ = next;
      publish_gate(state_, reason);
      RCLCPP_INFO(get_logger(), "MotionGate -> %s  (%s)",
                  state_name(state_), reason.c_str());
    }
  }

  static const char* state_name(uint8_t s) {
    switch (s) {
      case MotionGate::STATE_ENABLED: return "ENABLED";
      case MotionGate::STATE_SOFT_STOPPED: return "SOFT_STOPPED";
      case MotionGate::STATE_HARD_STOPPED: return "HARD_STOPPED";
      case MotionGate::STATE_BOOTING: return "BOOTING";
      default: return "?";
    }
  }

  void publish_heartbeat() {
    qmini_msgs::msg::SafetyHeartbeat msg;
    msg.header.stamp = now();
    msg.header.frame_id = "qmini_safety";
    msg.sequence = ++heartbeat_seq_;
    msg.origin_stamp = msg.header.stamp;
    heartbeat_pub_->publish(msg);
  }

  void publish_gate(uint8_t state, const std::string& reason) {
    MotionGate msg;
    msg.header.stamp = now();
    msg.header.frame_id = "qmini_safety";
    msg.state = state;
    msg.reason = reason;
    gate_pub_->publish(msg);
  }

  // params
  std::vector<int64_t> deadman_buttons_;
  int64_t joy_timeout_ns_{100000000};

  // state (evaluate() runs on a single timer thread; only it writes state_)
  uint8_t state_{MotionGate::STATE_BOOTING};
  bool hard_latched_{false};
  std::atomic<int64_t> last_joy_ns_{0};
  std::atomic<bool> deadman_held_{false};

  rclcpp::Publisher<qmini_msgs::msg::SafetyHeartbeat>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<MotionGate>::SharedPtr gate_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr eval_timer_;
  uint32_t heartbeat_seq_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}
