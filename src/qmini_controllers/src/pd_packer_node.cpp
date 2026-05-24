// pd_packer_node — the thin PD command packer (M2.1).
//
// The GO-M8010-6 closes its PD loop in firmware, so this node is NOT a software
// PD loop. It:
//   1. takes an absolute joint-position target on /joint_target (or, until a
//      policy publishes one, holds the home/standing pose),
//   2. ramps smoothly from the robot's current pose to that target when motion
//      is first enabled (ease-out, like hightorque_sim2real's init transition),
//   3. clamps every q_des to per-joint position limits (hard safety bound),
//   4. attaches joint-side kp/kd from gains.yaml,
//   5. publishes qmini_msgs/MotorCommand on /motor_command at policy rate.
//
// It emits commands ONLY while qmini_safety reports MotionGate::STATE_ENABLED.
// In every other state it stays silent; qmini_hardware owns the hold (soft
// stop) / zero-torque (hard stop) behavior. This node never applies torque on
// its own — and in M2.1 nothing subscribes to /motor_command yet, so it is
// fully testable by `ros2 topic echo /motor_command` with zero motor risk.
//
// All values (gains, home pose, action semantics) come from the authoritative
// Isaac Lab repo qmini_isaaclab; see config/gains.yaml and
// qmini_hardware/config/home_pose.yaml.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "qmini_msgs/msg/motion_gate.hpp"
#include "qmini_msgs/msg/motor_command.hpp"

#include <yaml-cpp/yaml.h>

namespace {
// Canonical joint order — MUST match qmini_hardware and the policy action order.
constexpr std::size_t kN = 10;
constexpr std::array<const char*, kN> kJointOrder = {
    "hip_yaw_l",  "hip_roll_l",  "hip_pitch_l",  "knee_pitch_l",  "ankle_pitch_l",
    "hip_yaw_r",  "hip_roll_r",  "hip_pitch_r",  "knee_pitch_r",  "ankle_pitch_r"};

// Smoothstep ease (0..1): zero velocity at both ends -> gentle start and stop.
double smoothstep(double a) {
  a = std::clamp(a, 0.0, 1.0);
  return a * a * (3.0 - 2.0 * a);
}
}  // namespace

using qmini_msgs::msg::MotionGate;

class PdPackerNode : public rclcpp::Node {
 public:
  PdPackerNode() : rclcpp::Node("pd_packer_node") {
    for (std::size_t i = 0; i < kN; ++i) name_to_idx_[kJointOrder[i]] = i;

    declare_parameter<std::string>("gains_file", "");
    declare_parameter<std::string>("home_pose_file", "");
    declare_parameter<double>("publish_rate_hz", 50.0);   // Isaac Lab policy rate
    declare_parameter<double>("ramp_duration_s", 3.0);    // ease-out to first target
    declare_parameter<double>("target_timeout_s", 0.5);   // stale target -> hold home

    load_home_pose(get_parameter("home_pose_file").as_string());
    load_gains_and_limits(get_parameter("gains_file").as_string());
    validate_home_within_limits();

    ramp_duration_s_ = get_parameter("ramp_duration_s").as_double();
    target_timeout_s_ = get_parameter("target_timeout_s").as_double();

    // /joint_states is best-effort from the driver; accept best-effort + reliable.
    auto sensor_qos = rclcpp::SensorDataQoS();
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", sensor_qos,
        std::bind(&PdPackerNode::on_joint_states, this, std::placeholders::_1));
    tgt_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_target", sensor_qos,
        std::bind(&PdPackerNode::on_joint_target, this, std::placeholders::_1));

    // MotionGate is latched RELIABLE + TRANSIENT_LOCAL — match it so we get the
    // current state immediately on startup.
    auto gate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    gate_sub_ = create_subscription<MotionGate>(
        "safety/motion_gate", gate_qos,
        std::bind(&PdPackerNode::on_gate, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<qmini_msgs::msg::MotorCommand>(
        "motor_command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const double rate = std::max(1.0, get_parameter("publish_rate_hz").as_double());
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate),
        std::bind(&PdPackerNode::tick, this));

    // Fixed-size (float64[10]) message arrays — set the constant fields once.
    // dq_des/tau_ff stay zero (firmware PD, no feed-forward); kp/kd are constant.
    cmd_.dq_des.fill(0.0);
    cmd_.tau_ff.fill(0.0);
    cmd_.kp = kp_;
    cmd_.kd = kd_;

    RCLCPP_INFO(get_logger(),
                "pd_packer up: %.0f Hz, ramp %.1fs. Emits MotorCommand only when "
                "MotionGate=ENABLED; target defaults to home pose.",
                rate, ramp_duration_s_);
  }

 private:
  // ---- config loading -------------------------------------------------------
  void load_home_pose(const std::string& path) {
    if (path.empty()) {
      RCLCPP_FATAL(get_logger(), "home_pose_file parameter is empty.");
      throw std::runtime_error("home_pose_file empty");
    }
    const auto root = YAML::LoadFile(path);
    const auto hp = root["home_pose"];
    for (std::size_t i = 0; i < kN; ++i) {
      if (!hp[kJointOrder[i]]) {
        RCLCPP_FATAL(get_logger(), "home_pose missing joint %s in %s",
                     kJointOrder[i], path.c_str());
        throw std::runtime_error("home_pose incomplete");
      }
      home_[i] = hp[kJointOrder[i]].as<double>();
    }
  }

  void load_gains_and_limits(const std::string& path) {
    if (path.empty()) {
      RCLCPP_FATAL(get_logger(), "gains_file parameter is empty.");
      throw std::runtime_error("gains_file empty");
    }
    const auto root = YAML::LoadFile(path);
    const auto kp = root["gains"]["kp"];
    const auto kd = root["gains"]["kd"];
    const auto lim = root["position_limits"];
    for (std::size_t i = 0; i < kN; ++i) {
      const char* j = kJointOrder[i];
      if (!kp[j] || !kd[j] || !lim[j] || lim[j].size() != 2) {
        RCLCPP_FATAL(get_logger(), "gains/limits missing or malformed for %s in %s",
                     j, path.c_str());
        throw std::runtime_error("gains/limits incomplete");
      }
      kp_[i] = kp[j].as<double>();
      kd_[i] = kd[j].as<double>();
      pos_min_[i] = lim[j][0].as<double>();
      pos_max_[i] = lim[j][1].as<double>();
    }
  }

  void validate_home_within_limits() {
    for (std::size_t i = 0; i < kN; ++i) {
      if (home_[i] < pos_min_[i] || home_[i] > pos_max_[i]) {
        RCLCPP_WARN(get_logger(),
                    "home pose for %s (%.3f) is OUTSIDE its position limit "
                    "[%.3f, %.3f] — it will be clamped.",
                    kJointOrder[i], home_[i], pos_min_[i], pos_max_[i]);
      }
    }
  }

  // ---- callbacks ------------------------------------------------------------
  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (std::size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      auto it = name_to_idx_.find(msg->name[k]);
      if (it != name_to_idx_.end()) cur_pos_[it->second] = msg->position[k];
    }
    cur_pos_valid_ = true;
  }

  void on_joint_target(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (std::size_t k = 0; k < msg->name.size() && k < msg->position.size(); ++k) {
      auto it = name_to_idx_.find(msg->name[k]);
      if (it != name_to_idx_.end()) target_[it->second] = msg->position[k];
    }
    last_target_time_ = now();
    target_seen_ = true;
  }

  void on_gate(const MotionGate::SharedPtr msg) {
    const bool was_enabled = (gate_state_ == MotionGate::STATE_ENABLED);
    gate_state_ = msg->state;
    const bool is_enabled = (gate_state_ == MotionGate::STATE_ENABLED);
    if (is_enabled && !was_enabled) {
      // Entering ENABLED: ramp from wherever the robot is now to the target, so
      // the first command doesn't yank the joints. If we have no joint_states
      // yet, ramp from the home pose (best available guess).
      for (std::size_t i = 0; i < kN; ++i) {
        ramp_start_[i] = cur_pos_valid_ ? cur_pos_[i] : home_[i];
      }
      ramp_t0_ = now();
      ramping_ = true;
      RCLCPP_INFO(get_logger(), "MotionGate ENABLED — ramping to target over %.1fs.",
                  ramp_duration_s_);
    }
  }

  // ---- control tick ---------------------------------------------------------
  void tick() {
    if (gate_state_ != MotionGate::STATE_ENABLED) return;  // silent unless enabled

    // Desired absolute target: fresh /joint_target, else hold home pose.
    const bool target_fresh =
        target_seen_ &&
        (now() - last_target_time_).seconds() <= target_timeout_s_;
    std::array<double, kN> desired = target_fresh ? target_ : home_;

    std::array<double, kN> q;
    if (ramping_) {
      const double a = (now() - ramp_t0_).seconds() / std::max(1e-3, ramp_duration_s_);
      const double s = smoothstep(a);
      for (std::size_t i = 0; i < kN; ++i)
        q[i] = ramp_start_[i] + s * (desired[i] - ramp_start_[i]);
      if (a >= 1.0) ramping_ = false;
    } else {
      q = desired;
    }

    for (std::size_t i = 0; i < kN; ++i)
      cmd_.q_des[i] = std::clamp(q[i], pos_min_[i], pos_max_[i]);

    cmd_.header.stamp = now();
    cmd_pub_->publish(cmd_);
  }

  // ---- state ----------------------------------------------------------------
  std::unordered_map<std::string, std::size_t> name_to_idx_;
  std::array<double, kN> home_{}, kp_{}, kd_{}, pos_min_{}, pos_max_{};
  std::array<double, kN> cur_pos_{}, target_{}, ramp_start_{};
  bool cur_pos_valid_{false};
  bool target_seen_{false};
  rclcpp::Time last_target_time_{0, 0, RCL_ROS_TIME};

  uint8_t gate_state_{MotionGate::STATE_BOOTING};
  bool ramping_{false};
  rclcpp::Time ramp_t0_{0, 0, RCL_ROS_TIME};
  double ramp_duration_s_{3.0};
  double target_timeout_s_{0.5};

  qmini_msgs::msg::MotorCommand cmd_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_, tgt_sub_;
  rclcpp::Subscription<MotionGate>::SharedPtr gate_sub_;
  rclcpp::Publisher<qmini_msgs::msg::MotorCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PdPackerNode>());
  rclcpp::shutdown();
  return 0;
}
