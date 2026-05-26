// policy_runner_node — M5 ONNX policy runner for Qmini.
//
// Loads the Isaac-Lab-exported policy (obs[44] -> action[10]), reconstructs the
// EXACT training observation on-robot, runs inference at 50 Hz, and publishes
// absolute joint-position targets on /joint_target (sensor_msgs/JointState),
// which qmini_controllers/pd_packer_node turns into MotorCommands.
//
// Every constant here is mirrored from the authoritative Isaac Lab repo
// (qmini_isaaclab). When a number disagrees, change THIS file to match the sim,
// never the reverse — the policy was trained against those exact values.
//
//   Observation (44), in order (no normalization, no clipping, no framestack):
//     [ 0.. 2] imu_ang_vel            * 0.2
//     [ 3.. 5] imu_projected_gravity  * 1.0   (quat_rotate_inverse(q,(0,0,-1)))
//     [ 6..15] joint_pos_rel = q - home
//     [16..25] joint_vel_rel = dq          * 0.05   (default vel = 0)
//     [26..35] last_action  = previous RAW policy output
//     [36..38] velocity_commands [vx, vy, wz]
//     [39..42] gait_phase_sincos = [sinL, sinR, cosL, cosR]   (code order!)
//     [43]     static_flag = 1.0 if ||[vx,vy,wz]||2 < 0.15 else 0.0
//
//   Action: q_des[i] = home[i] + 0.5 * action[i]   (use_default_offset=True).
//   Gait phase: free-running 1.5 Hz, phiR = phiL + 0.5, advance +freq*dt/step.
//
// !!! JOINT ORDER CAVEAT !!!  kJointOrder below is the PhysX articulation DOF
// order INFERRED from the URDF tree (left chain then right chain). It is not
// stored in any Isaac Lab artifact. Confirm it ONCE on the sim side by printing
// env.scene["robot"].joint_names in play.py and freezing that list. A silent
// reorder here is the most likely way to break the policy.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "qmini_msgs/msg/motion_gate.hpp"

#include <yaml-cpp/yaml.h>
#include <onnxruntime_cxx_api.h>

namespace {

constexpr std::size_t kN = 10;        // joints
constexpr std::size_t kObs = 44;      // observation dims
constexpr std::size_t kAct = 10;      // action dims

// Canonical joint order — MUST match qmini_controllers/qmini_hardware AND the
// policy's articulation DOF order (see caveat in the file header).
constexpr std::array<const char*, kN> kJointOrder = {
    "hip_yaw_l",  "hip_roll_l",  "hip_pitch_l",  "knee_pitch_l",  "ankle_pitch_l",
    "hip_yaw_r",  "hip_roll_r",  "hip_pitch_r",  "knee_pitch_r",  "ankle_pitch_r"};

// quat_rotate_inverse(q, v): express world vector v in the body frame, matching
// Isaac Lab's isaaclab.utils.math.quat_rotate_inverse exactly.
// q = (w, x, y, z). Returns a - b + c with:
//   a = v (2 w^2 - 1),  b = 2 w (q_vec x v),  c = 2 (q_vec . v) q_vec
std::array<double, 3> quat_rotate_inverse(
    double w, double x, double y, double z, const std::array<double, 3>& v) {
  const std::array<double, 3> qv = {x, y, z};
  const double w2 = 2.0 * w * w - 1.0;
  const std::array<double, 3> a = {v[0] * w2, v[1] * w2, v[2] * w2};
  // cross(qv, v)
  const std::array<double, 3> cross = {
      qv[1] * v[2] - qv[2] * v[1],
      qv[2] * v[0] - qv[0] * v[2],
      qv[0] * v[1] - qv[1] * v[0]};
  const double dot = qv[0] * v[0] + qv[1] * v[1] + qv[2] * v[2];
  std::array<double, 3> out{};
  for (std::size_t i = 0; i < 3; ++i)
    out[i] = a[i] - 2.0 * w * cross[i] + 2.0 * dot * qv[i];
  return out;
}

}  // namespace

using qmini_msgs::msg::MotionGate;

class PolicyRunnerNode : public rclcpp::Node {
 public:
  PolicyRunnerNode()
      : rclcpp::Node("policy_runner_node"),
        ort_env_(ORT_LOGGING_LEVEL_WARNING, "qmini_rl") {
    for (std::size_t i = 0; i < kN; ++i) name_to_idx_[kJointOrder[i]] = i;

    // ---- parameters (defaults mirror Isaac Lab) ----
    const auto policy_path = declare_parameter<std::string>("policy_path", "");
    const auto home_pose_file = declare_parameter<std::string>("home_pose_file", "");
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    gait_frequency_ = declare_parameter<double>("gait_frequency_hz", 1.5);
    static_threshold_ = declare_parameter<double>("static_velocity_threshold", 0.15);
    action_scale_ = declare_parameter<double>("action_scale", 0.5);
    ang_vel_scale_ = declare_parameter<double>("ang_vel_scale", 0.2);
    joint_vel_scale_ = declare_parameter<double>("joint_vel_scale", 0.05);
    // command clamps = trained velocity_commands ranges
    vx_range_ = declare_parameter<std::vector<double>>("vx_range", {-0.4, 0.7});
    vy_range_ = declare_parameter<std::vector<double>>("vy_range", {-0.4, 0.4});
    wz_range_ = declare_parameter<std::vector<double>>("wz_range", {-1.0, 1.0});
    // when true, only publish targets while MotionGate == ENABLED
    gate_required_ = declare_parameter<bool>("gate_required", true);
    obs_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.2);

    load_home_pose(home_pose_file);
    load_policy(policy_path);

    auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", sensor_qos,
        std::bind(&PolicyRunnerNode::on_imu, this, std::placeholders::_1));
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", sensor_qos,
        std::bind(&PolicyRunnerNode::on_joint_states, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&PolicyRunnerNode::on_cmd_vel, this, std::placeholders::_1));

    auto gate_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    gate_sub_ = create_subscription<MotionGate>(
        "safety/motion_gate", gate_qos,
        std::bind(&PolicyRunnerNode::on_gate, this, std::placeholders::_1));

    tgt_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_target", sensor_qos);

    const double rate = std::max(1.0, rate_hz_);
    step_dt_ = 1.0 / rate;
    timer_ = create_wall_timer(std::chrono::duration<double>(step_dt_),
                               std::bind(&PolicyRunnerNode::tick, this));

    RCLCPP_INFO(get_logger(),
                "policy_runner up: %.0f Hz, gait %.2f Hz, action_scale %.2f, "
                "gate_required=%s. Publishes /joint_target.",
                rate, gait_frequency_, action_scale_,
                gate_required_ ? "true" : "false");
  }

 private:
  // ---- config / model loading ----
  void load_home_pose(const std::string& path) {
    if (path.empty()) {
      RCLCPP_FATAL(get_logger(), "home_pose_file parameter is empty.");
      throw std::runtime_error("home_pose_file empty");
    }
    const auto hp = YAML::LoadFile(path)["home_pose"];
    for (std::size_t i = 0; i < kN; ++i) {
      if (!hp[kJointOrder[i]]) {
        RCLCPP_FATAL(get_logger(), "home_pose missing joint %s", kJointOrder[i]);
        throw std::runtime_error("home_pose incomplete");
      }
      home_[i] = hp[kJointOrder[i]].as<double>();
    }
  }

  void load_policy(const std::string& path) {
    if (path.empty()) {
      RCLCPP_FATAL(get_logger(), "policy_path parameter is empty.");
      throw std::runtime_error("policy_path empty");
    }
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);   // tiny MLP; 1 thread avoids Pi 5 contention
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session_ = std::make_unique<Ort::Session>(ort_env_, path.c_str(), opts);

    Ort::AllocatorWithDefaultOptions alloc;
    input_name_ = session_->GetInputNameAllocated(0, alloc).get();
    output_name_ = session_->GetOutputNameAllocated(0, alloc).get();

    // sanity-check the I/O dims against what we assemble
    const auto in_shape =
        session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto out_shape =
        session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    const auto in_dim = in_shape.empty() ? -1 : in_shape.back();
    const auto out_dim = out_shape.empty() ? -1 : out_shape.back();
    if (in_dim != static_cast<int64_t>(kObs) || out_dim != static_cast<int64_t>(kAct)) {
      RCLCPP_FATAL(get_logger(),
                   "Policy I/O mismatch: model is [%ld]->[%ld], expected [%zu]->[%zu]. "
                   "Wrong ONNX export?", in_dim, out_dim, kObs, kAct);
      throw std::runtime_error("policy I/O dim mismatch");
    }
    RCLCPP_INFO(get_logger(), "Loaded policy '%s' (in='%s'[%zu], out='%s'[%zu]).",
                path.c_str(), input_name_.c_str(), kObs, output_name_.c_str(), kAct);
  }

  // ---- callbacks ----
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    ang_vel_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    quat_ = {msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
    last_imu_time_ = now();
    imu_valid_ = true;
  }

  void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (std::size_t k = 0; k < msg->name.size(); ++k) {
      auto it = name_to_idx_.find(msg->name[k]);
      if (it == name_to_idx_.end()) continue;
      if (k < msg->position.size()) cur_pos_[it->second] = msg->position[k];
      if (k < msg->velocity.size()) cur_vel_[it->second] = msg->velocity[k];
    }
    last_js_time_ = now();
    js_valid_ = true;
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
  }

  void on_gate(const MotionGate::SharedPtr msg) {
    const bool was_enabled = enabled_;
    enabled_ = (msg->state == MotionGate::STATE_ENABLED);
    if (enabled_ && !was_enabled) {
      // entering ENABLED: reset to a clean sim-reset-like state
      phase_ = 0.0;
      last_action_.fill(0.0f);
      RCLCPP_INFO(get_logger(), "MotionGate ENABLED — policy active.");
    }
  }

  // ---- 50 Hz control tick ----
  void tick() {
    if (gate_required_ && !enabled_) return;  // gated: pd_packer holds home

    const auto t = now();
    const bool inputs_fresh =
        imu_valid_ && js_valid_ &&
        (t - last_imu_time_).seconds() <= obs_timeout_s_ &&
        (t - last_js_time_).seconds() <= obs_timeout_s_;
    if (!inputs_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Stale/missing IMU or joint_states — not publishing target.");
      return;
    }

    // clamp commands to the trained ranges (this clamped vector feeds BOTH
    // velocity_commands and static_flag, exactly like the sim command vector)
    const double vx = std::clamp(cmd_vx_, vx_range_[0], vx_range_[1]);
    const double vy = std::clamp(cmd_vy_, vy_range_[0], vy_range_[1]);
    const double wz = std::clamp(cmd_wz_, wz_range_[0], wz_range_[1]);

    // ---- assemble observation (44) ----
    std::array<float, kObs> obs{};
    std::size_t o = 0;
    // imu_ang_vel * 0.2
    for (double a : ang_vel_) obs[o++] = static_cast<float>(a * ang_vel_scale_);
    // imu_projected_gravity (unit gravity (0,0,-1) into body frame)
    const auto g = quat_rotate_inverse(quat_[0], quat_[1], quat_[2], quat_[3],
                                       {0.0, 0.0, -1.0});
    for (double gi : g) obs[o++] = static_cast<float>(gi);
    // joint_pos_rel = q - home
    for (std::size_t i = 0; i < kN; ++i)
      obs[o++] = static_cast<float>(cur_pos_[i] - home_[i]);
    // joint_vel_rel = dq * 0.05
    for (std::size_t i = 0; i < kN; ++i)
      obs[o++] = static_cast<float>(cur_vel_[i] * joint_vel_scale_);
    // last_action (previous raw policy output)
    for (std::size_t i = 0; i < kN; ++i) obs[o++] = last_action_[i];
    // velocity_commands
    obs[o++] = static_cast<float>(vx);
    obs[o++] = static_cast<float>(vy);
    obs[o++] = static_cast<float>(wz);
    // gait_phase_sincos = [sinL, sinR, cosL, cosR]  (matches code, not docstring)
    const double phase_l = std::fmod(phase_, 1.0);
    const double phase_r = std::fmod(phase_l + 0.5, 1.0);
    const double two_pi = 2.0 * M_PI;
    obs[o++] = static_cast<float>(std::sin(two_pi * phase_l));
    obs[o++] = static_cast<float>(std::sin(two_pi * phase_r));
    obs[o++] = static_cast<float>(std::cos(two_pi * phase_l));
    obs[o++] = static_cast<float>(std::cos(two_pi * phase_r));
    // static_flag = 1.0 if ||[vx,vy,wz]||2 < threshold
    const double cmd_norm = std::sqrt(vx * vx + vy * vy + wz * wz);
    obs[o++] = (cmd_norm < static_threshold_) ? 1.0f : 0.0f;

    // ---- inference ----
    std::array<int64_t, 2> in_shape{1, static_cast<int64_t>(kObs)};
    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
        mem, obs.data(), obs.size(), in_shape.data(), in_shape.size());
    const char* in_names[] = {input_name_.c_str()};
    const char* out_names[] = {output_name_.c_str()};
    auto outputs = session_->Run(Ort::RunOptions{nullptr}, in_names, &in_tensor, 1,
                                 out_names, 1);
    const float* action = outputs[0].GetTensorData<float>();

    // ---- map action -> absolute joint target, store raw action ----
    sensor_msgs::msg::JointState tgt;
    tgt.header.stamp = t;
    tgt.name.resize(kN);
    tgt.position.resize(kN);
    for (std::size_t i = 0; i < kN; ++i) {
      last_action_[i] = action[i];
      tgt.name[i] = kJointOrder[i];
      tgt.position[i] = home_[i] + action_scale_ * static_cast<double>(action[i]);
    }
    tgt_pub_->publish(tgt);

    // advance the free-running gait clock (never freezes under static_flag)
    phase_ += gait_frequency_ * step_dt_;
    if (phase_ >= 1.0) phase_ -= std::floor(phase_);
  }

  // ---- members ----
  std::unordered_map<std::string, std::size_t> name_to_idx_;
  std::array<double, kN> home_{};
  std::array<double, kN> cur_pos_{}, cur_vel_{};
  std::array<float, kAct> last_action_{};

  std::array<double, 3> ang_vel_{};
  std::array<double, 4> quat_{1, 0, 0, 0};  // (w,x,y,z) identity until first IMU
  double cmd_vx_{0}, cmd_vy_{0}, cmd_wz_{0};

  double phase_{0.0};
  double rate_hz_{50.0}, step_dt_{0.02};
  double gait_frequency_{1.5}, static_threshold_{0.15}, action_scale_{0.5};
  double ang_vel_scale_{0.2}, joint_vel_scale_{0.05}, obs_timeout_s_{0.2};
  std::vector<double> vx_range_, vy_range_, wz_range_;
  bool gate_required_{true};

  bool imu_valid_{false}, js_valid_{false}, enabled_{false};
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_js_time_{0, 0, RCL_ROS_TIME};

  Ort::Env ort_env_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<MotionGate>::SharedPtr gate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tgt_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolicyRunnerNode>());
  rclcpp::shutdown();
  return 0;
}
