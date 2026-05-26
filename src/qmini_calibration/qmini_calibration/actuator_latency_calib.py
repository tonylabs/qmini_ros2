"""Actuator latency + effective-PD measurement (M4, step 4).

THE FIRST CALIBRATION THAT MOVES THE ROBOT UNDER TORQUE. Run it ON THE ROPE,
deadman held, low/nominal gains, one joint at a time. It publishes /motor_command
DIRECTLY (bypassing pd_packer) so the step has a known shape + known kp/kd; safety
is still enforced downstream by qmini_hardware (torque only while
enable_motor_torque AND MotionGate==ENABLED AND the heartbeat is fresh — release
the deadman and the motors freewheel).

Protocol: hold all joints at the home pose; for each tested joint, apply a small
square-wave position step (+step_rad) for `hold_s`, return, repeat `n_steps`
times, then move to the next joint. Emits /calibration/event markers at each
step edge so the analyzer can segment.

Measured offline (analyze_actuator_bag.py) from /motor_command + /joint_states
(both driver-stamped; /joint_states carries q, dq, and effort=measured torque):
  * actuator round-trip latency: command-edge -> first joint motion
  * effective kp: steady-state  tau / (q_des - q)   vs the commanded (Isaac Lab) kp

Compared against DelayedPDActuatorCfg(min_delay,max_delay) (currently 0 — almost
certainly too small) and the stiffness/damping dicts.

HARD RULES: refuses to start if the policy node is running; only advances the
protocol while MotionGate==ENABLED (deadman held) — release the deadman and it
holds home and pauses.
"""

import math
import os
from datetime import date

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from qmini_msgs.msg import MotorCommand, MotionGate

GUARD_NODES = ("policy_runner_node",)

# Canonical BY-LEG order — matches MotorCommand / qmini_hardware (NOT the policy
# interleaved order; this node talks to the motor bus, not the policy).
JOINTS = ["hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
          "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r"]
IDX = {n: i for i, n in enumerate(JOINTS)}


class ActuatorLatencyCalib(Node):
    def __init__(self):
        super().__init__("actuator_latency_calib")
        self.declare_parameter("home_pose_file", "")
        self.declare_parameter("gains_file", "")
        self.declare_parameter("test_joints", ["knee_pitch_l"])  # safe default: one joint
        self.declare_parameter("step_rad", 0.08)
        self.declare_parameter("hold_s", 1.0)
        self.declare_parameter("n_steps", 5)
        self.declare_parameter("settle_s", 2.0)
        self.declare_parameter("gain_scale", 1.0)   # reduce for a gentler first run
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("output_dir", "")

        self.test_joints = list(self.get_parameter("test_joints").value)
        self.step_rad = self.get_parameter("step_rad").value
        self.hold_s = self.get_parameter("hold_s").value
        self.n_steps = int(self.get_parameter("n_steps").value)
        self.settle_s = self.get_parameter("settle_s").value
        self.gain_scale = self.get_parameter("gain_scale").value
        self.dt = 1.0 / max(1.0, self.get_parameter("rate_hz").value)
        out_dir = self.get_parameter("output_dir").value or os.path.join(
            os.getcwd(), "src", "qmini_calibration", "data",
            f"{date.today().isoformat()}_actuator_latency")
        os.makedirs(out_dir, exist_ok=True)
        self.out_dir = out_dir

        clash = set(self.get_node_names()).intersection(GUARD_NODES)
        if clash:
            self.get_logger().fatal(f"Refusing to start: policy node running: {sorted(clash)}.")
            raise SystemExit(1)
        for j in self.test_joints:
            if j not in IDX:
                self.get_logger().fatal(f"Unknown test joint '{j}'.")
                raise SystemExit(1)

        self._load_config()

        self.event_pub = self.create_publisher(String, "calibration/event", 10)
        self.cmd_pub = self.create_publisher(
            MotorCommand, "motor_command", QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE))
        gate_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gate_sub = self.create_subscription(MotionGate, "safety/motion_gate", self.on_gate, gate_qos)

        self.enabled = False
        self.protocol_t = 0.0          # advances only while enabled
        self.cur_joint = 0
        self.last_phase = None
        self.done = False
        # per-joint block duration: settle, then n_steps * (up hold + down hold)
        self.block_s = self.settle_s + self.n_steps * 2.0 * self.hold_s

        self.timer = self.create_timer(self.dt, self.tick)
        self.get_logger().warn(
            "ACTUATOR LATENCY CALIB — robot WILL move under torque. Rope on, hold the "
            f"deadman. Joints: {self.test_joints}, step {self.step_rad:.3f} rad, "
            f"gain_scale {self.gain_scale:.2f}. Releasing the deadman holds home + pauses.")

    def _load_config(self):
        hp = self.get_parameter("home_pose_file").value
        gf = self.get_parameter("gains_file").value
        if not hp or not gf:
            self.get_logger().fatal("home_pose_file and gains_file parameters are required.")
            raise SystemExit(1)
        home_y = yaml.safe_load(open(hp))["home_pose"]
        g = yaml.safe_load(open(gf))
        self.home = [home_y[n] for n in JOINTS]
        self.kp = [g["gains"]["kp"][n] * self.gain_scale for n in JOINTS]
        self.kd = [g["gains"]["kd"][n] * self.gain_scale for n in JOINTS]
        self.lim = [tuple(g["position_limits"][n]) for n in JOINTS]

    def on_gate(self, msg: MotionGate):
        was = self.enabled
        self.enabled = (msg.state == MotionGate.STATE_ENABLED)
        if self.enabled and not was:
            self.get_logger().info("MotionGate ENABLED — protocol running.")
        elif not self.enabled and was:
            self.get_logger().warn("MotionGate disabled — holding home, protocol paused.")

    def _emit(self, label):
        self.event_pub.publish(String(data=label))
        self.get_logger().info(f"[event] {label}")

    def tick(self):
        if self.done:
            return
        # default: hold home, no step
        offset = 0.0
        tested = None
        if self.enabled and self.cur_joint < len(self.test_joints):
            tested = self.test_joints[self.cur_joint]
            tb = self.protocol_t  # time within current joint block
            phase_label = None
            if tb < self.settle_s:
                offset = 0.0
                phase_label = f"{tested}_settle"
            else:
                k = int((tb - self.settle_s) // self.hold_s)   # half-cycle index
                up = (k % 2 == 1)
                offset = self.step_rad if up else 0.0
                phase_label = f"{tested}_step{k // 2}_{'up' if up else 'down'}"
            if phase_label != self.last_phase:
                self._emit(phase_label)
                self.last_phase = phase_label
            self.protocol_t += self.dt
            if tb >= self.block_s:
                self._emit(f"{tested}_done")
                self.cur_joint += 1
                self.protocol_t = 0.0
                self.last_phase = None
                if self.cur_joint >= len(self.test_joints):
                    self._emit("protocol_done")
                    self.done = True
                    self.get_logger().info("Protocol complete. Ctrl-C to stop; analyze the bag.")

        # build the command: hold home everywhere, add the step on the tested joint
        cmd = MotorCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        for i in range(10):
            q = self.home[i]
            if tested is not None and i == IDX[tested]:
                q = min(self.lim[i][1], max(self.lim[i][0], self.home[i] + offset))
            cmd.q_des[i] = q
            cmd.dq_des[i] = 0.0
            cmd.tau_ff[i] = 0.0
            cmd.kp[i] = self.kp[i]
            cmd.kd[i] = self.kd[i]
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    try:
        node = ActuatorLatencyCalib()
    except SystemExit:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
