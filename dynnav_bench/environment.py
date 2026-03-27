#!/usr/bin/env python3
"""
DynNav-Bench — Environment Node
================================
Velocity-aware observation space for DRL navigation with dynamic obstacles.

Observation (54 dimensions):
  scan           (24) — normalised LiDAR ranges [0, 1] via min-pooling
  scan_velocity  (24) — frame difference (current - previous), clipped [-1, 1]
  goal_dist       (1) — normalised distance to goal (d/8.0, capped 1.0)
  goal_angle      (1) — normalised heading error (angle/pi)
  own_lin_vel     (1) — actual linear velocity from odom (v/0.26)
  own_ang_vel     (1) — actual angular velocity from odom (w/1.82)
  min_scan        (1) — closest obstacle reading
  min_approach    (1) — fastest approaching scan velocity

DZ_MODE (env var) controls observation preprocessing ONLY — reward is identical:
  baseline    — raw observations
  static_dz   — amplify close LiDAR readings
  velocity_dz — static_dz + amplify approaching obstacle velocities
  stam        — raw observations (for learned attention networks)

ROS2 Interface:
  Publishes:
    /dynnav/reset_obs       Float32MultiArray  — observation after reset
    /dynnav/step_result     Float32MultiArray  — obs + reward + done + info
    /dynnav/need_goal       Bool               — request next goal
    /cmd_vel                Twist              — robot velocity commands
  Subscribes:
    /scan                   LaserScan          — LiDAR
    /odom                   Odometry           — robot odometry
    /dynnav/goal            String             — goal position "x,y"
    /dynnav/action          Float32MultiArray  — continuous action [lin_vel, ang_vel]
"""
import math
import os
import time

import numpy as np
import rclpy
from collections import deque
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool, Float32MultiArray
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

RESET_OBS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

# ── Danger Zone observation parameters ────────────────────────────────────────
DZ_RADIUS_NORM = 0.20     # 0.70m / 3.5m — activation radius (normalised)
DZ_AMPLIFY     = 3.0      # amplification factor for close readings
DZ_VEL_AMPLIFY = 2.0      # extra amplification for approaching+close obstacles

# ── Reward constants ──────────────────────────────────────────────────────────
REWARD_COLLISION = -100.0
REWARD_GOAL      =  200.0
REWARD_STUCK     = -60.0
REWARD_TIMEOUT   = -80.0
STEP_PENALTY     = -0.15
PROGRESS_SCALE   = 6.0
HEADING_BONUS    = 0.25
PROXIMITY_DIST   = 0.50    # baseline proximity warning radius (metres)
PROXIMITY_SCALE  = 0.2
DZ_PROX_DIST     = 0.70    # DZ proximity warning radius (metres)
DZ_PROX_SCALE    = 0.5
GOAL_RADIUS      = 0.45    # metres — success threshold
COLLISION_DIST   = 0.20    # metres — collision threshold
MAX_STEPS        = 800
STUCK_WINDOW     = 150
STUCK_THRESH     = 0.05    # metres — movement threshold for stuck


class Environment(Node):
    """DynNav-Bench environment node.

    Manages observation computation, reward shaping, episode resets, and
    step execution. Connect your RL agent via the ROS2 topics listed above.
    """

    def __init__(self):
        super().__init__("dynnav_env")
        self._prev_lidar = None
        self._dz_mode = os.environ.get("DZ_MODE", "velocity_dz")
        self._dist_hist = deque(maxlen=STUCK_WINDOW)

        # Publishers
        self._cmd_pub  = self.create_publisher(Twist, "/cmd_vel", 10)
        self._obs_pub  = self.create_publisher(Float32MultiArray, "/dynnav/reset_obs", RESET_OBS_QOS)
        self._step_pub = self.create_publisher(Float32MultiArray, "/dynnav/step_result", 10)
        self._goal_pub = self.create_publisher(Bool, "/dynnav/need_goal", 10)

        # Subscribers
        self.create_subscription(LaserScan, "/scan", self._cb_scan, 10)
        self.create_subscription(Odometry, "/odom", self._cb_odom, 10)
        self.create_subscription(String, "/dynnav/goal", self._cb_goal, 10)
        self.create_subscription(Float32MultiArray, "/dynnav/action", self._cb_action, 10)

        # Service clients
        self._reset_cli   = self.create_client(Empty, "/reset_world")
        self._pause_cli   = self.create_client(Empty, "/pause_physics")
        self._unpause_cli = self.create_client(Empty, "/unpause_physics")

        # State
        self._scan = self._odom = self._goal_xy = None
        self._step_id = 0
        self._want_reset = True
        self._rs = "IDLE"
        self._prev_dist = None

        self.get_logger().info(f"[DynNav-Bench] Environment ready — DZ_MODE={self._dz_mode}")
        self.create_timer(0.02, self._loop)

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _cb_scan(self, msg):
        self._scan = msg

    def _cb_odom(self, msg):
        self._odom = msg

    def _cb_goal(self, msg):
        try:
            x, y = msg.data.split(",")
            x, y = float(x), float(y)
            if math.isfinite(x) and math.isfinite(y):
                self._goal_xy = (x, y)
        except Exception:
            pass

    def _cb_action(self, msg):
        if self._rs == "IDLE" and not self._want_reset:
            self._act_lin = float(msg.data[0])
            self._act_ang = float(msg.data[1])

    # ── observation (54 dims) ─────────────────────────────────────────────────
    def _get_obs(self):
        # 1. LiDAR scan -> 24 normalised values via min-pooling
        if not self._scan:
            lidar = [1.0] * 24
        else:
            raw = list(self._scan.ranges)
            step = max(1, len(raw) // 24)
            lidar = []
            for i in range(24):
                v = [x for x in raw[i * step:(i + 1) * step] if 0.01 < x < 3.5]
                lidar.append((min(v) if v else 3.5) / 3.5)

        # 2. Scan velocity = frame difference, clipped to [-1, 1]
        if self._prev_lidar is not None:
            scan_vel = [max(-1.0, min(1.0, c - p))
                        for c, p in zip(lidar, self._prev_lidar)]
        else:
            scan_vel = [0.0] * 24
        self._prev_lidar = lidar[:]

        # 3. DZ observation preprocessing (affects observation only, NOT reward)
        if self._dz_mode in ("static_dz", "velocity_dz"):
            for i in range(24):
                if lidar[i] < DZ_RADIUS_NORM:
                    lidar[i] = lidar[i] / DZ_AMPLIFY
        if self._dz_mode == "velocity_dz":
            for i in range(24):
                if lidar[i] < DZ_RADIUS_NORM and scan_vel[i] < 0:
                    scan_vel[i] = max(-1.0, scan_vel[i] * DZ_VEL_AMPLIFY)

        # 4. Navigation state
        dn, an, d = 1.0, 0.0, 999.0
        if self._odom and self._goal_xy:
            p = self._odom.pose.pose.position
            d = math.hypot(self._goal_xy[0] - p.x, self._goal_xy[1] - p.y)
            dn = min(1.0, d / 8.0)
            q = self._odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            an = math.atan2(self._goal_xy[1] - p.y,
                            self._goal_xy[0] - p.x) - yaw
            an = math.atan2(math.sin(an), math.cos(an)) / math.pi

        # 5. Summary signals
        min_scan = min(lidar)
        min_approach = min(scan_vel)

        # 6. Actual velocities from odometry (not commanded)
        if self._odom:
            actual_lin = self._odom.twist.twist.linear.x
            actual_ang = self._odom.twist.twist.angular.z
        else:
            actual_lin, actual_ang = 0.0, 0.0

        obs = (lidar                                                 # 24
               + scan_vel                                            # 24
               + [dn, an, actual_lin / 0.26, actual_ang / 1.82]     #  4
               + [min_scan, min_approach])                           #  2 -> 54

        raw_min_m = min(self._prev_lidar) * 3.5 if self._prev_lidar else 3.5
        return obs, raw_min_m, d, an

    # ── reward ────────────────────────────────────────────────────────────────
    def _compute_reward(self, min_dist_m, d, an):
        """Compute step reward. Same for ALL DZ modes — ablation is observation-only."""
        rew = STEP_PENALTY

        # Progress toward goal
        if self._prev_dist is not None and d < 900:
            delta = max(-0.3, min(0.3, self._prev_dist - d))
            rew += delta * PROGRESS_SCALE

        # Heading bonus — only when moving forward (prevents spin exploit)
        fwd = max(0.0, self._act_lin) / 0.26
        rew += HEADING_BONUS * (1.0 - abs(an)) * fwd

        # Anti-spin penalty
        if self._act_lin < 0.05:
            rew -= 0.3 * abs(self._act_ang) / 1.82

        # Proximity warning
        if self._dz_mode == "baseline":
            if min_dist_m < PROXIMITY_DIST:
                rew -= PROXIMITY_SCALE * (PROXIMITY_DIST - min_dist_m) / 0.30
        else:
            if min_dist_m < DZ_PROX_DIST:
                rew -= DZ_PROX_SCALE * (DZ_PROX_DIST - min_dist_m) / 0.50

        # Terminal conditions
        done, info = False, 0
        if min_dist_m < COLLISION_DIST:
            if self._step_id < 5:
                info = 0        # spawn artifact — retry
            else:
                rew = REWARD_COLLISION; info = 2; done = True
        elif d < GOAL_RADIUS:
            rew = REWARD_GOAL; info = 1; done = True
        elif (len(self._dist_hist) == STUCK_WINDOW
              and (max(self._dist_hist) - min(self._dist_hist)) < STUCK_THRESH):
            rew = REWARD_STUCK; info = 8; done = True
        elif self._step_id > MAX_STEPS:
            rew = REWARD_TIMEOUT; info = 4; done = True

        return rew, done, info

    # ── main loop ─────────────────────────────────────────────────────────────
    def _loop(self):
        # Reset sequence
        if self._want_reset and self._rs == "IDLE":
            if not self._reset_cli.service_is_ready():
                return
            self._rs = "WAIT"
            self._want_reset = False

            self._cmd_pub.publish(Twist())
            time.sleep(0.05)

            if self._pause_cli.service_is_ready():
                self._pause_cli.call_async(Empty.Request())
            time.sleep(0.1)

            self._scan = None
            self._prev_lidar = None
            self._reset_cli.call_async(Empty.Request())
            time.sleep(0.3)

            if self._unpause_cli.service_is_ready():
                self._unpause_cli.call_async(Empty.Request())
            time.sleep(1.5)

            self._dist_hist.clear()
            self._rs = "IDLE"
            obs, _, d, _ = self._get_obs()
            self._prev_dist = d
            msg = Float32MultiArray()
            msg.data = obs
            self._obs_pub.publish(msg)

        # Step execution
        if hasattr(self, '_act_lin') and self._act_lin is not None:
            tw = Twist()
            tw.linear.x = self._act_lin
            tw.angular.z = self._act_ang
            self._cmd_pub.publish(tw)
            time.sleep(0.15)
            self._step_id += 1

            obs, min_dist_m, d, an = self._get_obs()
            self._dist_hist.append(d)

            rew, done, info = self._compute_reward(min_dist_m, d, an)

            msg = Float32MultiArray()
            msg.data = obs + [rew, float(done), float(info)]
            self._step_pub.publish(msg)
            self._act_lin = None
            self._prev_dist = d

            if done:
                self._cmd_pub.publish(Twist())
                self._want_reset = True
                self._step_id = 0
                self._goal_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    rclpy.spin(Environment())


if __name__ == "__main__":
    main()
