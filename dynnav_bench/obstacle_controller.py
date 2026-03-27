#!/usr/bin/env python3
"""
DynNav-Bench — Obstacle Controller
====================================
Publishes sinusoidal velocity commands to dynamic obstacles in the arena.

Obstacles are activated progressively based on the current phase:
  Phase 1-3:  6 perimeter obstacles (corners + sides)
  Phase 4:   12 obstacles (perimeter + 6 centre long-travel)
  Phase 5:   15 obstacles (all — perimeter + centre + 3 inner short-travel)

Motion model:
  All obstacles follow 1D sinusoidal motion along a single axis (x or y).
  v = V_PEAK * sin(2*pi/period * t + phase_offset)
  V_PEAK = 0.18 m/s (70% of robot max 0.26 m/s — robot can always outrun).
  Period diversity (8-20s) and phase offsets create non-trivial timing interactions.

ROS2 Interface:
  Subscribes:  /dynnav/phase     Int32 — current phase (1-5)
  Publishes:   /{obstacle}/cmd_vel  Twist — velocity for each obstacle
"""
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# (name, axis, period_s, phase_offset_rad)
# amplitude = V_PEAK * period / (2*pi)

# 6 perimeter obstacles — always active
OBSTACLES_PERIMETER = [
    ("dyn_obs_1",  "x", 12.0, 0.00),            # NW  (-3.5, +3.5)  ±0.76m
    ("dyn_obs_2",  "x", 13.0, math.pi),          # NE  (+3.5, +3.5)  ±0.83m
    ("dyn_obs_3",  "y", 11.0, 0.00),             # SW  (-3.5, -3.5)  ±0.70m
    ("dyn_obs_4",  "y", 14.0, math.pi),          # SE  (+3.5, -3.5)  ±0.89m
    ("dyn_obs_5",  "y", 10.0, 0.00),             # W   (-2.5,  0.0)  ±0.64m
    ("dyn_obs_6",  "y", 10.0, math.pi),          # E   (+2.5,  0.0)  ±0.64m
]

# Phase 4: 6 centre long-travel pillars
OBSTACLES_CENTER_LONG = [
    ("s1",  "y", 20.0, 0.00),         # centre-left   (-1.5,  0.0)  ±0.57m
    ("s2",  "y", 20.0, math.pi),      # centre-right  (+1.5,  0.0)  opposite phase
    ("s6",  "y", 15.0, 0.00),         # outer-left    (-3.5,  0.0)  ±0.43m
    ("s7",  "y", 15.0, math.pi),      # outer-right   (+3.5,  0.0)  opposite phase
    ("s8",  "x", 15.0, 0.00),         # upper-centre  (-0.8, +1.8)  ±0.43m
    ("s10", "x", 18.0, math.pi),      # north-centre  ( 0.0, +3.5)  ±0.51m
]

# Phase 5: 3 additional short-travel inner pillars
OBSTACLES_CENTER_SHORT = [
    ("uc_clone",       "y",  8.0, 0.00),    # inner (-0.8, -1.6)  ±0.23m
    ("uc_clone_1",     "x",  9.0, math.pi), # inner (+0.6, +2.0)  ±0.26m
    ("uc_clone_clone", "y",  8.0, math.pi), # inner (+0.9, -1.4)  ±0.23m
]

OBSTACLES_CENTER = OBSTACLES_CENTER_LONG + OBSTACLES_CENTER_SHORT

V_PEAK = 0.18  # m/s — 70% of robot max


class ObstacleController(Node):
    def __init__(self):
        super().__init__("dynnav_obstacle_controller")

        # Phase from env var or default to 1
        self._phase = int(os.environ.get("DYNNAV_PHASE", "1"))

        all_obs = OBSTACLES_PERIMETER + OBSTACLES_CENTER
        self._pubs = {
            name: self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            for name, *_ in all_obs
        }
        self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.05, self._tick)

        # Subscribe to phase topic (latched)
        _latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Int32, "/dynnav/phase", self._on_phase, _latched)

        self.get_logger().info(
            f"[DynNav-Bench] Obstacle controller — Phase {self._phase} "
            f"(6 perimeter always, +6 centre at Ph4, +3 inner at Ph5) "
            f"V_peak={V_PEAK} m/s")

    def _on_phase(self, msg: Int32):
        new = msg.data
        if new == self._phase:
            return
        self._phase = new
        if new >= 5:
            self.get_logger().info(f"  Phase 5 — ALL 15 OBSTACLES ACTIVE")
        elif new >= 4:
            self.get_logger().info(f"  Phase 4 — 12 OBSTACLES ACTIVE (perimeter + centre)")
        else:
            self.get_logger().info(f"  Phase {new} — 6 perimeter obstacles only")

    def _tick(self):
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        if self._phase >= 5:
            active = OBSTACLES_PERIMETER + OBSTACLES_CENTER
        elif self._phase >= 4:
            active = OBSTACLES_PERIMETER + OBSTACLES_CENTER_LONG
        else:
            active = OBSTACLES_PERIMETER
        for name, axis, period, phase in active:
            v = V_PEAK * math.sin(2 * math.pi / period * t + phase)
            msg = Twist()
            if axis == "x":
                msg.linear.x = v
            else:
                msg.linear.y = v
            self._pubs[name].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
