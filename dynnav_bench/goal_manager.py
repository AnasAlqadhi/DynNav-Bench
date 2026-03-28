#!/usr/bin/env python3
"""
DynNav-Bench — Goal Manager
=============================
Manages goal positions across 5 difficulty phases.

Phase selection: set env var DYNNAV_PHASE (1-5) or pass ROS param start_phase.

Phase structure:
  Phase 1 — EASY    12 goals at 1.0–2.0 m,  obstacles: 6 perimeter
  Phase 2 — NEAR    16 goals at 2.5–3.5 m,  obstacles: 6 perimeter
  Phase 3 — ALL     48 goals at 1.0–5.0 m,  obstacles: 6 perimeter
  Phase 4 — HARD    36 goals at 2.5–5.0 m,  obstacles: 12 (+ centre)
  Phase 5 — HARD    36 goals at 2.5–5.0 m,  obstacles: 15 (all active)

Goal positions are carefully placed to avoid dynamic obstacle spawn/sweep zones.
A coloured marker (phase-dependent) is spawned in Gazebo at the goal position.

ROS2 Interface:
  Publishes:
    /dynnav/goal     String  — "x.xxxx,y.yyyy"
    /dynnav/phase    Int32   — current phase number (latched)
  Subscribes:
    /dynnav/need_goal   Bool             — request next goal
    /dynnav/step_result Float32MultiArray — episode result tracking
    /dynnav/reset_obs   Float32MultiArray — reset signal for marker refresh
"""
import math
import os
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool, Float32MultiArray, Int32
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState as SvcSetState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose

RESET_OBS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST, depth=1)

OBS_DIM = 54

# =============================================================================
#  GOAL POOLS
# =============================================================================

# Phase 1 — EASY: close cardinal/diagonal goals
GOALS_EASY = [
    ( 1.0,  0.0), (-1.0,  0.0),    # E / W  1.0m
    ( 0.0,  1.0), ( 0.0, -1.0),    # N / S  1.0m
    ( 1.2,  1.2), (-1.2,  1.2),    # NE/NW  1.7m
    ( 1.2, -1.2), (-1.2, -1.2),    # SE/SW  1.7m
    ( 0.0,  2.0), ( 0.0, -2.0),    # far N/S  2.0m
    ( 1.5,  1.3), (-1.5,  1.3),    # far diag 2.0m
]

# Phase 2 — NEAR: medium distance (positioned to avoid obstacle spawn zones)
GOALS_NEAR = [
    ( 2.5,  1.5), (-2.5,  1.5),    # E/W   2.9m
    ( 1.5,  2.0), (-1.5, -2.0),    # offset N/S 2.5m
    ( 1.8,  1.8), (-1.8, -1.8),    # NE/SW 2.5m
    ( 1.8, -1.8), ( 2.5, -1.5),    # SE 2.5m + S offset 2.9m
    ( 2.1,  2.1), (-2.1, -2.1),    # diag 3.0m
    ( 1.5,  2.5), ( 0.0, -3.5),    # N offset 2.9m, S 3.5m
    ( 2.8,  1.8), (-2.8,  1.8),    # upper lane  3.3m
    ( 2.8, -1.8), (-2.8, -1.8),    # lower lane  3.3m
]

# Phase 3 — FAR: long corridors + corners (positioned to avoid obstacle sweeps)
GOALS_FAR = [
    (-1.5,  4.0), ( 1.5,  4.0),               # N corridor 4.3m
    (-1.5, -4.0), ( 0.0, -4.0), ( 1.5, -4.0), # S corridor
    (-4.2,  1.5), ( 4.2,  1.5),               # E/W corridors 4.5m
    (-2.2,  3.5), ( 2.2,  3.5),               # upper corners 4.1m
    (-2.2, -3.5), ( 2.2, -3.5),               # lower corners 4.1m
    (-3.5,  1.8), ( 3.5,  1.8),               # side upper 3.9m
    (-3.5, -1.8), ( 3.5, -1.8),               # side lower 3.9m
]

GOALS_ALL  = GOALS_EASY + GOALS_NEAR + GOALS_FAR
GOALS_HARD = GOALS_NEAR + GOALS_FAR

PHASE_POOLS = {
    1: GOALS_EASY,
    2: GOALS_NEAR,
    3: GOALS_ALL,
    4: GOALS_HARD,
    5: GOALS_HARD,
}

# =============================================================================
#  GOAL MARKER SDF (phase-coloured)
# =============================================================================
PHASE_COLORS = {
    1: {"ambient": "0 0.9 0 1",   "diffuse": "0 1 0 1",     "emissive": "0 0.8 0 1"},      # green
    2: {"ambient": "0.9 0.9 0 1", "diffuse": "1 1 0 1",     "emissive": "0.8 0.8 0 1"},    # yellow
    3: {"ambient": "1 0.5 0 1",   "diffuse": "1 0.55 0 1",  "emissive": "0.9 0.4 0 1"},    # orange
    4: {"ambient": "0.9 0 0 1",   "diffuse": "1 0.1 0.1 1", "emissive": "0.8 0 0 1"},      # red
    5: {"ambient": "0.6 0 0.9 1", "diffuse": "0.7 0 1 1",   "emissive": "0.5 0 0.8 1"},    # purple
}

PHASE_NAMES = {
    1: "EASY  (1-2 m,     6 perimeter obstacles)",
    2: "NEAR  (2.5-3.5 m, 6 perimeter obstacles)",
    3: "ALL   (1-5 m,     6 perimeter obstacles)",
    4: "HARD  (2.5-5 m,   12 obstacles active)",
    5: "HARD  (2.5-5 m,   ALL 15 obstacles active)",
}


def _make_goal_sdf(phase: int = 3) -> str:
    """Goal marker: vertical pole + glowing beacon sphere + ground disc + outer ring."""
    c = PHASE_COLORS.get(phase, PHASE_COLORS[3])
    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="goal_pole">
    <static>true</static>
    <link name="link">
      <!-- Ground disc — solid coloured circle on the floor -->
      <visual name="ground_disc">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><cylinder><radius>0.42</radius><length>0.01</length></cylinder></geometry>
        <material>
          <ambient>{c['ambient']}</ambient>
          <diffuse>{c['diffuse']}</diffuse>
          <emissive>{c['emissive']}</emissive>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>
      <!-- Outer ring — white border around the disc -->
      <visual name="outer_ring">
        <pose>0 0 0.006 0 0 0</pose>
        <geometry><cylinder><radius>0.45</radius><length>0.005</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>1 1 1 0.8</diffuse>
          <emissive>0.4 0.4 0.4 1</emissive>
        </material>
      </visual>
      <!-- Vertical pole — thin metallic rod rising from the disc -->
      <visual name="pole">
        <pose>0 0 0.35 0 0 0</pose>
        <geometry><cylinder><radius>0.02</radius><length>0.70</length></cylinder></geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.8 0.8 0.8 1</diffuse>
          <specular>0.9 0.9 0.9 1</specular>
        </material>
      </visual>
      <!-- Beacon sphere — glowing ball on top of the pole -->
      <visual name="beacon">
        <pose>0 0 0.75 0 0 0</pose>
        <geometry><sphere><radius>0.08</radius></sphere></geometry>
        <material>
          <ambient>{c['ambient']}</ambient>
          <diffuse>{c['diffuse']}</diffuse>
          <emissive>{c['emissive']}</emissive>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      <!-- Inner glow — slightly larger transparent sphere for bloom effect -->
      <visual name="glow">
        <pose>0 0 0.75 0 0 0</pose>
        <geometry><sphere><radius>0.12</radius></sphere></geometry>
        <material>
          <ambient>{c['ambient']}</ambient>
          <diffuse>{c['diffuse']}</diffuse>
          <emissive>{c['emissive']}</emissive>
          <specular>0 0 0 1</specular>
        </material>
        <transparency>0.6</transparency>
      </visual>
    </link>
  </model>
</sdf>"""


class GoalManager(Node):
    def __init__(self):
        super().__init__("dynnav_goal_manager")

        # Phase selection: env var > ROS param > default 1
        self.declare_parameter("start_phase", int(os.environ.get("DYNNAV_PHASE", "1")))
        self._phase = max(1, min(5, int(self.get_parameter("start_phase").value)))
        self._pool = list(PHASE_POOLS[self._phase])
        self._prev_idx = -1
        self._total_eps = 0
        self._total_succs = 0
        self._ep_open = False

        # Gazebo marker state
        self._marker_spawned = False
        self._current_goal = None
        self._confirmed_marker_pos = None
        self._pending_marker = None
        self._marker_busy = False
        self._spawn_cli  = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")
        self._set_cli    = self.create_client(SvcSetState, "/set_entity_state")

        # ROS2 publishers
        self._goal_pub = self.create_publisher(String, "/dynnav/goal", 10)
        _latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._phase_pub = self.create_publisher(Int32, "/dynnav/phase", _latched)

        # Subscribers
        self.create_subscription(Bool, "/dynnav/need_goal", self._on_need_goal, 10)
        self.create_subscription(Float32MultiArray, "/dynnav/step_result", self._on_step_result, 10)
        self.create_subscription(Float32MultiArray, "/dynnav/reset_obs", self._on_reset_obs, RESET_OBS_QOS)

        # Timers
        self._last_goal_msg = None
        self.create_timer(1.5, self._republish)
        self.create_timer(0.3, self._enforce_marker)
        self._startup_done = False
        self.create_timer(0.8, self._startup_goal)
        self.create_timer(0.5, self._publish_phase_once)

        self._print_phase_banner()

    def _publish_phase_once(self):
        msg = Int32(); msg.data = self._phase
        self._phase_pub.publish(msg)

    def _print_phase_banner(self):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  DynNav-Bench  Phase {self._phase} — {PHASE_NAMES[self._phase]}")
        self.get_logger().info(f"  {len(self._pool)} goal positions active")
        self.get_logger().info("=" * 60)

    # ── episode tracking ──────────────────────────────────────────────────────
    def _on_step_result(self, msg: Float32MultiArray):
        if len(msg.data) < OBS_DIM + 3:
            return
        done = bool(msg.data[OBS_DIM + 1])
        info = int(msg.data[OBS_DIM + 2])
        success = (info == 1)
        if done and self._ep_open:
            self._ep_open = False
            self._total_eps += 1
            self._total_succs += int(success)
            if self._total_eps % 10 == 0:
                sr = self._total_succs / self._total_eps if self._total_eps else 0
                self.get_logger().info(
                    f"  [Phase {self._phase}] ep={self._total_eps}"
                    f"  SR={sr*100:.0f}%"
                    f"  ({self._total_succs}/{self._total_eps})")

    # ── goal publishing ───────────────────────────────────────────────────────
    def _on_need_goal(self, msg: Bool):
        if msg.data:
            self._ep_open = True
            self._publish_next_goal()

    def _publish_next_goal(self):
        pool = self._pool
        cands = [i for i in range(len(pool)) if i != self._prev_idx]
        idx = random.choice(cands)
        self._prev_idx = idx
        gx, gy = pool[idx]

        out = String()
        out.data = f"{gx:.4f},{gy:.4f}"
        self._goal_pub.publish(out)
        self._last_goal_msg = out

        dist = math.hypot(gx, gy)
        self.get_logger().info(
            f"[Goal Ph{self._phase}] ({gx:+.2f},{gy:+.2f}) {dist:.1f}m")

        self._pending_marker = (gx, gy)
        self._place_goal_marker(gx, gy)

    def _startup_goal(self):
        if not self._startup_done:
            self._startup_done = True
            self._ep_open = True
            self._publish_next_goal()

    def _on_reset_obs(self, msg: Float32MultiArray):
        self._marker_busy = False
        self._confirmed_marker_pos = None
        if self._last_goal_msg is not None:
            try:
                gx, gy = [float(v) for v in self._last_goal_msg.data.split(",")]
                self._pending_marker = (gx, gy)
                if self._marker_spawned and self._set_cli.service_is_ready():
                    self._move_marker(gx, gy)
                elif not self._marker_spawned and self._spawn_cli.service_is_ready():
                    self._spawn_marker(gx, gy)
            except Exception:
                pass

    def _republish(self):
        if self._last_goal_msg is not None:
            self._goal_pub.publish(self._last_goal_msg)

    # ── Gazebo marker ─────────────────────────────────────────────────────────
    def _enforce_marker(self):
        if self._current_goal is None or self._marker_busy:
            return
        x, y = self._current_goal
        if not self._marker_spawned:
            if self._spawn_cli.service_is_ready():
                self._spawn_marker(x, y)
        else:
            if self._confirmed_marker_pos == (x, y):
                return
            if self._set_cli.service_is_ready():
                self._move_marker(x, y)

    def _place_goal_marker(self, x: float, y: float):
        self._current_goal = (x, y)
        if not self._marker_spawned:
            if self._spawn_cli.service_is_ready():
                self._spawn_marker(x, y)
        else:
            if self._set_cli.service_is_ready() and not self._marker_busy:
                self._move_marker(x, y)

    def _spawn_marker(self, x: float, y: float):
        req = SpawnEntity.Request()
        req.name = "goal_pole"
        req.xml = _make_goal_sdf(self._phase)
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.0
        req.reference_frame = "world"
        fut = self._spawn_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_spawn_done(f))

    def _on_spawn_done(self, future):
        try:
            res = future.result()
            already = (not res.success and "already exists" in res.status_message)
            if res.success or already:
                self._marker_spawned = True
                if self._pending_marker is not None:
                    x, y = self._pending_marker
                    self._move_marker(x, y)
                    self._pending_marker = None
                elif self._current_goal is not None:
                    self._move_marker(*self._current_goal)
        except Exception as e:
            self.get_logger().warn(f"Marker spawn exception: {e}")

    def _move_marker(self, x: float, y: float):
        self._marker_busy = True
        req = SvcSetState.Request()
        req.state = EntityState()
        req.state.name = "goal_pole"
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = 0.0
        p.orientation.w = 1.0
        req.state.pose = p
        fut = self._set_cli.call_async(req)
        fut.add_done_callback(lambda f: self._on_move_done(f))

    def _on_move_done(self, future):
        self._marker_busy = False
        try:
            future.result()
            self._confirmed_marker_pos = self._current_goal
        except Exception:
            self._confirmed_marker_pos = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
