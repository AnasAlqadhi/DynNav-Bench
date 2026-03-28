# DynNav-Bench

**A Dynamic-Obstacle Navigation Benchmark for Deep Reinforcement Learning**

DynNav-Bench is a Gazebo-based simulation benchmark for training and evaluating DRL agents on mapless robot navigation through environments with moving obstacles. It features a 10×10 m arena with up to 15 dynamic obstacles, a 5-phase difficulty curriculum, and a velocity-aware 54-dimensional observation space.

<p align="center">
  <a href="https://youtu.be/REPLACE_WITH_VIDEO_ID">
    <img src="docs/arena_overview.png" alt="DynNav-Bench Arena — Click to watch demo" width="700">
  </a>
  <br>
  <em>Click the image to watch the demo video</em>
</p>

---

## Key Features

- **15 dynamic obstacles** with sinusoidal motion (8–20 s periods, 0.18 m/s peak velocity)
- **5 difficulty phases** — progressive difficulty from close-range goals to full-arena navigation with all obstacles active
- **54-dim velocity-aware observation** — LiDAR ranges + temporal scan velocity + goal vector + robot velocity
- **4 Danger Zone (DZ) preprocessing modes** for ablation studies
- **Deterministic obstacle motion** — fully reproducible evaluation across seeds
- **Bring your own agent** — standard ROS2 topic interface; plug in any RL algorithm
- **TurtleBot3 Waffle Pi** — standard research platform, sim-to-real transferable

---

## Difficulty Phases

| Phase | Goals | Distance | Active Obstacles | Description |
|-------|-------|----------|-----------------|-------------|
| 1 | 12 | 1.0–2.0 m | 6 perimeter | Easy — close targets, minimal obstacle interaction |
| 2 | 16 | 2.5–3.5 m | 6 perimeter | Near — medium range, must navigate around perimeter |
| 3 | 48 | 1.0–5.0 m | 6 perimeter | All — full arena coverage, obstacles still peripheral |
| 4 | 36 | 2.5–5.0 m | 12 (+6 centre) | Hard — centre corridor obstacles activate |
| 5 | 36 | 2.5–5.0 m | 15 (all) | Final — all obstacles active, maximum difficulty |

Select the phase with `DYNNAV_PHASE=1..5` environment variable.

---

## Observation Space (54 dims)

| Component | Dims | Range | Description |
|-----------|------|-------|-------------|
| `scan` | 24 | [0, 1] | Normalised LiDAR (min-pooled from 360 rays) |
| `scan_velocity` | 24 | [-1, 1] | Frame-to-frame scan difference (detects moving obstacles) |
| `goal_dist` | 1 | [0, 1] | Normalised distance to goal (d/8.0) |
| `goal_angle` | 1 | [-1, 1] | Normalised heading error (angle/π) |
| `own_lin_vel` | 1 | — | Actual linear velocity (v/0.26) |
| `own_ang_vel` | 1 | — | Actual angular velocity (ω/1.82) |
| `min_scan` | 1 | [0, 1] | Closest obstacle reading |
| `min_approach` | 1 | [-1, 1] | Fastest approaching obstacle velocity |

## Action Space

| Action | Range | Description |
|--------|-------|-------------|
| `linear_vel` | [0.0, 0.26] m/s | Forward velocity |
| `angular_vel` | [-1.82, 1.82] rad/s | Turning velocity |

---

## Installation

### Prerequisites

- **Ubuntu 22.04** (or compatible)
- **ROS2 Humble**
- **Gazebo Classic 11**
- **TurtleBot3 packages**

### 1. Install ROS2 Humble + Gazebo (if not already installed)

```bash
# Follow: https://docs.ros.org/en/humble/Installation.html
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs
```

### 2. Install TurtleBot3 packages

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-description
```

### 3. Clone and build DynNav-Bench

```bash
mkdir -p ~/dynnav_ws/src
cd ~/dynnav_ws/src
git clone https://github.com/AnasAlqadhi/DynNav-Bench.git

cd ~/dynnav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dynnav_bench
source install/setup.bash
```

### 4. Set TurtleBot3 model

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```

---

## Quick Start

### Terminal 1 — Launch the simulation

```bash
source ~/dynnav_ws/install/setup.bash

# Phase 1 (easy) — headless
DYNNAV_PHASE=1 ros2 launch dynnav_bench dynnav_bench_launch.py

# Phase 3 (all goals) — with GUI
DYNNAV_PHASE=3 ros2 launch dynnav_bench dynnav_bench_launch.py gui:=true

# Phase 5 (hardest — all 15 obstacles)
DYNNAV_PHASE=5 ros2 launch dynnav_bench dynnav_bench_launch.py
```

### Terminal 2 — Start environment nodes

```bash
source ~/dynnav_ws/install/setup.bash
export DYNNAV_PHASE=1

ros2 run dynnav_bench environment &
ros2 run dynnav_bench goal_manager &
ros2 run dynnav_bench obstacle_controller &
```

### Terminal 3 — Connect your RL agent

Your agent communicates via these ROS2 topics:

```python
# Subscribe to get observations
#   /dynnav/reset_obs    → Float32MultiArray (54 floats — initial obs)
#   /dynnav/step_result  → Float32MultiArray (54 + reward + done + info)

# Publish actions
#   /dynnav/action       → Float32MultiArray [linear_vel, angular_vel]
```

**Episode info codes** (last float in step_result):
- `0` — step (not terminal)
- `1` — goal reached
- `2` — collision
- `4` — timeout
- `8` — stuck

### Danger Zone modes

Control observation preprocessing with the `DZ_MODE` env var:

```bash
DZ_MODE=baseline     ros2 run dynnav_bench environment   # raw observations
DZ_MODE=static_dz    ros2 run dynnav_bench environment   # amplify close readings
DZ_MODE=velocity_dz  ros2 run dynnav_bench environment   # amplify close + approaching (default)
DZ_MODE=stam         ros2 run dynnav_bench environment   # raw (for learned attention)
```

---

## ROS2 Topic Reference

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/dynnav/reset_obs` | Float32MultiArray | env → agent | Observation after reset (54 floats) |
| `/dynnav/step_result` | Float32MultiArray | env → agent | obs + reward + done + info (57 floats) |
| `/dynnav/action` | Float32MultiArray | agent → env | Continuous action [lin_vel, ang_vel] |
| `/dynnav/goal` | String | goal_mgr → env | Goal position "x.xxxx,y.yyyy" |
| `/dynnav/need_goal` | Bool | env → goal_mgr | Request next goal |
| `/dynnav/phase` | Int32 | goal_mgr → obs_ctrl | Current phase (1-5, latched) |
| `/cmd_vel` | Twist | env → robot | Robot velocity commands |
| `/scan` | LaserScan | robot → env | 360° LiDAR scan |
| `/odom` | Odometry | robot → env | Robot odometry |

---

## Configuration

All parameters are in `config/dynnav_bench.yaml`. Key settings:

```yaml
phase: 1                # Difficulty phase (1-5)
dz_mode: velocity_dz    # Observation preprocessing mode
goal_radius: 0.45       # Success threshold (metres)
collision_dist: 0.20    # Collision threshold (metres)
max_steps: 800          # Episode timeout
obstacle_v_peak: 0.18   # Obstacle speed (m/s)
```

Environment variables override config values:
- `DYNNAV_PHASE` — phase (1-5)
- `DZ_MODE` — danger zone mode

---

## Citation

If you use DynNav-Bench in your research, please cite:

```bibtex
@software{dynnav_bench,
  title   = {DynNav-Bench: A Dynamic-Obstacle Navigation Benchmark for Deep Reinforcement Learning},
  author  = {Anas Alqadhi},
  year    = {2026},
  url     = {https://github.com/AnasAlqadhi/DynNav-Bench},
  license = {MIT}
}
```

## License

MIT — see [LICENSE](LICENSE) for details.
