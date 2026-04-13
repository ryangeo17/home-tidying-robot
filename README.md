# home-tidying-robot

A ROS 2 + Gazebo Sim (Harmonic) demo of a mobile manipulator that autonomously
navigates a two-room home, picks up six coloured objects, and drops them into a
collection box. The package contains:

- A 4-wheel differential-drive base with a 3-DOF right arm and parallel-jaw
  gripper, described in xacro/URDF.
- A two-room SDF world with furniture, six pickup objects, and a collection
  box.
- `arm_init_node` — tucks the arm immediately after spawn so it does not clip
  the base.
- `nav_node` — a 16-state pick-and-place mission controller (pure-pursuit
  navigation, LiDAR obstacle avoidance, named arm poses, and per-object
  attach/detach via the `gz-sim-detachable-joint-system` plugin).
- A `ros_gz_bridge` configuration that bridges 21 topics between Gazebo and
  ROS 2.

---

## 1. Supported platforms

This package is tested on the following combinations:

| Ubuntu | ROS 2  | Gazebo    |
| ------ | ------ | --------- |
| 22.04  | Humble | Harmonic  |
| 24.04  | Jazzy  | Harmonic  |

> **Important:** The robot SDF uses the `gz-sim-*` plugin namespace, so
> **Gazebo Harmonic is required** — Gazebo Classic (11) and Gazebo Fortress
> (`ignition-gazebo-*`) will **not** work. On Ubuntu 24.04 / Jazzy, Harmonic is
> the default. On Ubuntu 22.04 / Humble, Harmonic is **not** the default and
> must be installed from the OSRF apt repository (instructions below).

---

## 2. Install dependencies

The reviewer is assumed to have a clean Ubuntu 22.04 or 24.04 install with
ROS 2 Humble or Jazzy already installed and sourced
(`source /opt/ros/$ROS_DISTRO/setup.bash`). Everything else needed is listed
below.

### 2a. Set the ROS distro variable

```bash
# Humble on Ubuntu 22.04
export ROS_DISTRO=humble

# OR Jazzy on Ubuntu 24.04
export ROS_DISTRO=jazzy
```

### 2b. (Ubuntu 22.04 / Humble only) Add the OSRF apt repository for Gazebo Harmonic

Skip this section entirely on Ubuntu 24.04 / Jazzy — Harmonic is already in
the default Ubuntu archive.

```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg

sudo curl https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
```

### 2c. Install system + ROS 2 packages

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  git \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  gz-harmonic \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-nav-msgs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-tf2-msgs \
  ros-${ROS_DISTRO}-rclpy \
  ros-${ROS_DISTRO}-launch \
  ros-${ROS_DISTRO}-launch-ros
```

Then install the `ros_gz` bridge that targets **Harmonic**:

```bash
# Ubuntu 22.04 / Humble  → use the explicitly Harmonic-paired metapackage
sudo apt install -y ros-humble-ros-gzharmonic

# Ubuntu 24.04 / Jazzy   → ros-gz already targets Harmonic
sudo apt install -y ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### 2d. Python dependencies

There are **no extra Python (pip) dependencies** beyond what ROS 2 already
provides — every Python import in this package is either part of the standard
library or comes from a `ros-${ROS_DISTRO}-*` apt package installed above.
A `requirements.txt` is included for completeness; it only pins `setuptools`
(used by `ament_python` at build time) and is safe to run:

```bash
pip3 install -r requirements.txt
```

---

## 3. Build

Create a colcon workspace and clone this repo into `src/`:

```bash
mkdir -p ~/tidybot_ws/src
cd ~/tidybot_ws/src
git clone <this-repo-url> home-tidying-robot
cd ~/tidybot_ws
```

Resolve any remaining ROS dependencies and build:

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
```

Source the overlay (do this in **every** new terminal that runs the demo):

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/tidybot_ws/install/setup.bash
```

---

## 4. Run

A single launch file starts Gazebo, spawns the robot, brings up the
ROS↔Gazebo bridge, tucks the arm, and starts the mission controller:

```bash
ros2 launch home-tidying-robot sim_launch.py
```

What happens:

1. `gz sim` loads `worlds/world.sdf` (server-only, headless by default).
2. `robot_state_publisher` publishes the robot's URDF + TF tree.
3. `ros_gz_sim create` spawns `tidying_robot` at `(-4.0, 0.0, 0.05)`.
4. `ros_gz_bridge parameter_bridge` bridges 21 topics
   (see `config/bridge_config.yaml`).
5. `arm_init_node` publishes the tucked arm pose, then exits.
6. `nav_node` runs the 16-state pick-and-place mission. It logs status every
   ~5 s and prints `=== TIDYBOT PICK-AND-PLACE COMPLETE ===` when finished
   (or after the 900 s mission timeout).

### Watching the simulation with a GUI

The launch file starts Gazebo in **server-only** mode. To visualise it, open
a second terminal (with both ROS and the workspace overlay sourced) and run:

```bash
gz sim -g
```

This attaches a GUI client to the running server.

### Useful debug commands

```bash
# Confirm bridged topics are alive
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once

# Manually drive the robot
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

---

## 5. Complete dependency list

**ROS 2 packages (apt, all `ros-${ROS_DISTRO}-*`):**
`rclpy`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`, `tf2_msgs`,
`xacro`, `robot_state_publisher`, `launch`, `launch_ros`,
`ros_gz_sim`, `ros_gz_bridge` (via `ros-humble-ros-gzharmonic` on Humble or
`ros-jazzy-ros-gz` on Jazzy).

**System packages (apt):**
`build-essential`, `git`, `python3-pip`, `python3-colcon-common-extensions`,
`python3-rosdep`, `python3-vcstool`, `gz-harmonic`.

**Python packages (pip):** none required beyond ROS defaults; see
[requirements.txt](requirements.txt).

**Test dependencies (declared in `package.xml`):** `ament_copyright`,
`ament_flake8`, `ament_pep257`, `python3-pytest`.

---

## 6. Repository layout

```
home-tidying-robot/
├── home_tidying_robot/        # Python package
│   ├── nav_node.py            # 16-state pick-and-place mission controller
│   ├── arm_init_node.py       # Tucks arm at startup, then exits
│   └── arm_controller.py      # Named-pose helper used by nav_node
├── launch/sim_launch.py       # Single launch entrypoint
├── urdf/home-tidying-robot.urdf.xacro
├── worlds/world.sdf           # Two-room home with 6 pickup objects + box
├── config/bridge_config.yaml  # 21 ros_gz_bridge entries
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
```

---

## 7. Troubleshooting

- **`gz: command not found`** — Gazebo Harmonic is not installed. Re-run
  section 2b (on 22.04) and `sudo apt install gz-harmonic`.
- **Bridge starts but `/odom` and `/scan` are empty** — you have the wrong
  `ros_gz_bridge` (Fortress instead of Harmonic). On Humble, install
  `ros-humble-ros-gzharmonic` and uninstall any `ros-humble-ros-gz` /
  `ros-humble-ros-ign-*` packages.
- **`Package 'home-tidying-robot' not found`** — you forgot to source the
  workspace: `source ~/tidybot_ws/install/setup.bash`.
- **Robot spawns but the arm collides with the base** — `arm_init_node` did
  not finish before `nav_node` started. Re-launch; the launch file already
  schedules them in order.
