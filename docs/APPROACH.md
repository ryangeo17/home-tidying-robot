# Home Tidying Robot Simulation — Approach Document

## Overview

This project implements a full ROS 2 + Gazebo simulation of a home-tidying robot capable of autonomous navigation, object pickup, and return-to-base placement. The final system spawns a custom mobile manipulator in a two-room apartment environment, navigates through both rooms, identifies and collects objects, and returns them to a collection box. Everything is fully self-contained and runs through a single launch command.

---

## Robot Model

The robot is a custom mobile manipulator defined entirely in URDF/xacro using primitive geometries. No external meshes or prebuilt robot descriptions were used.

### Base and Locomotion

The base is a 4-wheel skid-steer platform (0.50 m × 0.40 m × 0.12 m, ~12 kg). Skid-steer was chosen over differential caster designs because it is more stable in Gazebo's ODE contact model and avoids caster-induced instability on uneven collision meshes.

Each wheel is actively driven. Wheel collisions are cylindrical for stable ground contact, and all wheel links include physically valid inertial tensors. A `base_footprint` frame is used for ROS navigation compatibility. Motion is controlled via a DiffDrive-style plugin consuming `/cmd_vel` and publishing `/odom` and TF transforms.

### Torso, Arms, and End-Effector

A rigid vertical torso is mounted on the base and serves as the attachment point for all upper-body components.

The **right arm** is a planar 3-DOF manipulator (shoulder, elbow, wrist). Each joint is independently actuated using PID-controlled `JointPositionController` plugins.

The **left arm** is purely cosmetic — it exists to preserve symmetry and visual completeness.

The **gripper** is a simple parallel-jaw mechanism. However, because reliable force-based grasping in ODE is unstable at this scale, object attachment is handled using Gazebo's detachable joint mechanism (fixed joint insertion/removal). The fingers still animate for realism, but object retention is simulated through physics-level attachment rather than fragile friction tuning.

### Sensors

The robot includes two sensors:

- **Forward-facing RGB camera** mounted on the torso (ROS 2 image topic published, not currently used in decision-making).
- **2D LiDAR** (360 samples) mounted on the base, used for obstacle detection and reactive avoidance.

### Physical Fidelity

All links include correct mass properties, valid inertia tensors (no identity or placeholder matrices), and explicit center-of-mass offsets where geometry is asymmetric.

---

## Home Environment

The environment is a two-room apartment (approximately 10 m × 8 m) split by an interior wall with a 2 m doorway connecting the rooms.

### Layout

- **Room 1 (Living Room):** sofa, table, chairs, bookshelf
- **Room 2 (Bedroom):** bed, desk, wardrobe, side table
- **Doorway:** central passage enabling constrained navigation between rooms

All furniture is constructed from SDF primitives with full collision geometry, including internal structure (e.g., bookshelf shelves, chair legs). This ensures realistic LiDAR interaction and prevents "ghost navigation" through hollow meshes.

---

## Objects and Task Elements

Six small objects are distributed across both rooms. These are simple geometric primitives (boxes and cylinders, 0.08–0.12 m scale, 0.08–0.15 kg mass).

Each object has full collision geometry, valid inertia tensors, and surface friction tuned for stable resting contact.

A **collection box** is placed in Room 1 near the robot's spawn location, constructed as an open-top container with physically simulated walls.

---

## Navigation and Task Execution

The system is controlled by a **finite state machine (FSM)** that coordinates navigation, arm motion, and object handling.

For each object:

1. Navigate to precomputed approach position
2. Align heading toward object
3. Lower arm into pickup pose
4. Attach object (detachable joint)
5. Lift and stow arm
6. Navigate to collection box
7. Place object
8. Repeat

### Navigation Controller

Navigation is implemented using a **pure-pursuit controller** rather than Nav2. The system drives toward waypoint goals using proportional steering, rotates in place when angular error exceeds a threshold, and reduces speed near targets for precision alignment.

**Obstacle avoidance** is LiDAR-based and reactive. A front arc is monitored, and forward motion is halted if obstacles are detected within a threshold distance.

A critical design detail: obstacle avoidance is **disabled near target objects** — otherwise, the robot would continuously avoid the object it is trying to pick up.

### Arm Control

The arm uses a **discrete pose system** rather than IK:

| Pose | Description |
|------|-------------|
| `TUCKED` | Arm stowed for transit |
| `READY` | Arm prepared for action |
| `REACH_DOWN` | Arm extended downward toward object |
| `GRASP` | Gripper engaged |
| `LIFT` | Object raised |
| `OVER_BOX` | Arm positioned above collection box |
| `RELEASE` | Object released |

Each pose corresponds to predefined joint angles. Transitions are gated by **joint settle detection**, ensuring that motion completes before proceeding. This avoids dependency on IK solvers and eliminates failure modes like singularities or unreachable configurations.

---

## Key Engineering Tradeoffs

Several deliberate tradeoffs were made to prioritize system reliability:

| Decision | Rationale |
|----------|-----------|
| Hardcoded object positions vs. perception | Simplifies task execution and ensures deterministic evaluation |
| Detachable joint grasping vs. physics-based grasping | ODE contact grasping is unstable at this scale; fixed-joint attachment ensures consistent behavior |
| Pure-pursuit vs. Nav2 | The environment is known and static, making full SLAM unnecessary |
| Pose-based arm control vs. IK/MoveIt | Reduces dependency complexity and eliminates solver instability |
| Reactive avoidance vs. global planning | Sufficient for a structured environment with limited obstacle density |

---

## Debugging and System Issues

### Wheel Slipping / No Translation Under Velocity Commands

- **Symptom:** Wheels spun but odometry showed near-zero displacement.
- **Diagnosis:** Friction parameters present in URDF were not taking effect; inspection of generated SDF revealed missing surface properties.
- **Fix:** Confirmed URDF-to-SDF friction loss and relocated contact parameters into Gazebo `<gazebo>` extensions.

### Arm Self-Collision at Spawn

- **Symptom:** Secondary arm spawned intersecting the base, causing immediate joint explosions.
- **Diagnosis:** Initial fix attempted in URDF by changing default joint poses, but this caused TF inconsistencies. The real issue was that controllers were not active at spawn, so Gazebo applied raw initial geometry.
- **Fix:** Moved initialization to a ROS 2 arm-init node that forces a safe tucked pose after spawn and before mission start, separating spawn state from controlled state.

### Robot Not Moving (Gazebo Paused State)

- **Symptom:** Simulation ran with no robot motion despite correct control commands and topic activity.
- **Diagnosis:** System was launched without Gazebo's `-r` (unpause) flag, leaving the physics engine paused while all nodes executed normally — creating the false impression of a control failure.
- **Fix:** Enabled auto-run (`-r`) so physics stepped on startup.

---

## Performance and Execution

The system completes a full 6-object run in **under 5 minutes** of simulated time. Each pick-and-place cycle is consistent and repeatable due to deterministic state transitions and fixed world structure.

All required topics publish continuously during execution: odometry, TF transforms, camera feed, and LiDAR scan data.

---

## Comparison: My Build vs. Drift CLI

| Category | Home-Tidying-Robot | Drift-Home-Tidying-Robot |
|----------|-------------------|--------------------------|
| Objects | 6 (blocks + cylinders) | 5 (blocks + cylinders) |
| State Machine | 16 states | 8 states |
| Python LOC | 822 | 401 |
| Build System | `ament_python` (clean) | CMake + unused C++ stub |
| Grasping | Detachable-joint physics (real attach/detach) | Simulated (arm pose change only) |
| Gripper Control | PID-controlled parallel-jaw fingers | Scoop-style, no actual close command |
| Navigation | Pure-pursuit + 3-sector LiDAR avoidance | Proportional + simple right-turn avoidance |
| Waypoint Tolerance | 0.12 m (precise) | 0.35 m (loose) |
| Arm Settle Detection | Yes — joint-level tolerance checking | No — fixed timers only |
| Gripper Offset Compensation | Yes — IK-like DX/DY math | No |
| Named Arm Poses | 7 (`TUCKED` → `RELEASE`) | 4 (`HOME` → `DEPOSIT`) |
| Arm Init Sequence | Dedicated node tucks arm on spawn | Single `HOME` command at start |
| Bridge Config | 21 topics via YAML file | 7 topics inline in launch |
| README / Docs | 252-line README, platform matrix, troubleshooting | None |
| Tests | flake8 + pep257 + copyright | Linting configured but no tests written |
| Mission Timeout | 900 s graceful exit | None |
| Status Logging | Every 5 s + final 9-line summary | Every 1 s + JSON status at end |
| Launch Timing | Parallel (fast startup) | Staggered delays (6s, 8s, 10s) |

### Key Advantages

- **Manipulation is real.** Detachable-joint attach/detach means the robot actually picks things up in physics. Drift's version just changes arm poses — the objects don't actually move with the gripper.
- **Navigation is tighter.** 0.12 m tolerance with gripper-offset compensation vs. 0.35 m tolerance with no offset math. The robot actually positions its gripper over the object rather than getting "close enough."
- **State machine is more robust.** 16 states vs. 8 handles the full pick-place lifecycle (reach → attach → grasp → lift → tuck → nav → align → place → detach → release → tuck) rather than lumping steps into timed sequences.
- **Arm control is smarter.** Settle detection checks actual joint angles before proceeding. Drift uses hardcoded sleep timers — if the arm is slow, it moves on anyway.

---

## Future Improvements

If extended further, the most impactful upgrades would be:

- Vision-based object detection using the existing camera feed
- SLAM-based mapping and Nav2 integration
- Physics-based grasping using tuned contact models instead of detachable joints
- Automated inertia and URDF validation in CI
- Proper IK-based / MoveIt arm control for general 3D manipulation
- Global path planning through doorway-aware costmaps