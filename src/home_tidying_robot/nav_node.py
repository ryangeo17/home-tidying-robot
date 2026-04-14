"""Pick-and-place mission: navigate to each object, pick it up, return it
to the collection box, then repeat.

This replaces the old waypoint-loop with a state machine:

    INIT
     │
     ▼
    NAV_TO_OBJECT ──► ALIGN_OBJECT ──► PICK_REACH ──► PICK_ATTACH
                                                           │
                                                           ▼
    NAV_TO_BOX ◄── PICK_TUCK ◄── PICK_LIFT ◄── PICK_GRASP
        │
        ▼
    ALIGN_BOX ──► PLACE_OVER ──► PLACE_DETACH ──► PLACE_RELEASE ──► PLACE_TUCK
                                                                         │
                                                                         ▼
                                                                   NEXT_OBJECT
                                                                         │
                                                   (more?) ◄─────────────┤
                                                      │                  │
                                                      ▼                  ▼
                                              NAV_TO_OBJECT             DONE

Detection of objects is by hardcoded world positions (same coordinates as
worlds/world.sdf). Pick/place is implemented via the DetachableJoint plugin
on each pickup model: publishing std_msgs/Empty on /<obj>/attach creates a
fixed joint between the object and right_gripper_palm; /<obj>/detach
removes it.
"""

import enum
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Empty

from home_tidying_robot.arm_controller import ArmController


# ---------------------------------------------------------------------------
# Mission configuration
# ---------------------------------------------------------------------------

class PickObject:
    def __init__(self, color, topic_base, xy):
        self.color = color
        self.topic_base = topic_base   # e.g. 'pickup_block_red'
        self.xy = xy                   # (world_x, world_y)


# World positions copied from worlds/world.sdf. Order chosen to roughly
# minimize back-and-forth through the doorway (Room 1 first, then Room 2).
OBJECTS = [
    PickObject('red',    'pickup_block_red',       (-2.0, -2.0)),
    PickObject('yellow', 'pickup_block_yellow',    (-1.5, -1.2)),
    PickObject('blue',   'pickup_cylinder_blue',   (-1.5,  2.8)),
    PickObject('orange', 'pickup_cylinder_orange', (2.0, -1.5)),
    PickObject('green',  'pickup_block_green',     (4.0, -0.8)),
    PickObject('purple', 'pickup_block_purple',    (3.0, -2.8)),
]

# Collection box centre, matches worlds/world.sdf.
BOX_XY = (-4.2, -3.0)

# Robot spawn position in the world frame (must match sim_launch.py).
SPAWN_X = -4.0
SPAWN_Y = 0.0

# Gripper tip offset in base_link frame at REACH_DOWN pose. The right shoulder
# joint origin is (0, -0.12, 0.60) in base_link. With shoulder=1.0, elbow=-1.0
# the forearm hangs vertically and the gripper tip lies at roughly
# (+0.236, -0.12) in the XY plane. These are hand-tuned — verify visually and
# adjust before shipping.
GRIPPER_DX_REACH = 0.236
GRIPPER_DY_REACH = -0.12

# Same for the OVER_BOX pose (shoulder=1.5, elbow=-2.0). Forearm is angled
# forward/up, so the gripper sits lower in X but higher in Z.
GRIPPER_DX_OVERBOX = 0.113
GRIPPER_DY_OVERBOX = -0.12

# ---------------------------------------------------------------------------
# Obstacle-avoidance waypoints
# ---------------------------------------------------------------------------
# Doorway centre — the divider wall at x=0 has a 2 m opening at y ∈ [-1,+1].
DOORWAY_WP = (0.0, 0.0)

# Open corridor SE of the table / Chair 2 cluster in Room 1.
R1_CLEAR_SOUTH = (-1.2, -1.0)

# South of spawn — used once on the very first leg (Spawn → Red) to avoid
# clipping Chair 2 when heading SE from the spawn position.
SPAWN_CLEAR_SOUTH = (-4.0, -1.5)


# ---------------------------------------------------------------------------
# States
# ---------------------------------------------------------------------------

class State(enum.Enum):
    INIT = 'INIT'
    NAV_TO_OBJECT = 'NAV_TO_OBJECT'
    ALIGN_OBJECT = 'ALIGN_OBJECT'
    PICK_REACH = 'PICK_REACH'
    PICK_ATTACH = 'PICK_ATTACH'
    PICK_GRASP = 'PICK_GRASP'
    PICK_LIFT = 'PICK_LIFT'
    PICK_TUCK = 'PICK_TUCK'
    NAV_TO_BOX = 'NAV_TO_BOX'
    ALIGN_BOX = 'ALIGN_BOX'
    PLACE_OVER = 'PLACE_OVER'
    PLACE_DETACH = 'PLACE_DETACH'
    PLACE_RELEASE = 'PLACE_RELEASE'
    PLACE_TUCK = 'PLACE_TUCK'
    NEXT_OBJECT = 'NEXT_OBJECT'
    DONE = 'DONE'


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class NavNode(Node):

    # --- Driving / navigation parameters ---
    WAYPOINT_TOLERANCE = 0.12     # m — enter ALIGN when within this distance
    ALIGN_YAW_TOL = 0.05          # rad — in-place alignment tolerance
    OBSTACLE_DIST = 0.40          # m — LiDAR trip distance for avoidance
    OBSTACLE_IGNORE_RADIUS = 0.7  # m — disable avoidance within this of target
    FRONT_ARC_DEG = 35.0          # LiDAR arc considered "front"
    LINEAR_SPEED = 0.22
    APPROACH_SPEED = 0.08         # slowed final approach
    TURN_KP = 0.9
    STEER_KP = 0.5
    MAX_ANGULAR = 1.0

    # --- Loop + timing ---
    TIMER_HZ = 10
    STATUS_INTERVAL_S = 5.0
    TIMEOUT_S = 900.0             # 15 min budget for all objects

    # --- Sub-state dwell ---
    POST_ALIGN_SETTLE_TICKS = 5       # ticks held at target yaw before PICK
    ATTACH_DWELL_TICKS = 5            # ticks between publish + next substate
    ARM_SETTLE_TIMEOUT_TICKS = 60     # 6 s max per arm motion

    def __init__(self):
        super().__init__('nav_node')

        # --- Publishers ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.attach_pubs = {}
        self.detach_pubs = {}
        for obj in OBJECTS:
            self.attach_pubs[obj.topic_base] = self.create_publisher(
                Empty, f'/{obj.topic_base}/attach', 1)
            self.detach_pubs[obj.topic_base] = self.create_publisher(
                Empty, f'/{obj.topic_base}/detach', 1)

        # --- Subscribers ---
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_subscription(JointState, 'joint_states', self._js_cb, 10)

        # --- Arm controller (plain helper, not a Node) ---
        self.arm = ArmController(self)

        # --- Control timer ---
        self.create_timer(1.0 / self.TIMER_HZ, self._control_loop)

        # --- Odom / scan state ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.scan_received = False
        self.joint_state_received = False

        self.obstacle_ahead = False
        self.obstacle_left = False
        self.obstacle_right = False

        # --- Mission state ---
        self.state = State.INIT
        self.obj_index = 0
        self.state_ticks = 0
        self.aligned_count = 0
        self.nav_target = None        # (x, y) world-frame target
        self.target_yaw = None        # desired heading on arrival
        self.nav_waypoints = []       # ordered (x,y) waypoints for current nav
        self.wp_index = 0             # index into nav_waypoints

        # --- Metrics ---
        self.dist_traveled = 0.0
        self.prev_x = None
        self.prev_y = None
        self.objects_picked = 0
        self.objects_placed = 0
        self.room1_visited = False
        self.room2_visited = False

        self.start_time = None
        self.tick = 0
        self.summary_printed = False
        self.done = False

        self.get_logger().info(
            f'NavNode started — {len(OBJECTS)} objects, '
            f'timeout {self.TIMEOUT_S:.0f}s'
        )

    # ======================================================================
    # Sensor callbacks
    # ======================================================================

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x + SPAWN_X
        self.y = msg.pose.pose.position.y + SPAWN_Y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.odom_received = True

        if self.prev_x is not None:
            self.dist_traveled += math.hypot(
                self.x - self.prev_x, self.y - self.prev_y
            )
        self.prev_x = self.x
        self.prev_y = self.y

        if self.x < 0.0:
            self.room1_visited = True
        else:
            self.room2_visited = True

    def _scan_cb(self, msg):
        n = len(msg.ranges)
        if n == 0 or msg.angle_increment == 0.0:
            return
        self.scan_received = True

        inc = msg.angle_increment
        angle_min = msg.angle_min
        half_arc = math.radians(self.FRONT_ARC_DEG)

        def idx(angle):
            i = int(round((angle - angle_min) / inc))
            return max(0, min(n - 1, i))

        front_lo = idx(-half_arc)
        front_hi = idx(half_arc)
        left_lo = idx(half_arc)
        left_hi = idx(half_arc * 2.0)
        right_lo = idx(-half_arc * 2.0)
        right_hi = idx(-half_arc)

        def min_range(lo, hi):
            if lo > hi:
                lo, hi = hi, lo
            vals = [
                r for r in msg.ranges[lo:hi + 1]
                if math.isfinite(r) and r > 0.0
            ]
            return min(vals) if vals else float('inf')

        self.obstacle_ahead = min_range(front_lo, front_hi) < self.OBSTACLE_DIST
        self.obstacle_left = min_range(left_lo, left_hi) < self.OBSTACLE_DIST
        self.obstacle_right = min_range(right_lo, right_hi) < self.OBSTACLE_DIST

    def _js_cb(self, msg):
        self.joint_state_received = True
        self.arm.update_joint_state(msg)

    # ======================================================================
    # Control loop
    # ======================================================================

    def _control_loop(self):
        # Need all sensors before planning anything.
        if not (self.odom_received and self.scan_received
                and self.joint_state_received):
            self.cmd_pub.publish(Twist())
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.TIMEOUT_S and not self.done:
            self.get_logger().warn(f'Timeout reached after {elapsed:.1f}s')
            self.cmd_pub.publish(Twist())
            self._transition(State.DONE)

        self.tick += 1
        self.state_ticks += 1

        if self.tick % int(self.STATUS_INTERVAL_S * self.TIMER_HZ) == 0:
            self._log_status(elapsed)

        handler = self._DISPATCH.get(self.state)
        if handler is not None:
            handler(self)

        if self.done and not self.summary_printed:
            self._print_summary(elapsed)

    # ======================================================================
    # State transitions
    # ======================================================================

    def _transition(self, new_state):
        prev = self.state
        self.state = new_state
        self.state_ticks = 0
        self.aligned_count = 0
        self.get_logger().info(
            f'[state] {prev.value} -> {new_state.value}'
        )
        self._on_enter(new_state)

    def _on_enter(self, state):
        if state == State.INIT:
            return
        if state == State.NAV_TO_OBJECT:
            self._plan_object_approach()
        elif state == State.PICK_REACH:
            self.arm.goto_pose('REACH_DOWN')
        elif state == State.PICK_ATTACH:
            obj = OBJECTS[self.obj_index]
            self.attach_pubs[obj.topic_base].publish(Empty())
            self.objects_picked += 1
            self.get_logger().info(f'[attach] {obj.color}')
        elif state == State.PICK_GRASP:
            self.arm.goto_pose('GRASP')
        elif state == State.PICK_LIFT:
            self.arm.goto_pose('LIFT')
        elif state == State.PICK_TUCK:
            self.arm.goto_pose('TUCKED')
        elif state == State.NAV_TO_BOX:
            self._plan_box_approach()
        elif state == State.PLACE_OVER:
            self.arm.goto_pose('OVER_BOX')
        elif state == State.PLACE_DETACH:
            obj = OBJECTS[self.obj_index]
            self.detach_pubs[obj.topic_base].publish(Empty())
            self.objects_placed += 1
            self.get_logger().info(f'[detach] {obj.color}')
        elif state == State.PLACE_RELEASE:
            self.arm.goto_pose('RELEASE')
        elif state == State.PLACE_TUCK:
            self.arm.goto_pose('TUCKED')
        elif state == State.NEXT_OBJECT:
            self.obj_index += 1
        elif state == State.DONE:
            self.done = True
            self.cmd_pub.publish(Twist())

    # ======================================================================
    # Planning helpers
    # ======================================================================

    def _plan_waypoints(self, start, goal):
        """Return intermediate waypoints (plus goal) that avoid obstacles."""
        waypoints = []

        # Cross-room: route through the doorway
        if (start[0] < -0.1 and goal[0] > 0.1) or \
           (start[0] > 0.1 and goal[0] < -0.1):
            waypoints.append(DOORWAY_WP)

        # Chair / table cluster in Room 1: if the remaining segment goes
        # from south (y < -0.5) to north (y > 1.5) or vice-versa, detour
        # through the open corridor south-east of the cluster.
        prev = waypoints[-1] if waypoints else start
        if prev[0] < 0 and goal[0] < 0:
            if (prev[1] < -0.5 and goal[1] > 1.5) or \
               (prev[1] > 1.5 and goal[1] < -0.5):
                waypoints.append(R1_CLEAR_SOUTH)

        waypoints.append(goal)
        return waypoints

    def _plan_object_approach(self):
        obj = OBJECTS[self.obj_index]
        ox, oy = obj.xy
        start = (self.x, self.y)

        # Build obstacle-free route to the object's position.
        route = self._plan_waypoints(start, (ox, oy))

        # First leg ever (Spawn → Red): prepend the south-of-spawn waypoint
        # so the diagonal doesn't clip Chair 2.
        if self.obj_index == 0 and self.objects_picked == 0:
            route.insert(0, SPAWN_CLEAR_SOUTH)

        # Compute approach heading from the penultimate waypoint (the
        # direction the robot is actually arriving from) so the gripper
        # offset aligns correctly.
        if len(route) > 1:
            prev = route[-2]
        else:
            prev = start
        yaw = math.atan2(oy - prev[1], ox - prev[0])
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        # robot = obj - R(yaw) * (DX, DY)
        target_x = ox - GRIPPER_DX_REACH * cos_y + GRIPPER_DY_REACH * sin_y
        target_y = oy - GRIPPER_DX_REACH * sin_y - GRIPPER_DY_REACH * cos_y

        # Replace the last waypoint (raw object pos) with the offset approach
        route[-1] = (target_x, target_y)

        self.nav_waypoints = route
        self.wp_index = 0
        self.target_yaw = yaw
        self.get_logger().info(
            f'[plan] object {obj.color} @ ({ox:.2f},{oy:.2f}) '
            f'-> {len(route)} waypoints, '
            f'final ({target_x:.2f},{target_y:.2f}) yaw={yaw:.2f}'
        )

    def _plan_box_approach(self):
        bx, by = BOX_XY
        start = (self.x, self.y)

        route = self._plan_waypoints(start, (bx, by))

        if len(route) > 1:
            prev = route[-2]
        else:
            prev = start
        yaw = math.atan2(by - prev[1], bx - prev[0])
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        target_x = bx - GRIPPER_DX_OVERBOX * cos_y + GRIPPER_DY_OVERBOX * sin_y
        target_y = by - GRIPPER_DX_OVERBOX * sin_y - GRIPPER_DY_OVERBOX * cos_y

        route[-1] = (target_x, target_y)

        self.nav_waypoints = route
        self.wp_index = 0
        self.target_yaw = yaw
        self.get_logger().info(
            f'[plan] box @ ({bx:.2f},{by:.2f}) '
            f'-> {len(route)} waypoints, '
            f'final ({target_x:.2f},{target_y:.2f}) yaw={yaw:.2f}'
        )

    # ======================================================================
    # Driving primitives
    # ======================================================================

    def _drive_toward(self, target):
        """Pure-pursuit drive to (tx, ty). Returns True when arrived."""
        tx, ty = target
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.WAYPOINT_TOLERANCE:
            self.cmd_pub.publish(Twist())
            return True

        # Obstacle avoidance — only when far from the target; otherwise the
        # target object itself shows up in the LiDAR arc and trips avoidance.
        if dist > self.OBSTACLE_IGNORE_RADIUS and self.obstacle_ahead:
            twist = Twist()
            if self.obstacle_left and not self.obstacle_right:
                twist.angular.z = -0.6
            else:
                twist.angular.z = 0.6
            self.cmd_pub.publish(twist)
            return False

        target_yaw = math.atan2(dy, dx)
        yaw_err = self._wrap_angle(target_yaw - self.yaw)

        twist = Twist()
        if abs(yaw_err) > 0.3:
            twist.angular.z = self._clamp(
                self.TURN_KP * yaw_err, -self.MAX_ANGULAR, self.MAX_ANGULAR
            )
        else:
            speed = (self.APPROACH_SPEED
                     if dist < self.OBSTACLE_IGNORE_RADIUS
                     else self.LINEAR_SPEED)
            twist.linear.x = speed
            twist.angular.z = self._clamp(
                self.STEER_KP * yaw_err, -self.MAX_ANGULAR, self.MAX_ANGULAR
            )
        self.cmd_pub.publish(twist)
        return False

    def _rotate_to(self, target_yaw):
        """In-place rotation. Returns True once held aligned for enough ticks."""
        yaw_err = self._wrap_angle(target_yaw - self.yaw)
        twist = Twist()
        if abs(yaw_err) > self.ALIGN_YAW_TOL:
            twist.angular.z = self._clamp(
                self.TURN_KP * yaw_err, -0.8, 0.8
            )
            self.cmd_pub.publish(twist)
            self.aligned_count = 0
            return False
        self.cmd_pub.publish(twist)  # zero
        self.aligned_count += 1
        return self.aligned_count >= self.POST_ALIGN_SETTLE_TICKS

    def _wait_arm_settled(self, next_state):
        if self.arm.is_settled():
            self._transition(next_state)
            return
        if self.state_ticks >= self.ARM_SETTLE_TIMEOUT_TICKS:
            self.get_logger().warn(
                f'[arm] {self.state.value} timed out waiting for settle'
            )
            self._transition(next_state)

    def _wait_dwell(self, ticks, next_state):
        if self.state_ticks >= ticks:
            self._transition(next_state)

    # ======================================================================
    # State handlers
    # ======================================================================

    def _do_init(self):
        # arm_init_node has already tucked the arm. Re-send the TUCKED target
        # so the arm_controller has a baseline for is_settled() checks.
        self.arm.goto_pose('TUCKED')
        self._transition(State.NAV_TO_OBJECT)

    def _do_nav_to_object(self):
        if self._drive_toward(self.nav_waypoints[self.wp_index]):
            self.wp_index += 1
            if self.wp_index >= len(self.nav_waypoints):
                self._transition(State.ALIGN_OBJECT)

    def _do_align_object(self):
        if self._rotate_to(self.target_yaw):
            self._transition(State.PICK_REACH)

    def _do_pick_reach(self):
        self._wait_arm_settled(State.PICK_ATTACH)

    def _do_pick_attach(self):
        self._wait_dwell(self.ATTACH_DWELL_TICKS, State.PICK_GRASP)

    def _do_pick_grasp(self):
        self._wait_arm_settled(State.PICK_LIFT)

    def _do_pick_lift(self):
        self._wait_arm_settled(State.PICK_TUCK)

    def _do_pick_tuck(self):
        self._wait_arm_settled(State.NAV_TO_BOX)

    def _do_nav_to_box(self):
        if self._drive_toward(self.nav_waypoints[self.wp_index]):
            self.wp_index += 1
            if self.wp_index >= len(self.nav_waypoints):
                self._transition(State.ALIGN_BOX)

    def _do_align_box(self):
        if self._rotate_to(self.target_yaw):
            self._transition(State.PLACE_OVER)

    def _do_place_over(self):
        self._wait_arm_settled(State.PLACE_DETACH)

    def _do_place_detach(self):
        self._wait_dwell(self.ATTACH_DWELL_TICKS, State.PLACE_RELEASE)

    def _do_place_release(self):
        self._wait_arm_settled(State.PLACE_TUCK)

    def _do_place_tuck(self):
        self._wait_arm_settled(State.NEXT_OBJECT)

    def _do_next_object(self):
        if self.obj_index >= len(OBJECTS):
            self.get_logger().info('All objects processed.')
            self._transition(State.DONE)
        else:
            self._transition(State.NAV_TO_OBJECT)

    def _do_done(self):
        self.cmd_pub.publish(Twist())

    # Dispatch map (bound after class definition below).
    _DISPATCH = None

    # ======================================================================
    # Status / summary
    # ======================================================================

    def _log_status(self, elapsed):
        if self.obj_index < len(OBJECTS):
            obj_str = OBJECTS[self.obj_index].color
        else:
            obj_str = '-'
        room = '1' if self.x < 0 else '2'
        self.get_logger().info(
            f'[status] state={self.state.value} '
            f'pos=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} '
            f'dist={self.dist_traveled:.1f}m room={room} '
            f'obj={self.obj_index}/{len(OBJECTS)}({obj_str}) '
            f'picked={self.objects_picked} placed={self.objects_placed} '
            f't={elapsed:.0f}s'
        )

    def _print_summary(self, elapsed):
        if self.summary_printed:
            return
        self.summary_printed = True
        self.get_logger().info('=== TIDYBOT PICK-AND-PLACE COMPLETE ===')
        self.get_logger().info(
            f'Objects placed: {self.objects_placed}/{len(OBJECTS)}'
        )
        self.get_logger().info(
            f'Objects picked: {self.objects_picked}/{len(OBJECTS)}'
        )
        self.get_logger().info(f'Total distance: {self.dist_traveled:.2f} m')
        self.get_logger().info(f'Room 1 visited: {self.room1_visited}')
        self.get_logger().info(f'Room 2 visited: {self.room2_visited}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f} s')
        self.get_logger().info('=======================================')

    # ======================================================================
    # Utilities
    # ======================================================================

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


# Bind the dispatch map now that the handler methods exist.
NavNode._DISPATCH = {
    State.INIT: NavNode._do_init,
    State.NAV_TO_OBJECT: NavNode._do_nav_to_object,
    State.ALIGN_OBJECT: NavNode._do_align_object,
    State.PICK_REACH: NavNode._do_pick_reach,
    State.PICK_ATTACH: NavNode._do_pick_attach,
    State.PICK_GRASP: NavNode._do_pick_grasp,
    State.PICK_LIFT: NavNode._do_pick_lift,
    State.PICK_TUCK: NavNode._do_pick_tuck,
    State.NAV_TO_BOX: NavNode._do_nav_to_box,
    State.ALIGN_BOX: NavNode._do_align_box,
    State.PLACE_OVER: NavNode._do_place_over,
    State.PLACE_DETACH: NavNode._do_place_detach,
    State.PLACE_RELEASE: NavNode._do_place_release,
    State.PLACE_TUCK: NavNode._do_place_tuck,
    State.NEXT_OBJECT: NavNode._do_next_object,
    State.DONE: NavNode._do_done,
}


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
