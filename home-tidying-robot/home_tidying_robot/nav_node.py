"""Waypoint-following navigation node with LiDAR obstacle avoidance."""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class NavNode(Node):

    # Waypoints trace a loop through both rooms, passing near all 6 objects.
    # Robot spawns at roughly (-4.0, 0.0). Collection box at (-4.2, -3.0).
    #
    # World layout reference:
    #   Room 1 (x<0): table(-3,1), chairs(-3,±), bookshelf(-4.82,-2.5),
    #                 sofa(-1.8,3.55), collection box(-4.2,-3.0)
    #   Room 2 (x>0): bed(3.6,2.5), bedside(2.2,1.6), wardrobe(4.72,-2.0),
    #                 desk(2.0,-3.6)
    #   Doorway: x=0, y in [-1, 1]
    #   Objects: red(-2,-2), blue(-1.5,2.8), yellow(-1.5,-1.2),
    #            orange(2,-1.5), purple(3,-2.8), green(4,-0.8)
    WAYPOINTS = [
        (-3.0,  0.0),
        (-2.0, -2.0),
        (-1.5, -1.2),
        (-2.5,  2.0),
        (-1.5,  2.5),
        (-0.5,  0.0),
        ( 1.0,  0.0),
        ( 2.5, -1.0),
        ( 2.0, -1.5),
        ( 2.5, -2.5),
        ( 3.5, -0.8),
        ( 2.5,  1.0),
        ( 1.0,  0.0),
        (-0.5,  0.0),
        (-3.0,  0.0),
        (-4.0,  0.0),
    ]

    WAYPOINT_TOLERANCE = 0.3
    OBSTACLE_DIST = 0.40
    FRONT_ARC_DEG = 35.0
    LINEAR_SPEED = 0.25
    TURN_KP = 0.8
    STEER_KP = 0.4
    MAX_ANGULAR = 1.0
    TIMER_HZ = 10
    STATUS_INTERVAL_S = 5.0
    TIMEOUT_S = 240.0

    def __init__(self):
        super().__init__('nav_node')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.create_timer(1.0 / self.TIMER_HZ, self._control_loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.obstacle_ahead = False
        self.obstacle_left = False
        self.obstacle_right = False

        self.wp_index = 0
        self.done = False

        self.dist_traveled = 0.0
        self.prev_x = None
        self.prev_y = None
        self.room1_visited = False
        self.room2_visited = False

        self.start_time = None
        self.tick = 0
        self.summary_printed = False

        self.get_logger().info(
            f'NavNode started — {len(self.WAYPOINTS)} waypoints, '
            f'timeout {self.TIMEOUT_S}s'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

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
        if n == 0:
            return

        inc = msg.angle_increment
        if inc == 0.0:
            return

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

        front_min = min_range(front_lo, front_hi)
        left_min = min_range(left_lo, left_hi)
        right_min = min_range(right_lo, right_hi)

        self.obstacle_ahead = front_min < self.OBSTACLE_DIST
        self.obstacle_left = left_min < self.OBSTACLE_DIST
        self.obstacle_right = right_min < self.OBSTACLE_DIST

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def _control_loop(self):
        twist = Twist()

        if not self.odom_received:
            self.cmd_pub.publish(twist)
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.TIMEOUT_S and not self.done:
            self.done = True
            self.get_logger().warn('Timeout reached — stopping.')

        if self.done:
            self.cmd_pub.publish(twist)
            self._print_summary(elapsed)
            return

        self.tick += 1
        if self.tick % int(self.STATUS_INTERVAL_S * self.TIMER_HZ) == 0:
            room = '1' if self.x < 0 else '2'
            self.get_logger().info(
                f'[status] pos=({self.x:.2f},{self.y:.2f}) yaw={self.yaw:.2f} '
                f'dist={self.dist_traveled:.1f}m room={room} '
                f'wp={self.wp_index}/{len(self.WAYPOINTS)} t={elapsed:.0f}s'
            )

        # Obstacle avoidance
        if self.obstacle_ahead:
            if self.obstacle_left and not self.obstacle_right:
                twist.angular.z = -0.6
            elif self.obstacle_right and not self.obstacle_left:
                twist.angular.z = 0.6
            else:
                twist.angular.z = 0.6
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        # Waypoint following
        tx, ty = self.WAYPOINTS[self.wp_index]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < self.WAYPOINT_TOLERANCE:
            self.wp_index += 1
            if self.wp_index >= len(self.WAYPOINTS):
                self.done = True
                self.get_logger().info('All waypoints reached.')
                self.cmd_pub.publish(twist)
                self._print_summary(elapsed)
                return
            ntx, nty = self.WAYPOINTS[self.wp_index]
            self.get_logger().info(
                f'Waypoint {self.wp_index}/{len(self.WAYPOINTS)}: '
                f'({ntx:.1f},{nty:.1f}) | '
                f'pos=({self.x:.2f},{self.y:.2f}) | '
                f'dist={self.dist_traveled:.1f}m'
            )
            self.cmd_pub.publish(twist)
            return

        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(
            math.sin(target_yaw - self.yaw),
            math.cos(target_yaw - self.yaw),
        )

        if abs(yaw_err) > 0.15:
            twist.angular.z = max(
                -self.MAX_ANGULAR,
                min(self.MAX_ANGULAR, self.TURN_KP * yaw_err),
            )
        else:
            twist.linear.x = self.LINEAR_SPEED
            twist.angular.z = max(
                -self.MAX_ANGULAR,
                min(self.MAX_ANGULAR, self.STEER_KP * yaw_err),
            )

        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    def _print_summary(self, elapsed):
        if self.summary_printed:
            return
        self.summary_printed = True

        self.get_logger().info('=== TIDYBOT NAV COMPLETE ===')
        self.get_logger().info(f'Total distance: {self.dist_traveled:.2f} m')
        self.get_logger().info(
            f'Waypoints: {self.wp_index}/{len(self.WAYPOINTS)}'
        )
        self.get_logger().info(f'Room 1 visited: {self.room1_visited}')
        self.get_logger().info(f'Room 2 visited: {self.room2_visited}')
        self.get_logger().info(f'Elapsed: {elapsed:.1f} s')
        self.get_logger().info('============================')


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
