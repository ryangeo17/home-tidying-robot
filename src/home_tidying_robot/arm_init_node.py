"""Publish a tucked arm pose once the robot has spawned, then exit."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Tucked pose: upper arm ~horizontal forward, forearm folded back up
TUCKED_POSITIONS = {
    'right_shoulder_joint/cmd': 1.5,
    'right_elbow_joint/cmd': -1.5,
    'right_wrist_joint/cmd': 0.0,
    'right_gripper_left_finger_joint/cmd': 0.0,
    'right_gripper_right_finger_joint/cmd': 0.0,
}

PUBLISH_COUNT = 10  # repeat a few times to ensure controllers receive it
PUBLISH_HZ = 10.0


class ArmInitNode(Node):

    def __init__(self):
        super().__init__('arm_init_node')
        self.pubs = {
            topic: self.create_publisher(Float64, topic, 10)
            for topic in TUCKED_POSITIONS
        }
        self.spawned = False
        self.publishes_remaining = PUBLISH_COUNT

        self.create_subscription(JointState, 'joint_states', self._js_cb, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info('Waiting for robot to spawn...')

    def _js_cb(self, msg):
        if not self.spawned:
            self.spawned = True
            self.get_logger().info('Robot spawned — sending tucked arm pose.')

    def _tick(self):
        if not self.spawned:
            return

        for topic, position in TUCKED_POSITIONS.items():
            msg = Float64()
            msg.data = position
            self.pubs[topic].publish(msg)

        self.publishes_remaining -= 1
        if self.publishes_remaining <= 0:
            self.get_logger().info('Arm tucked. Shutting down arm_init_node.')
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = ArmInitNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
