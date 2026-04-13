"""Arm motion primitives: named poses + settle detection.



Joint limits (from URDF):
    right_shoulder_joint: [-2.356, +2.356]   (0 = straight down, +pi/2 = forward)
    right_elbow_joint:    [-2.356,  0.0]     (one-sided; negative folds forearm up)
    right_wrist_joint:    [-1.571, +1.571]
    gripper fingers:      [0.0, 0.02]
"""

from std_msgs.msg import Float64


# ---------------------------------------------------------------------------
# Named poses — (shoulder, elbow, wrist, gripper_open_m)
# ---------------------------------------------------------------------------
# TUCKED matches arm_init_node.TUCKED_POSITIONS so mission start state agrees
# with the startup pose.
POSES = {
    'TUCKED':     (1.5, -1.5, 0.0, 0.0),   # folded, fingers closed
    'READY':      (0.8, -1.0, 0.0, 0.02),  # arm up-forward, fingers open
    'REACH_DOWN': (1.0, -1.0, 0.0, 0.02),  # forearm vertical, palm ~ground
    'GRASP':      (1.0, -1.0, 0.0, 0.0),   # same geometry, fingers closed
    'LIFT':       (1.5, -1.8, 0.0, 0.0),   # lifted, fingers closed
    'OVER_BOX':   (1.5, -2.0, 0.0, 0.0),   # elbow bent more, palm above box
    'RELEASE':    (1.5, -2.0, 0.0, 0.02),  # same geometry, fingers open
}

# Commanded-vs-measured tolerance for is_settled(). Rad for revolute, m for
# prismatic gripper fingers. The arm PID gains are soft (shoulder P=30) so the
# measured position lags the command by ~0.02-0.04 rad in steady state.
REVOLUTE_TOL = 0.06
PRISMATIC_TOL = 0.005


class ArmController:
    """Publishes arm joint commands and tracks measured state for settling."""

    SHOULDER = 'right_shoulder_joint'
    ELBOW = 'right_elbow_joint'
    WRIST = 'right_wrist_joint'
    LEFT_FINGER = 'right_gripper_left_finger_joint'
    RIGHT_FINGER = 'right_gripper_right_finger_joint'

    def __init__(self, node):
        self._node = node
        self._logger = node.get_logger()

        self._pubs = {
            self.SHOULDER: node.create_publisher(
                Float64, f'/{self.SHOULDER}/cmd', 10),
            self.ELBOW: node.create_publisher(
                Float64, f'/{self.ELBOW}/cmd', 10),
            self.WRIST: node.create_publisher(
                Float64, f'/{self.WRIST}/cmd', 10),
            self.LEFT_FINGER: node.create_publisher(
                Float64, f'/{self.LEFT_FINGER}/cmd', 10),
            self.RIGHT_FINGER: node.create_publisher(
                Float64, f'/{self.RIGHT_FINGER}/cmd', 10),
        }

        # Most recently commanded target per joint (None until first goto).
        self._target = {name: None for name in self._pubs}

        # Most recently measured position per joint (from /joint_states).
        self._measured = {name: None for name in self._pubs}

        self._current_pose_name = None

    # ------------------------------------------------------------------
    # State update — call from the owning node's /joint_states callback
    # ------------------------------------------------------------------

    def update_joint_state(self, msg):
        """Consume a sensor_msgs/JointState message and cache positions."""
        for name, position in zip(msg.name, msg.position):
            if name in self._measured:
                self._measured[name] = position

    # ------------------------------------------------------------------
    # Pose commands
    # ------------------------------------------------------------------

    def goto_pose(self, pose_name):
        """Publish targets for a named pose. Non-blocking."""
        if pose_name not in POSES:
            raise ValueError(f'Unknown arm pose: {pose_name}')

        shoulder, elbow, wrist, gripper = POSES[pose_name]
        self._publish(self.SHOULDER, shoulder)
        self._publish(self.ELBOW, elbow)
        self._publish(self.WRIST, wrist)
        self._publish(self.LEFT_FINGER, gripper)
        self._publish(self.RIGHT_FINGER, gripper)
        self._current_pose_name = pose_name
        self._logger.info(
            f'[arm] goto {pose_name} '
            f'(s={shoulder:.2f} e={elbow:.2f} w={wrist:.2f} g={gripper:.3f})'
        )

    def republish_current(self):
        """Re-send the latest targets (useful if a command was dropped)."""
        for name, target in self._target.items():
            if target is not None:
                self._publish(name, target, log=False)

    def _publish(self, joint_name, value, log=True):
        msg = Float64()
        msg.data = float(value)
        self._pubs[joint_name].publish(msg)
        self._target[joint_name] = float(value)

    # ------------------------------------------------------------------
    # Settle detection
    # ------------------------------------------------------------------

    def is_settled(self):
        """True when every joint is within tolerance of its last target.

        If no target has been commanded yet, or we have not yet received
        measured state for a commanded joint, returns False.
        """
        for name, target in self._target.items():
            if target is None:
                return False
            measured = self._measured[name]
            if measured is None:
                return False
            tol = (PRISMATIC_TOL
                   if name in (self.LEFT_FINGER, self.RIGHT_FINGER)
                   else REVOLUTE_TOL)
            if abs(measured - target) > tol:
                return False
        return True

    def current_pose_name(self):
        return self._current_pose_name
