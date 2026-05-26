import rclpy
import math
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# ── Parámetros calibrados ─────────────────────────────────────
KV          = 0.20
V_MAX       = 0.3
V_MIN       = 0.04
THRESHOLD_D = 0.05

KW          = 0.50
W_MAX       = 0.4
W_MIN       = 0.08
THRESHOLD_A = 0.02

# Tiempo de descanso
PAUSE_TIME = 1.0 # [s]

# Cuadrada
WAYPOINTS = [
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0),
    (0.0, 0.0),   # return to start
]


class HuskySquare(Node):

    def __init__(self):
        super().__init__('husky_square')

        self.x  = None
        self.y  = None
        self.th = None

        self.create_subscription(
            Odometry,
            '/a200_1075/platform/odom/filtered',
            self._odom_cb,
            qos.qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/a200_1075/cmd_vel',
            10)

        # Waypoint sequence: 4x (move 1m forward, turn 90°)
        self.steps = (
            [('move', SQUARE_SIDE), ('turn', 90)] * 4
        )
        self.step_idx   = 0
        self.step_start_x  = None
        self.step_start_y  = None
        self.step_start_th = None

        self.create_timer(0.05, self._loop)

    # ── Odometría ─────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.th = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # ── Publicar ──────────────────────────────────────────────

    def _pub(self, v=0.0, w=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)

    # ── Primitivas ────────────────────────────────────────────

    def move_forward(self, meters) -> bool:
        """Returns True when goal reached."""
        dx = self.x - self.step_start_x
        dy = self.y - self.step_start_y
        travelled  = math.sqrt(dx**2 + dy**2)
        remaining  = meters - travelled
        if remaining <= THRESHOLD_D:
            self._pub()
            return True
        V = max(V_MIN, min(V_MAX, KV * remaining))
        self._pub(v=V)
        return False

    def turn_90(self) -> bool:
        """Returns True when goal reached. Positive = left."""
        turned = math.atan2(
            math.sin(self.th - self.step_start_th),
            math.cos(self.th - self.step_start_th))
        remaining = math.radians(86) - turned
        if remaining <= THRESHOLD_A:
            self._pub()
            return True
        W = max(W_MIN, min(W_MAX, KW * remaining))
        self._pub(w=W)
        return False

    # ── Loop ──────────────────────────────────────────────────

    def _loop(self):
        if self.x is None:
            return

        if self.step_idx >= len(self.steps):
            self._pub()
            return

        kind, arg = self.steps[self.step_idx]

        # Init snapshot for this step
        if self.step_start_x is None:
            self.step_start_x  = self.x
            self.step_start_y  = self.y
            self.step_start_th = self.th

        # Execute
        if kind == 'move':
            done = self.move_forward(arg)
        else:
            done = self.turn_90()

        if done:
            self.step_idx  += 1
            self.step_start_x  = None
            self.step_start_y  = None
            self.step_start_th = None


def main(args=None):
    rclpy.init(args=args)
    node = HuskySquare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

