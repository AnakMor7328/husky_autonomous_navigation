import rclpy
import math
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# Control angular P
KW           = 0.50

# Saturaciones
W_MAX        = 0.4     # [rad/s] — kept low for first test
W_MIN        = 0.08    # [rad/s] — dead zone

# Criterio de llegada
THRESHOLD_A  = 0.02    # [rad] — stop when within ~1 degree of goal

# Goal: turn left 90 degrees (86 = 90)
TARGET_ANGLE = math.radians(86)   # [rad] 

# Nodo
class HuskyTurnLeft(Node):

    def __init__(self):
        super().__init__('husky_turn_left')

        # ── Orientación (se actualiza con odom) ───────────────────
        self.th       = None   # current yaw
        self.start_th = None   # yaw at start
        self.goal_reached = False

        # ── Subscripción a odometría filtrada ─────────────────────
        self.create_subscription(
            Odometry,
            '/a200_1075/platform/odom/filtered',
            self._odom_cb,
            qos.qos_profile_sensor_data)

        # ── Publicador de velocidad (TwistStamped) ─────────────────
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/a200_1075/cmd_vel',
            10)

        # ── Loop a 20 Hz ──────────────────────────────────────────
        self.create_timer(0.05, self._loop)

        self.get_logger().info('Nodo iniciado. Esperando odometría...')


    # ── Callback odometría ────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny_cosp, cosy_cosp)

        # Save starting yaw on first message
        if self.start_th is None:
            self.start_th = self.th
            self.get_logger().info(
                f'Orientación inicial: {math.degrees(self.start_th):.2f}°')


    # ── Publicar velocidad ────────────────────────────────────────

    def _publish(self, v: float = 0.0, w: float = 0.0):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = v
        msg.twist.angular.z = w
        self.cmd_pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)


    # ── Bucle principal ───────────────────────────────────────────

    def _loop(self):

        # Wait until we have odometry
        if self.th is None:
            self.get_logger().info('Esperando odometría...', once=True)
            return

        # Already done
        if self.goal_reached:
            self._stop()
            return

        # How much have we turned so far (handles wrap-around at ±180°)
        angle_turned = math.atan2(
            math.sin(self.th - self.start_th),
            math.cos(self.th - self.start_th))

        # Remaining angle to turn
        remaining = TARGET_ANGLE - angle_turned

        self.get_logger().info(
            f'Girado: {math.degrees(angle_turned):.2f}° | '
            f'Restante: {math.degrees(remaining):.2f}°',
            throttle_duration_sec=0.5)

        # Check if goal reached
        if remaining <= THRESHOLD_A:
            self._stop()
            self.goal_reached = True
            self.get_logger().info(
                f'Giro completado. Ángulo total: {math.degrees(angle_turned):.2f}°')
            return

        # Proportional angular speed — slows down as it approaches goal
        W = KW * remaining
        W = max(W_MIN, min(W_MAX, W))   # clamp between min and max

        # Positive W = turn left
        self._publish(w=W)


def main(args=None):
    rclpy.init(args=args)
    node = HuskyTurnLeft()
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
