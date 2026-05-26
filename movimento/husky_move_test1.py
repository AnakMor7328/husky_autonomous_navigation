import rclpy
import math
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

# Físicos (Husky A200)
WHEEL_RADIUS = 0.155   # [m]
WHEEL_BASE   = 0.555   # [m] — track (left to right)

# Control lineal P
KV           = 0.20

# Saturaciones
V_MAX        = 0.3     # [m/s] — kept low if/for first test
V_MIN        = 0.155    # [m/s] — dead zone

# Criterio de llegada
THRESHOLD_D  = 0.05    # [m] — stop when within 5cm of goal

# Goal: move forward 0.5 meters
GOAL_X       = 0.5     # [m] - 1m es ~igual a 3 azulejos
GOAL_Y       = 0.0     # [m]

# Nodo
class HuskyMoveForward(Node):

    def __init__(self):
        super().__init__('husky_move_forward')

        # ── Posición inicial (se actualiza con odom) ───────────────
        self.x  = None   # This will be set on first odom message
        self.y  = None
        self.th = None

        self.start_x = None
        self.start_y = None

        self.goal_reached = False

        # ── Subscripción a odometría filtrada ─────────────────────
        self.create_subscription(
            Odometry,
            '/a200_1075/platform/odom/filtered',
            self._odom_cb,
            qos.qos_profile_sensor_data)

        # ── Publicador de velocidad (con TwistStamped) ─────────────────
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/a200_1075/cmd_vel',
            10)

        # ── Loop a 20 Hz (Husky tiene un rango de 10hz a 50hz) ────────── 
        self.create_timer(0.05, self._loop)
        self.get_logger().info('Nodo iniciado. Esperando odometría...')

    # ── Callback odometría ────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny_cosp, cosy_cosp)

        # Save starting position on first message
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info(
                f'Posición inicial: ({self.start_x:.3f}, {self.start_y:.3f})')


    # ── Publicar velocidad ────────────────────────────────────────

    def _publish(self, v: float, w: float = 0.0):
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
        if self.x is None:
            self.get_logger().info('Esperando odometría...', once=True)
            return

        # Already done
        if self.goal_reached:
            self._stop()
            return

        # Distance travelled from start
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        distance_travelled = math.sqrt(dx**2 + dy**2)

        # Distance remaining to goal
        remaining = GOAL_X - distance_travelled

        self.get_logger().info(
            f'Distancia recorrida: {distance_travelled:.3f} m | '
            f'Restante: {remaining:.3f} m',
            throttle_duration_sec=0.5)

        # Check if goal reached
        if remaining <= THRESHOLD_D:
            self._stop()
            self.goal_reached = True
            self.get_logger().info(
                f'Meta alcanzada. Distancia total: {distance_travelled:.3f} m')
            return

        # Proportional speed — slows down as it approaches goal
        V = KV * remaining
        V = max(V_MIN, min(V_MAX, V))   # clamp between min and max

        self._publish(V)

def main(args=None):
    rclpy.init(args=args)
    node = HuskyMoveForward()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__': # Necessario si estas utilizando nodos 
    main()
