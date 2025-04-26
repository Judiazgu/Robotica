import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import TeleportRelative
from turtlesim.msg import Pose
import threading
import time
import sys
import termios
import tty
import math

def get_key():
    """Lee una tecla sin esperar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class JTrajectoryTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Servicios
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        # Esperar que los servicios estén disponibles
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /clear...')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_absolute...')
        self.teleport_relative_client = self.create_client(TeleportRelative, '/turtle1/teleport_relative')
        while not self.teleport_relative_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_relative...')
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)


        thread = threading.Thread(target=self.keyboard_listener)
        thread.daemon = True
        thread.start()

    def keyboard_listener(self):
        self.get_logger().info("Presiona 'j' para trazar una J. Ctrl+C para salir.")
        while True:
            key = get_key()
            if key.lower() == 'j':
                self.prepare_for_j()
                self.draw_j()
            elif key.lower() == 'd':
                self.prepare_for_j()
                self.draw_d()
            elif key.lower() == 'g':
                self.prepare_for_j()
                self.prepare_for_d(math.pi)
                self.draw_g()

    def prepare_for_j(self):
        # Teletransportar al centro (x=5.5, y=5.5) y mirar hacia abajo (theta=-π/2)
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = -math.pi / 2
        self.teleport_client.call_async(request)
        time.sleep(0.5)  # Pequeña pausa para que el servicio se complete

        # Borrar trazos
        self.clear_client.call_async(Empty.Request())

    def prepare_for_d(self, ang):
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return

        request = TeleportAbsolute.Request()
        request.x = self.pose.x
        request.y = self.pose.y
        request.theta = ang  # Rotar hacia la derecha

        self.teleport_client.call_async(request)
        time.sleep(0.5)

    def draw_j(self):
        self.get_logger().info("Trazando la letra J...")

        msg = Twist()

        # Paso 1: Línea recta hacia abajo
        self._teleport_relative(3.0, 0.0)  # Mover 1.5 unidades hacia la derecha
        time.sleep(0.2)

        # Paso 2: Arco hacia la izquierda
        msg.linear.x = 1.5
        msg.angular.z = -1.5
        self._publish_for_duration(msg, 2.0)

        self._stop_turtle()
        self.get_logger().info("¡Letra J completada!")

    def draw_d(self):
        self.get_logger().info("Trazando la letra D...")
        msg = Twist()

        # Línea vertical hacia abajo
        self._teleport_relative(3.0, 0.0)  # Mover 1.5 unidades hacia la derecha
        time.sleep(0.2)

        # Arco hacia la derecha
        self.prepare_for_d(0.0)
        self._teleport_relative(0.5, 0.0)  # Mover 1.5 unidades hacia la derecha
        time.sleep(0.2)
        msg.linear.x = 2.0
        msg.angular.z = 1.2
        self._publish_for_duration(msg, 3.5)

        self._stop_turtle()
        self.get_logger().info("¡Letra D completada!")

    def draw_g(self):
        self.get_logger().info("Trazando la letra D...")
        msg = Twist()

        # Arco hacia la izquierda
        msg.linear.x = 2.0
        msg.angular.z = 1.2
        self._publish_for_duration(msg, 3.5)

        self._stop_turtle()
        self.get_logger().info("¡Letra D completada!")

        # VERTICAL LINE
        self.prepare_for_d(math.pi / 2)
        self._teleport_relative(1.0, 0.0)  # Mover 1.5 unidades hacia la derecha
        time.sleep(0.2)

        # HORIZONTAL LINE
        self.prepare_for_d(math.pi)
        self._teleport_relative(1.0, 0.0)  # Mover 1.5 unidades hacia la derecha
        time.sleep(0.2)

    def _teleport_relative(self, linear, angular):
        request = TeleportRelative.Request()
        request.linear = linear
        request.angular = angular
        self.teleport_relative_client.call_async(request)

    def _publish_for_duration(self, msg, duration_sec):
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            self.publisher_.publish(msg)
            time.sleep(0.1)

    def _stop_turtle(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def pose_callback(self, msg):
        self.pose = msg


def main(args=None):
    rclpy.init(args=args)
    node = JTrajectoryTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
