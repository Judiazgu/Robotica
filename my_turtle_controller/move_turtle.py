import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, TeleportRelative
from turtlesim.msg import Pose
import threading
import time
import sys
import termios
import tty
import math

def get_key():
    #Captura una tecla del teclado, incluyendo las flechas de dirección.
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class JTrajectoryTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Configuración de los clientes para los servicios
        self.clear_client = self.create_client(Empty, '/clear')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.teleport_relative_client = self.create_client(TeleportRelative, '/turtle1/teleport_relative')

        # Espera a que los servicios estén disponibles
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /clear...')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_absolute...')
        while not self.teleport_relative_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /teleport_relative...')

        # Subscripción a la posición actual de la tortuga
        self.pose = None
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Hilo que escucha continuamente el teclado
        thread = threading.Thread(target=self.keyboard_listener)
        thread.daemon = True
        thread.start()

    def keyboard_listener(self):
        # Escucha eventos del teclado y ejecuta acciones según la tecla presionada.
        self.get_logger().info("Presiona 'j', 'd', 'g', etc. o usa flechas ↑ ↓ ← →. Ctrl+C para salir.")
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
            elif key.lower() == 'c':
                self.prepare_for_c()
                self.draw_c()
            elif key.lower() == 'f':
                self.prepare_for_f(math.pi / 2)
                self.draw_f()
            elif key.lower() == 'm':
                self.prepare_for_m(math.pi / 2)
                self.draw_m()
            elif key.lower() == 'b':
                self.prepare_for_b(math.pi / 2)
                self.draw_b()
            elif key == '\x1b[A':
                self.move_turtle(1.0, 0.0, 0.2)
            elif key == '\x1b[B':
                self.move_turtle(-1.0, 0.0, 0.2)
            elif key == '\x1b[C':
                self.move_turtle(0.0, -1.5, 0.2)
            elif key == '\x1b[D':
                self.move_turtle(0.0, 1.5, 0.2)

    def move_turtle(self, linear_x, angular_z, duration):
        # Mueve la tortuga con velocidad linear y angular durante un tiempo determinado.
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.get_logger().info(f"Moviendo tortuga: linear_x={linear_x}, angular_z={angular_z}")
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.01)
        self._stop_turtle()

    def prepare_for_j(self):
        # Posiciona la tortuga para comenzar a trazar la letra J y limpia la pantalla.
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 5.5
        request.theta = -math.pi / 2
        self.teleport_client.call_async(request)
        time.sleep(0.5)
        self.clear_client.call_async(Empty.Request())

    def prepare_for_d(self, ang):
        # Gira la tortuga hacia el ángulo dado desde su posición actual.
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return
        request = TeleportAbsolute.Request()
        request.x = self.pose.x
        request.y = self.pose.y
        request.theta = ang
        self.teleport_client.call_async(request)
        time.sleep(0.5)

    def draw_j(self):
        # Traza la letra J mediante movimientos curvos y lineales.
        self.get_logger().info("Trazando la letra J...")
        msg = Twist()
        self._teleport_relative(3.0, 0.0)
        time.sleep(0.2)
        msg.linear.x = 1.5
        msg.angular.z = -1.5
        self._publish_for_duration(msg, 2.0)
        self._stop_turtle()
        self.get_logger().info("¡Letra J completada!")

    def draw_d(self):
        # Traza la letra D con línea recta y arco semicircular derecho.
        self.get_logger().info("Trazando la letra D...")
        msg = Twist()
        self._teleport_relative(3.0, 0.0)
        time.sleep(0.2)
        self.prepare_for_d(0.0)
        self._teleport_relative(0.5, 0.0)
        time.sleep(0.2)
        msg.linear.x = 2.0
        msg.angular.z = 1.2
        self._publish_for_duration(msg, 3.5)
        self._stop_turtle()
        self.get_logger().info("¡Letra D completada!")

    def draw_g(self):
        # Traza la letra G combinando un arco y dos líneas.
        self.get_logger().info("Trazando la letra G...")
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.2
        self._publish_for_duration(msg, 3.5)
        self._stop_turtle()
        self.get_logger().info("¡Letra G completada!")
        self.prepare_for_d(math.pi / 2)
        self._teleport_relative(1.0, 0.0)
        time.sleep(0.2)
        self.prepare_for_d(math.pi)
        self._teleport_relative(1.0, 0.0)
        time.sleep(0.2)

    def prepare_for_c(self):
        # Posiciona la tortuga para comenzar a trazar la letra C y limpia la pantalla.
        request = TeleportAbsolute.Request()
        request.x = 6.5
        request.y = 7.5
        request.theta = 0.0
        self.teleport_client.call_async(request)
        time.sleep(0.5)
        self.clear_client.call_async(Empty.Request())

    def rotate(self, ang):
        # Gira la tortuga a un ángulo específico sin moverla de posición.
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return
        request = TeleportAbsolute.Request()
        request.x = self.pose.x
        request.y = self.pose.y
        request.theta = ang
        self.teleport_client.call_async(request)
        time.sleep(0.5)

    def prepare_for_f(self, ang):
        # Prepara la posición y orientación para trazar la letra F.
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 4.5
        request.theta = ang
        self.teleport_client.call_async(request)
        time.sleep(0.5)
        self.clear_client.call_async(Empty.Request())

    def prepare_for_m(self, ang):
        # Prepara la posición y orientación para trazar la letra M.
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return
        request = TeleportAbsolute.Request()
        request.x = 4.5
        request.y = 4.5
        request.theta = ang
        self.teleport_client.call_async(request)
        time.sleep(0.5)
        self.clear_client.call_async(Empty.Request())

    def prepare_for_b(self, ang):
        # Prepara la posición y orientación para trazar la letra B.
        if self.pose is None:
            self.get_logger().warn("No se ha recibido la posición del turtle aún.")
            return
        request = TeleportAbsolute.Request()
        request.x = 5.5
        request.y = 3.5
        request.theta = ang
        self.teleport_client.call_async(request)
        time.sleep(0.5)
        self.clear_client.call_async(Empty.Request())

    def draw_c(self):
        # Traza la letra C como un arco hacia la izquierda.
        self.get_logger().info("Trazando la letra C...")
        msg = Twist()
        time.sleep(0.2)
        msg.linear.x = -2.0
        msg.angular.z = 1.0
        self._publish_for_duration(msg, math.pi)
        self._stop_turtle()
        self.get_logger().info("¡Letra C completada!")

    def draw_f(self):
        # Traza la letra F con una línea vertical y dos horizontales.
        self.get_logger().info("Trazando la letra F...")
        self._teleport_relative(3.0, 0.0)
        time.sleep(0.2)
        self.rotate(0.0)
        self._teleport_relative(1.5, 0.0)
        time.sleep(0.2)
        self.rotate(-math.pi)
        self._teleport_relative(1.5, 0.0)
        time.sleep(0.2)
        self.rotate((3 * math.pi) / 2)
        self._teleport_relative(1.25, 0.0)
        time.sleep(0.2)
        self.rotate(0.0)
        self._teleport_relative(1.5, 0.0)
        time.sleep(0.2)
        self._stop_turtle()
        self.get_logger().info("¡Letra F completada!")

    def draw_m(self):
        #Traza la letra M con líneas verticales y diagonales.
        self.get_logger().info("Trazando la letra M...")
        self._teleport_relative(3.0, 0.0)
        time.sleep(0.2)
        self.rotate(-math.pi / 6)
        self._teleport_relative(1.5, 0.0)
        time.sleep(0.2)
        self.rotate(math.pi / 6)
        self._teleport_relative(1.5, 0.0)
        time.sleep(0.2)
        self.rotate(-math.pi / 2)
        self._teleport_relative(3.0, 0.0)
        time.sleep(0.2)
        self._stop_turtle()
        self.get_logger().info("¡Letra M completada!")

    def draw_b(self):
        #Traza la letra B con una línea vertical y dos arcos cerrados.
        self.get_logger().info("Trazando la letra B...")
        msg = Twist()
        self._teleport_relative(4.0, 0.0)
        time.sleep(0.2)
        self.rotate(0.0)
        msg.linear.x = 1.0
        msg.angular.z = -1.0
        self._publish_for_duration(msg, math.pi)
        self.rotate(0.0)
        self._publish_for_duration(msg, math.pi)
        self._stop_turtle()
        self.get_logger().info("¡Letra B completada!")

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
        #Envía un mensaje de velocidad cero para detener la tortuga."""
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def pose_callback(self, msg):
        #Actualiza la posición actual de la tortuga.
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
