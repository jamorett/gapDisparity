import rclpy  # Cliente de ROS2 en Python
from rclpy.node import Node  # Clase base para crear nodos ROS2
from sensor_msgs.msg import LaserScan  # Mensaje para datos LiDAR
from ackermann_msgs.msg import AckermannDriveStamped  # Mensaje para controlar dirección y velocidad
from nav_msgs.msg import Odometry  # Mensaje de odometría para posición
import numpy as np  # Librería numérica para manejar arrays
import math  # Funciones matemáticas como atan2, sqrt, etc.
import time  # Medición de tiempo para cronómetro de vueltas

# === Parámetros configurables ===
CAR_WIDTH = 0.6                  # Ancho del vehículo en metros
SAFETY_MARGIN = 1.6              # Margen de seguridad lateral

MAX_SPEED = 10.0                 # Velocidad máxima deseada [m/s]
MIN_STOP_DISTANCE = 0.4          # Distancia mínima al frente para detenerse
MAX_ACCEL_DISTANCE = 7.0         # Distancia para alcanzar velocidad máxima
MAX_DECEL = 3.0                  # Desaceleración máxima [m/s²] para frenado seguro

# Parámetros del LiDAR
LIDAR_FOV_RAD = 4.7              # Campo visual del LiDAR en radianes (~269°)
LIDAR_RESOLUTION = 1080          # Número de rayos LiDAR

DISPARITY_THRESHOLD = 1.5        # Umbral de disparidad para detectar obstáculos
STEERING_ALPHA = 0.75            # Suavizado de dirección (filtro exponencial)
SIDE_CHECK_ARC_DEG = 9.0         # Ángulo para verificar colisiones laterales
LAP_DETECTION_RADIUS = 1.0       # Radio para detectar paso por punto de origen (para contar vueltas)

class DisparityExtenderNode(Node):
    def __init__(self):
        super().__init__('disparity_extender')  # Inicializa el nodo con nombre

        # Suscripciones y publicación
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        # Precalculo de los ángulos de cada rayo del LiDAR
        self.angle_increment = LIDAR_FOV_RAD / LIDAR_RESOLUTION
        self.lidar_angles = np.linspace(-LIDAR_FOV_RAD/2, LIDAR_FOV_RAD/2, LIDAR_RESOLUTION)

        # Estado interno del nodo
        self.prev_steering = 0.0  # Último ángulo de giro publicado
        self.track_origin = None  # Posición de partida para contar vueltas
        self.crossed = False  # Bandera para evitar múltiples conteos por vuelta
        self.lap_started = False  # Ignora primer cruce por el origen
        self.lap_count = 0
        self.lap_start_time = time.time()  # Marca de tiempo del inicio de la vuelta

        self.get_logger().info('Disparity Extender Node initialized.')

    def scan_callback(self, msg: LaserScan):
        # Limpia datos del LiDAR, reemplaza 'inf' por valor máximo
        ranges = np.array(msg.ranges, dtype=np.float32)
        ranges[np.isinf(ranges)] = msg.range_max

        # Aplica extensión de obstáculos
        extended = self.disparity_extender(ranges)

        # Aplica sesgo frontal y busca dirección más despejada
        bias = np.cos(self.lidar_angles)
        target_idx = np.argmax(extended * bias)
        target_angle = self.lidar_angles[target_idx]

        # Corrige posibles colisiones laterales
        adjusted = self.avoid_side_collision(ranges, target_angle)

        # Suaviza el ángulo de dirección
        steer = STEERING_ALPHA * adjusted + (1 - STEERING_ALPHA) * self.prev_steering
        max_angle = self.get_max_steering_angle()
        steer = np.clip(steer, -max_angle, max_angle)
        self.prev_steering = steer

        # Calcula velocidad considerando obstáculos y frenado
        speed = self.compute_speed(extended, steer)

        # Publica comando de conducción
        drive_msg = AckermannDriveStamped()
        drive_msg.header = msg.header
        drive_msg.drive.steering_angle = float(steer)
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, msg: Odometry):
        # Obtiene posición actual del vehículo
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pos = np.array([x, y])

        # Guarda el punto de partida
        if self.track_origin is None:
            self.track_origin = pos
            return

        # Calcula distancia al origen
        dist = np.linalg.norm(pos - self.track_origin)

        # Verifica si está dentro del radio para contar vuelta
        if dist < LAP_DETECTION_RADIUS:
            if not self.crossed:
                if self.lap_started:
                    self.lap_count += 1
                    now = time.time()
                    lap_time = now - self.lap_start_time
                    self.get_logger().info(f"Lap {self.lap_count} completed in {lap_time:.2f} s")
                    self.lap_start_time = now
                else:
                    self.lap_started = True  # Ignora la primera detección
                self.crossed = True
        else:
            self.crossed = False

    def disparity_extender(self, ranges: np.ndarray) -> np.ndarray:
        # Crea una copia para modificar
        ext = ranges.copy()
        for i in range(1, len(ranges)):
            delta = abs(ranges[i] - ranges[i-1])
            # Detecta una disparidad
            if delta > DISPARITY_THRESHOLD:
                closer = min(ranges[i], ranges[i-1])
                theta = math.atan2((CAR_WIDTH/2 + SAFETY_MARGIN), closer)
                mask = int(theta / self.angle_increment)
                # Enmascara hacia adelante o atrás según dirección
                if ranges[i-1] < ranges[i]:
                    end = min(len(ext), i + mask)
                    ext[i:end] = closer
                else:
                    start = max(0, i - mask)
                    ext[start:i] = closer
        return ext

    def compute_speed(self, ranges: np.ndarray, steer: float) -> float:
        front = ranges[LIDAR_RESOLUTION//2]
        if front <= MIN_STOP_DISTANCE:
            base = 0.0
        else:
            desired = (front - MIN_STOP_DISTANCE) / max(1e-3, (MAX_ACCEL_DISTANCE - MIN_STOP_DISTANCE))
            base = min(MAX_SPEED, desired * MAX_SPEED)

        # Reduce velocidad en curvas
        turn_factor = max(0.2, 1.0 - abs(steer) / self.get_max_steering_angle())
        speed_turn = base * turn_factor

        # Limita según distancia de frenado segura
        braking_v = math.sqrt(2 * MAX_DECEL * max(0.0, front - MIN_STOP_DISTANCE))
        return min(speed_turn, braking_v)

    def avoid_side_collision(self, ranges: np.ndarray, angle: float) -> float:
        # Verifica zonas laterales ±90° para evitar giros inseguros
        clearance = (CAR_WIDTH/2) + SAFETY_MARGIN
        center = LIDAR_RESOLUTION//2
        idx90 = int(math.radians(90) / self.angle_increment)
        arc = int(math.radians(SIDE_CHECK_ARC_DEG) / (self.angle_increment * 2))

        right = np.min(ranges[max(0, center-idx90-arc):center-idx90+arc])
        left  = np.min(ranges[center+idx90-arc: min(len(ranges), center+idx90+arc)])

        if angle > 0 and right < clearance:
            return angle * 0.3  # Reduce giro a la derecha
        if angle < 0 and left < clearance:
            return angle * 0.3  # Reduce giro a la izquierda
        return angle

    def get_max_steering_angle(self) -> float:
        # Último ángulo representa el máximo ángulo observable
        return float(self.lidar_angles[-1])


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
