#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from scipy.spatial.transform import Rotation as R

from cv_bridge import CvBridge
import cv2
import numpy as np

class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        self.configurar_parametros()

        # Publisher para comando de velocidade
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10,
        )

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam/colored_map', self.camera_callback, 10)

        # Utilizado para converter imagens ROS -> OpenCV
        self.bridge = CvBridge()

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.obstaculo_a_frente = False

    def configurar_parametros(self):
        # Parametros principais do comportamento atual. Eles podem ser
        # ajustados pelo launch sem alterar o codigo do controlador.
        self.declare_parameter('velocidade_linear', 0.1)
        self.declare_parameter('velocidade_angular_desvio', -0.3)
        self.declare_parameter('distancia_obstaculo', 0.5)
        self.declare_parameter('angulo_frontal_graus', 30.0)

        self.velocidade_linear = float(
            self.get_parameter('velocidade_linear').value
        )
        self.velocidade_angular_desvio = float(
            self.get_parameter('velocidade_angular_desvio').value
        )
        self.distancia_obstaculo = float(
            self.get_parameter('distancia_obstaculo').value
        )
        self.angulo_frontal_graus = float(
            self.get_parameter('angulo_frontal_graus').value
        )
        self.limite_frontal = math.radians(self.angulo_frontal_graus)

    def scan_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        distancias = []

        for indice, distancia in enumerate(msg.ranges):
            angulo = msg.angle_min + indice * msg.angle_increment
            angulo = math.atan2(math.sin(angulo), math.cos(angulo))

            leitura_valida = (
                math.isfinite(distancia)
                and msg.range_min <= distancia <= msg.range_max
            )
            if abs(angulo) <= self.limite_frontal and leitura_valida:
                distancias.append(distancia)

        distancia_frontal = min(distancias, default=math.inf)
        self.obstaculo_a_frente = distancia_frontal < self.distancia_obstaculo

        if self.obstaculo_a_frente:
            self.get_logger().info(
                f'Obstáculo detectado a {distancia_frontal:.2f} m à frente'
            )

    def imu_callback(self, msg: Imu):
        # # Extraindo o quaternion da mensagem
        # orientation_q = msg.orientation
        # quat = [
        #     orientation_q.x,
        #     orientation_q.y,
        #     orientation_q.z,
        #     orientation_q.w
        # ]

        # # Conversão para Euler usando SciPy
        # r = R.from_quat(quat)
        # roll, pitch, yaw = r.as_euler('xyz', degrees=True)

        # # Exibindo resultados
        # self.get_logger().info('IMU Data Received:')
        # self.get_logger().info(
        #     f'Orientation (Euler): Roll={roll:.2f}°, '
        #     f'Pitch={pitch:.2f}°, Yaw={yaw:.2f}°'
        # )
        # self.get_logger().info(
        #     f'Angular velocity: [{msg.angular_velocity.x:.2f}, '
        #     f'{msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}] rad/s'
        # )
        # self.get_logger().info(
        #     f'Linear acceleration: [{msg.linear_acceleration.x:.2f}, '
        #     f'{msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}] m/s²'
        # )
        pass

    def odom_callback(self, msg: Odometry):
        # Mensagens de Odometria das rodas!
        pass

    def camera_callback(self, msg: Image):
        # Converte mensagem ROS para imagem OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Define a cor-alvo em BGR
        target_color = np.array([171, 242, 0])  # OBS: OpenCV usa BGR

        # Cria máscara para cor exata
        mask = cv2.inRange(frame, target_color, target_color)

        # # Mostra a máscara em uma janela para debug
        # cv2.imshow('Mascara de Blobs #00f2ab', mask)
        # cv2.waitKey(1)  # Tempo mínimo para a janela atualizar (1 ms)

        # Detecta contornos (blobs)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Conta e localiza blobs
        self.get_logger().info(f'{len(contours)} blob(s) encontrados com cor #00f2ab:')
        for i, cnt in enumerate(contours):
            M = cv2.moments(cnt)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                self.get_logger().info(f'  Blob {i+1}: posição (x={cx}, y={cy})')

    def move_robot(self):
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()

        if not self.obstaculo_a_frente:
            cmd_vel.twist.linear.x = self.velocidade_linear  # Move para frente
        else:
            # Gira em torno do proprio eixo para procurar caminho livre.
            cmd_vel.twist.angular.z = self.velocidade_angular_desvio

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
