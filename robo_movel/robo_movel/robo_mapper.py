#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from scipy.spatial.transform import Rotation as R

import numpy as np

# Necessario para publicar o frame map:
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class RoboMapper(Node):

    def __init__(self):
        super().__init__('robo_mapper')

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Pose, '/model/prm_robot/pose', self.odom_callback, 10)

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.5, self.atualiza_mapa)

        # Estado atual do robo:
        self.x = 0
        self.y = 0
        self.heading = 0
        self.ultimo_scan = None

        # Atributos de configuração do mapa
        # Parametros do mapa. Com 100 celulas a 25 cm, cobrimos 25 x 25 m,
        # suficiente para a arena completa e para o spawn em x=-8.
        self.grid_size = 100
        self.resolution = 0.25  # 25 cm por célula

        # Matriz do mapa (-1 = desconhecido)
        self.grid_map = -np.ones((self.grid_size, self.grid_size), dtype=np.int8)

        # Publisher do mapa
        self.map_pub = self.create_publisher(OccupancyGrid, '/grid_map', 10)

        # Publicando o frame map para vizualização no RVis
        # Utilizar o comando: ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
        # ou o código abaixo:
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = "map"
        static_tf.child_frame_id = "odom_gt"
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.w = 1.0  # identidade (Quaternions!!)
        self.tf_static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(
            'Mapper iniciado: publicando /grid_map com pose e LIDAR.'
        )


    def scan_callback(self, msg: LaserScan):
        self.ultimo_scan = msg

    def odom_callback(self, msg: Pose):
        # Extrair posição
        self.x = msg.position.x
        self.y = msg.position.y

        # Extrair orientação (quaternion)
        orientation_q = msg.orientation
        quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        # Converter de quaternion para Euler (roll, pitch, yaw)
        r = R.from_quat(quat)
        euler = r.as_euler('xyz', degrees=False)

        # Armazenar heading (Z - yaw)
        self.heading = euler[2]

    def world_to_grid(self, x, y):
        origin_offset = self.grid_size * self.resolution / 2
        gx = int((x + origin_offset) / self.resolution)
        gy = int((y + origin_offset) / self.resolution)
        return gx, gy

    def atualiza_mapa(self):
        if self.ultimo_scan is not None:
            self.integrar_lidar(self.ultimo_scan)
        else:
            self.get_logger().warn(
                'Mapper ainda nao recebeu /scan; publicando apenas pose do robo.',
                throttle_duration_sec=3.0,
            )

        # Marcar a posicao atual do robo no mapa por ultimo para ela ficar
        # visivel mesmo quando um raio do LIDAR passa pela mesma celula.
        gx, gy = self.world_to_grid(self.x, self.y)
        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.grid_map[gy, gx] = 100

        # Publicar o mapa
        self.publish_occupancy_grid()

    def integrar_lidar(self, scan: LaserScan):
        origem = self.world_to_grid(self.x, self.y)
        if not self.celula_valida(*origem):
            return

        # Usa no maximo 180 raios por atualizacao para manter o mapa leve.
        passo = max(1, len(scan.ranges) // 180)

        for indice in range(0, len(scan.ranges), passo):
            distancia = scan.ranges[indice]
            leitura_valida = (
                math.isfinite(distancia)
                and scan.range_min <= distancia <= scan.range_max
            )

            if leitura_valida:
                alcance = distancia
                marca_obstaculo = distancia < scan.range_max * 0.98
            else:
                alcance = scan.range_max
                marca_obstaculo = False

            angulo_laser = scan.angle_min + indice * scan.angle_increment
            angulo_mundo = self.heading + angulo_laser
            fim_x = self.x + alcance * math.cos(angulo_mundo)
            fim_y = self.y + alcance * math.sin(angulo_mundo)
            destino = self.world_to_grid(fim_x, fim_y)

            self.marcar_raio_livre(origem, destino, marca_obstaculo)

    def marcar_raio_livre(self, origem, destino, marca_obstaculo):
        celulas = self.bresenham(origem[0], origem[1], destino[0], destino[1])
        if not celulas:
            return

        for gx, gy in celulas[:-1]:
            if self.celula_valida(gx, gy):
                self.grid_map[gy, gx] = 0

        fim_x, fim_y = celulas[-1]
        if marca_obstaculo and self.celula_valida(fim_x, fim_y):
            self.grid_map[fim_y, fim_x] = 100

    def bresenham(self, x0, y0, x1, y1):
        celulas = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        erro = dx + dy

        x_atual = x0
        y_atual = y0
        while True:
            celulas.append((x_atual, y_atual))
            if x_atual == x1 and y_atual == y1:
                break

            erro2 = 2 * erro
            if erro2 >= dy:
                erro += dy
                x_atual += sx
            if erro2 <= dx:
                erro += dx
                y_atual += sy

            if len(celulas) > self.grid_size * 2:
                break

        return celulas

    def celula_valida(self, gx, gy):
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def publish_occupancy_grid(self):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = "map"

        # Metadados do mapa
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.grid_size
        grid_msg.info.height = self.grid_size

        # Origem do mapa (canto inferior esquerdo do grid no mundo)
        origin = Pose()
        origin.position.x = - (self.grid_size * self.resolution) / 2
        origin.position.y = - (self.grid_size * self.resolution) / 2
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        grid_msg.info.origin = origin

        # Convertendo numpy array para lista 1D em row-major
        grid_msg.data = self.grid_map.flatten().tolist()

        # Publicar
        self.map_pub.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoboMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
