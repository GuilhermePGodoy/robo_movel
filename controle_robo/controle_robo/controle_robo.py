#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from scipy.spatial.transform import Rotation as R

from controle_robo.maquina_estados import MaquinaEstadosMissao
from controle_robo.modelos_missao import DeteccaoBandeira


class ControleRobo(Node):
    """No ROS que junta sensores, atuadores e a logica da missao."""

    def __init__(self):
        super().__init__('controle_robo')

        self.configurar_parametros()

        # Usado por varios callbacks para evitar spam no terminal.
        self.ultimo_log_por_chave = {}

        # Os callbacks atualizam essas leituras. A maquina de estados apenas
        # consulta os valores mais recentes e decide o proximo comando.
        self.deteccao_bandeira = DeteccaoBandeira()
        self.ultimo_instante_bandeira = None
        self.ultimo_erro_bandeira = 0.0

        self.obstaculo_a_frente = False
        self.distancia_frontal = math.inf
        self.distancia_esquerda = math.inf
        self.distancia_direita = math.inf
        self.direcao_desvio = 1.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.cmd_vel_pub = self.create_publisher(
            TwistStamped,
            '/diff_drive_base_controller/cmd_vel',
            10,
        )
        self.garra_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10,
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            '/bandeira_azul/deteccao',
            self.deteccao_bandeira_callback,
            10,
        )

        self.maquina_estados = MaquinaEstadosMissao(self)
        self.timer = self.create_timer(0.1, self.maquina_estados.executar)

        self.get_logger().info(
            'Controle iniciado em EXPLORANDO: procurando a bandeira azul.'
        )

    def configurar_parametros(self):
        """Declara e le os parametros usados pelo controle da missao."""

        self.declare_parameter('velocidade_linear', 0.1)
        self.declare_parameter('velocidade_angular_desvio', -0.3)
        self.declare_parameter('distancia_obstaculo', 0.5)
        self.declare_parameter('angulo_frontal_graus', 30.0)
        self.declare_parameter('velocidade_exploracao', 0.08)
        self.declare_parameter('velocidade_posicionamento', 0.04)
        self.declare_parameter('distancia_velocidade_livre', 1.8)
        self.declare_parameter('fator_velocidade_livre', 1.35)
        self.declare_parameter('fator_velocidade_proxima', 0.45)
        self.declare_parameter('amplitude_varredura_camera', 0.18)
        self.declare_parameter('velocidade_giro_busca', 0.25)
        self.declare_parameter('ganho_angular_bandeira', 0.9)
        self.declare_parameter('erro_alinhamento_bandeira', 0.12)
        self.declare_parameter('area_posicionamento_bandeira', 0.02)
        self.declare_parameter('area_coleta_bandeira', 0.07)
        self.declare_parameter('distancia_posicionamento', 0.9)
        self.declare_parameter('distancia_coleta', 0.45)
        self.declare_parameter('tempo_perda_bandeira', 1.0)
        self.declare_parameter('tempo_reexploracao', 3.0)
        self.declare_parameter('tempo_minimo_desvio', 0.8)
        self.declare_parameter('habilitar_garra', True)
        self.declare_parameter('garra_extensao_aberta', 0.0)
        self.declare_parameter('garra_direita_aberta', -0.06)
        self.declare_parameter('garra_esquerda_aberta', 0.06)
        self.declare_parameter('garra_extensao_captura', 0.0)
        self.declare_parameter('garra_direita_captura', 0.0)
        self.declare_parameter('garra_esquerda_captura', 0.0)

        self.velocidade_linear = float(
            self.get_parameter('velocidade_linear').value
        )
        self.velocidade_angular_desvio = abs(float(
            self.get_parameter('velocidade_angular_desvio').value
        ))
        self.distancia_obstaculo = float(
            self.get_parameter('distancia_obstaculo').value
        )
        self.angulo_frontal_graus = float(
            self.get_parameter('angulo_frontal_graus').value
        )
        self.limite_frontal = math.radians(self.angulo_frontal_graus)
        self.velocidade_exploracao = float(
            self.get_parameter('velocidade_exploracao').value
        )
        self.velocidade_posicionamento = float(
            self.get_parameter('velocidade_posicionamento').value
        )
        self.distancia_velocidade_livre = max(
            self.distancia_obstaculo + 0.05,
            float(self.get_parameter('distancia_velocidade_livre').value),
        )
        self.fator_velocidade_livre = max(
            1.0,
            float(self.get_parameter('fator_velocidade_livre').value),
        )
        self.fator_velocidade_proxima = self.limitar(
            float(self.get_parameter('fator_velocidade_proxima').value),
            0.05,
            1.0,
        )
        self.amplitude_varredura_camera = abs(float(
            self.get_parameter('amplitude_varredura_camera').value
        ))
        self.velocidade_giro_busca = abs(float(
            self.get_parameter('velocidade_giro_busca').value
        ))
        self.ganho_angular_bandeira = float(
            self.get_parameter('ganho_angular_bandeira').value
        )
        self.erro_alinhamento_bandeira = float(
            self.get_parameter('erro_alinhamento_bandeira').value
        )
        self.area_posicionamento_bandeira = float(
            self.get_parameter('area_posicionamento_bandeira').value
        )
        self.area_coleta_bandeira = float(
            self.get_parameter('area_coleta_bandeira').value
        )
        self.distancia_posicionamento = float(
            self.get_parameter('distancia_posicionamento').value
        )
        self.distancia_coleta = float(
            self.get_parameter('distancia_coleta').value
        )
        self.tempo_perda_bandeira = float(
            self.get_parameter('tempo_perda_bandeira').value
        )
        self.tempo_reexploracao = float(
            self.get_parameter('tempo_reexploracao').value
        )
        self.tempo_minimo_desvio = float(
            self.get_parameter('tempo_minimo_desvio').value
        )
        self.habilitar_garra = bool(
            self.get_parameter('habilitar_garra').value
        )
        self.comando_garra_aberta = [
            float(self.get_parameter('garra_extensao_aberta').value),
            float(self.get_parameter('garra_direita_aberta').value),
            float(self.get_parameter('garra_esquerda_aberta').value),
        ]
        self.comando_garra_captura = [
            float(self.get_parameter('garra_extensao_captura').value),
            float(self.get_parameter('garra_direita_captura').value),
            float(self.get_parameter('garra_esquerda_captura').value),
        ]

    def scan_callback(self, msg: LaserScan):
        """Separa a leitura do laser em frente, esquerda e direita."""

        if not msg.ranges:
            return

        distancias_frente = []
        distancias_esquerda = []
        distancias_direita = []

        for indice, distancia in enumerate(msg.ranges):
            angulo = msg.angle_min + indice * msg.angle_increment
            angulo = math.atan2(math.sin(angulo), math.cos(angulo))

            leitura_valida = (
                math.isfinite(distancia)
                and msg.range_min <= distancia <= msg.range_max
            )
            if abs(angulo) <= self.limite_frontal and leitura_valida:
                distancias_frente.append(distancia)
            elif (
                self.limite_frontal < angulo <= math.radians(90)
                and leitura_valida
            ):
                distancias_esquerda.append(distancia)
            elif (
                -math.radians(90) <= angulo < -self.limite_frontal
                and leitura_valida
            ):
                distancias_direita.append(distancia)

        self.distancia_frontal = min(distancias_frente, default=math.inf)
        self.distancia_esquerda = min(distancias_esquerda, default=math.inf)
        self.distancia_direita = min(distancias_direita, default=math.inf)
        self.obstaculo_a_frente = (
            self.distancia_frontal < self.distancia_obstaculo
        )

        # Z angular positivo gira para a esquerda; negativo gira para a direita.
        self.direcao_desvio = (
            1.0 if self.distancia_esquerda >= self.distancia_direita else -1.0
        )

        if self.obstaculo_a_frente:
            self.log_periodico(
                'scan_obstaculo',
                (
                    f'LIDAR: obstaculo a {self.distancia_frontal:.2f} m; '
                    f'esq={self.formatar_distancia(self.distancia_esquerda)}, '
                    f'dir={self.formatar_distancia(self.distancia_direita)}.'
                ),
                periodo=1.0,
            )

    def imu_callback(self, msg: Imu):
        # A IMU fica assinada para debug e extensoes futuras. Nesta versao, a
        # orientacao usada no controle vem da odometria ground truth.
        _ = msg

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        quat = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        self.yaw = R.from_quat(quat).as_euler('xyz', degrees=False)[2]

    def deteccao_bandeira_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 10:
            self.log_periodico(
                'deteccao_invalida',
                'Deteccao: mensagem incompleta recebida do detector.',
                periodo=2.0,
                nivel='warn',
            )
            return

        if msg.data[0] < 0.5:
            return

        self.deteccao_bandeira = DeteccaoBandeira(
            visivel=True,
            erro_x=float(msg.data[1]),
            area_relativa=float(msg.data[2]),
            area=float(msg.data[3]),
            centro_x=float(msg.data[4]),
            centro_y=float(msg.data[5]),
            largura=int(msg.data[6]),
            altura=int(msg.data[7]),
        )

        # Tempo monotonic evita surpresas quando o Gazebo pausa ou reinicia o
        # relogio simulado durante os testes.
        self.ultimo_instante_bandeira = time.monotonic()
        self.ultimo_erro_bandeira = self.deteccao_bandeira.erro_x

        self.log_periodico(
            'deteccao_bandeira',
            (
                'Deteccao: alvo visual recebido '
                f'erro={self.deteccao_bandeira.erro_x:+.2f}, '
                f'area={self.deteccao_bandeira.area_relativa:.3f}.'
            ),
            periodo=1.0,
        )

    def publicar_garra(self, posicoes):
        comando = Float64MultiArray()
        comando.data = [float(posicao) for posicao in posicoes]
        self.garra_pub.publish(comando)

    def publicar_velocidade(self, linear: float, angular: float):
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.twist.linear.x = float(linear)
        cmd_vel.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd_vel)

    def log_periodico(
        self,
        chave: str,
        mensagem: str,
        periodo: float = 1.0,
        nivel: str = 'info',
    ):
        agora = time.monotonic()
        ultimo_log = self.ultimo_log_por_chave.get(chave, -math.inf)
        if agora - ultimo_log < periodo:
            return

        self.ultimo_log_por_chave[chave] = agora
        logger = self.get_logger()
        if nivel == 'warn':
            logger.warn(mensagem)
        else:
            logger.info(mensagem)

    @staticmethod
    def limitar(valor: float, minimo: float, maximo: float):
        return max(minimo, min(maximo, valor))

    @staticmethod
    def formatar_distancia(distancia: float):
        if math.isinf(distancia):
            return 'inf'
        return f'{distancia:.2f}m'


def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
