#!/usr/bin/env python3
from dataclasses import dataclass
from enum import Enum
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from scipy.spatial.transform import Rotation as R


class EstadoMissao(Enum):
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    NAVIGANDO_PARA_BANDEIRA = 'NAVIGANDO_PARA_BANDEIRA'
    DESVIANDO_OBSTACULO = 'DESVIANDO_OBSTACULO'
    REDETECTANDO_BANDEIRA = 'REDETECTANDO_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    CAPTURANDO_BANDEIRA = 'CAPTURANDO_BANDEIRA'


@dataclass
class DeteccaoBandeira:
    visivel: bool = False
    centro_x: float = 0.0
    centro_y: float = 0.0
    erro_x: float = 0.0
    area: float = 0.0
    area_relativa: float = 0.0
    largura: int = 0
    altura: int = 0


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo')

        self.configurar_parametros()

        # Estado interno da missao. A maquina de estados fica toda neste no,
        # pois ele e o responsavel por transformar percepcao em movimento.
        self.estado_atual = EstadoMissao.EXPLORANDO
        self.instante_inicio_estado = time.monotonic()
        self.ultimo_log_por_chave = {}

        # Percepcao da bandeira azul pela camera segmentada.
        self.deteccao_bandeira = DeteccaoBandeira()
        self.ultimo_instante_bandeira = None
        self.ultimo_erro_bandeira = 0.0
        self.garra_aberta = False
        self.garra_fechada = False

        # Percepcao de obstaculos pelo LIDAR.
        self.obstaculo_a_frente = False
        self.distancia_frontal = math.inf
        self.distancia_esquerda = math.inf
        self.distancia_direita = math.inf
        self.direcao_desvio = 1.0

        # Odometria usada apenas para logs de debug da missao.
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Publisher para comando de velocidade
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

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            '/bandeira_azul/deteccao',
            self.deteccao_bandeira_callback,
            10,
        )

        # Timer para enviar comandos continuamente
        self.timer = self.create_timer(0.1, self.executar_maquina_estados)

        self.get_logger().info(
            'Controle iniciado em EXPLORANDO: procurando a bandeira azul.'
        )

    def configurar_parametros(self):
        # Parametros principais da maquina de estados. Eles podem ser
        # ajustados pelo launch sem alterar o codigo do controlador.
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
            elif self.limite_frontal < angulo <= math.radians(90) and leitura_valida:
                distancias_esquerda.append(distancia)
            elif -math.radians(90) <= angulo < -self.limite_frontal and leitura_valida:
                distancias_direita.append(distancia)

        self.distancia_frontal = min(distancias_frente, default=math.inf)
        self.distancia_esquerda = min(distancias_esquerda, default=math.inf)
        self.distancia_direita = min(distancias_direita, default=math.inf)
        self.obstaculo_a_frente = (
            self.distancia_frontal < self.distancia_obstaculo
        )

        # Escolhe o lado mais livre para desvio. Z angular positivo gira
        # para a esquerda; negativo gira para a direita.
        if self.distancia_esquerda >= self.distancia_direita:
            self.direcao_desvio = 1.0
        else:
            self.direcao_desvio = -1.0

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

        visivel = msg.data[0] >= 0.5
        if not visivel:
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
        # Usa tempo monotonic para a validade da deteccao nao depender do
        # relogio simulado durante pausas ou atrasos iniciais do Gazebo.
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

    def executar_maquina_estados(self):
        if self.estado_atual == EstadoMissao.EXPLORANDO:
            self.estado_explorando()
        elif self.estado_atual == EstadoMissao.BANDEIRA_DETECTADA:
            self.estado_bandeira_detectada()
        elif self.estado_atual == EstadoMissao.NAVIGANDO_PARA_BANDEIRA:
            self.estado_navegando_para_bandeira()
        elif self.estado_atual == EstadoMissao.DESVIANDO_OBSTACULO:
            self.estado_desviando_obstaculo()
        elif self.estado_atual == EstadoMissao.REDETECTANDO_BANDEIRA:
            self.estado_redetectando_bandeira()
        elif self.estado_atual == EstadoMissao.POSICIONANDO_PARA_COLETA:
            self.estado_posicionando_para_coleta()
        elif self.estado_atual == EstadoMissao.CAPTURANDO_BANDEIRA:
            self.estado_capturando_bandeira()

    def estado_explorando(self):
        self.abrir_garra()

        if self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'a camera segmentada encontrou a label da bandeira azul',
            )
            self.publicar_velocidade(0.0, 0.0)
            return

        if self.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo no caminho durante exploracao '
                    f'({self.distancia_frontal:.2f} m)'
                ),
            )
            return

        # Movimento de exploracao em curva suave. A ideia e varrer a camera
        # sem assumir previamente onde a bandeira azul esta no mundo.
        fase = math.sin(time.monotonic() * 0.55)
        angular = self.limitar(
            self.amplitude_varredura_camera * fase,
            -self.velocidade_giro_busca,
            self.velocidade_giro_busca,
        )
        fator_obstaculo = self.fator_velocidade_por_obstaculo()
        linear = self.velocidade_exploracao * fator_obstaculo
        self.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'explorando em curva suave; '
                f'pose=({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f}), '
                f'frente={self.formatar_distancia(self.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=1.5,
        )

    def estado_bandeira_detectada(self):
        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                'deteccao visual ficou antiga logo apos encontrar a bandeira',
            )
            return

        det = self.deteccao_bandeira
        self.log_estado_periodico(
            (
                'bandeira detectada; calculando direcao relativa '
                f'(erro={det.erro_x:+.2f}, area={det.area_relativa:.3f})'
            ),
            periodo=0.5,
        )

        if self.bandeira_pronta_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira ja esta centralizada e proxima',
            )
        else:
            self.trocar_estado(
                EstadoMissao.NAVIGANDO_PARA_BANDEIRA,
                'bandeira detectada, iniciando aproximacao visual',
            )

    def estado_navegando_para_bandeira(self):
        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                f'bandeira perdida ha {self.tempo_desde_bandeira():.1f} s',
            )
            return

        if self.bandeira_pronta_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira centralizada/proxima o bastante para ajuste fino',
            )
            return

        if self.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo entre robo e bandeira '
                    f'({self.distancia_frontal:.2f} m)'
                ),
            )
            return

        det = self.deteccao_bandeira
        angular = self.controle_angular_para_bandeira()
        fator_alinhamento = max(0.25, 1.0 - abs(det.erro_x))
        fator_obstaculo = self.fator_velocidade_por_obstaculo()
        linear = self.velocidade_linear * fator_alinhamento * fator_obstaculo

        self.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'navegando para bandeira; '
                f'erro={det.erro_x:+.2f}, area={det.area_relativa:.3f}, '
                f'frente={self.formatar_distancia(self.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=1.0,
        )

    def estado_desviando_obstaculo(self):
        tempo_no_estado = time.monotonic() - self.instante_inicio_estado

        if (
            not self.obstaculo_a_frente
            and tempo_no_estado >= self.tempo_minimo_desvio
        ):
            if self.bandeira_recente():
                self.trocar_estado(
                    EstadoMissao.NAVIGANDO_PARA_BANDEIRA,
                    'obstaculo liberado e bandeira continua visivel',
                )
            else:
                self.trocar_estado(
                    EstadoMissao.EXPLORANDO,
                    'obstaculo liberado; retomando busca pela bandeira',
                )
            return

        angular = self.velocidade_angular_desvio * self.direcao_desvio
        sentido = 'esquerda' if self.direcao_desvio > 0 else 'direita'
        self.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                f'desviando: girando para {sentido}; '
                f'frente={self.formatar_distancia(self.distancia_frontal)}, '
                f'esq={self.formatar_distancia(self.distancia_esquerda)}, '
                f'dir={self.formatar_distancia(self.distancia_direita)}'
            ),
            periodo=0.8,
        )

    def estado_redetectando_bandeira(self):
        if self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'bandeira reapareceu no campo de visao',
            )
            return

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= self.tempo_reexploracao:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'tempo de redeteccao esgotado; voltando para exploracao',
            )
            return

        # Se a bandeira estava a direita, gira para a direita; se estava
        # a esquerda, gira para a esquerda.
        direcao_busca = -1.0 if self.ultimo_erro_bandeira > 0 else 1.0
        angular = self.velocidade_giro_busca * direcao_busca
        self.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                'redetectando bandeira; '
                f'ultimo_erro={self.ultimo_erro_bandeira:+.2f}, '
                f'tempo={tempo_no_estado:.1f}/{self.tempo_reexploracao:.1f}s'
            ),
            periodo=0.8,
        )

    def estado_posicionando_para_coleta(self):
        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                'bandeira perdida durante ajuste fino',
            )
            return

        if self.bandeira_pronta_para_coleta():
            self.trocar_estado(
                EstadoMissao.CAPTURANDO_BANDEIRA,
                'distancia e alinhamento suficientes para captura',
            )
            return

        det = self.deteccao_bandeira
        angular = self.controle_angular_para_bandeira()

        if abs(det.erro_x) > self.erro_alinhamento_bandeira:
            linear = 0.0
            fator_obstaculo = 1.0
            acao = 'ajustando orientacao'
        else:
            fator_obstaculo = self.fator_velocidade_por_obstaculo(
                permitir_aceleracao=False
            )
            linear = self.velocidade_posicionamento * fator_obstaculo
            acao = 'aproximando devagar'

        self.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                f'{acao}; erro={det.erro_x:+.2f}, '
                f'area={det.area_relativa:.3f}, '
                f'frente={self.formatar_distancia(self.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=0.7,
        )

    def estado_capturando_bandeira(self):
        self.publicar_velocidade(0.0, 0.0)
        self.fechar_garra()
        self.log_estado_periodico(
            (
                'missao concluida: robo parado de frente para a bandeira '
                f'azul em pose=({self.x:.2f}, {self.y:.2f}).'
            ),
            periodo=2.0,
        )

    def bandeira_recente(self):
        return (
            self.deteccao_bandeira.visivel
            and self.tempo_desde_bandeira() <= self.tempo_perda_bandeira
        )

    def tempo_desde_bandeira(self):
        if self.ultimo_instante_bandeira is None:
            return math.inf

        return time.monotonic() - self.ultimo_instante_bandeira

    def bandeira_pronta_para_posicionamento(self):
        det = self.deteccao_bandeira
        centralizada = abs(det.erro_x) <= self.erro_alinhamento_bandeira * 1.5
        proxima_por_imagem = (
            det.area_relativa >= self.area_posicionamento_bandeira
        )
        proxima_por_lidar = (
            self.distancia_frontal <= self.distancia_posicionamento
            and det.area_relativa >= self.area_posicionamento_bandeira * 0.4
        )
        return centralizada and (proxima_por_imagem or proxima_por_lidar)

    def bandeira_pronta_para_coleta(self):
        det = self.deteccao_bandeira
        centralizada = abs(det.erro_x) <= self.erro_alinhamento_bandeira
        proxima_por_imagem = det.area_relativa >= self.area_coleta_bandeira
        proxima_por_lidar = (
            self.distancia_frontal <= self.distancia_coleta
            and det.area_relativa >= self.area_posicionamento_bandeira
        )
        return centralizada and (proxima_por_imagem or proxima_por_lidar)

    def controle_angular_para_bandeira(self):
        # Erro positivo significa que a bandeira esta para a direita da imagem;
        # em ROS, angular.z negativo gira o robo para a direita.
        angular = -self.ganho_angular_bandeira * self.deteccao_bandeira.erro_x
        return self.limitar(
            angular,
            -self.velocidade_giro_busca,
            self.velocidade_giro_busca,
        )

    def fator_velocidade_por_obstaculo(self, permitir_aceleracao: bool = True):
        # Acelera um pouco quando o LIDAR nao enxerga nada na frente e reduz
        # progressivamente quando algo se aproxima do limiar de obstaculo.
        if math.isinf(self.distancia_frontal):
            fator = self.fator_velocidade_livre
        elif self.distancia_frontal >= self.distancia_velocidade_livre:
            fator = self.fator_velocidade_livre
        elif self.distancia_frontal <= self.distancia_obstaculo:
            fator = self.fator_velocidade_proxima
        else:
            faixa = self.distancia_velocidade_livre - self.distancia_obstaculo
            progresso = (self.distancia_frontal - self.distancia_obstaculo) / faixa
            fator = (
                self.fator_velocidade_proxima
                + progresso
                * (self.fator_velocidade_livre - self.fator_velocidade_proxima)
            )

        if not permitir_aceleracao:
            fator = min(1.0, fator)

        return fator

    def abrir_garra(self):
        if not self.habilitar_garra or self.garra_aberta or self.garra_fechada:
            return

        self.publicar_garra(self.comando_garra_aberta)
        self.garra_aberta = True
        self.get_logger().info(
            f'Garra: aberta para captura futura {self.comando_garra_aberta}.'
        )

    def fechar_garra(self):
        if not self.habilitar_garra or self.garra_fechada:
            return

        self.publicar_garra(self.comando_garra_captura)
        self.garra_fechada = True
        self.get_logger().info(
            f'Garra: comando de captura enviado {self.comando_garra_captura}.'
        )

    def publicar_garra(self, posicoes):
        comando = Float64MultiArray()
        comando.data = [float(posicao) for posicao in posicoes]
        self.garra_pub.publish(comando)

    def trocar_estado(self, novo_estado: EstadoMissao, motivo: str):
        if novo_estado == self.estado_atual:
            return

        estado_anterior = self.estado_atual
        self.estado_atual = novo_estado
        self.instante_inicio_estado = time.monotonic()
        self.get_logger().info(
            f'Estado: {estado_anterior.value} -> {novo_estado.value} | {motivo}'
        )

    def publicar_velocidade(self, linear: float, angular: float):
        cmd_vel = TwistStamped()
        cmd_vel.header.stamp = self.get_clock().now().to_msg()
        cmd_vel.twist.linear.x = float(linear)
        cmd_vel.twist.angular.z = float(angular)
        self.cmd_vel_pub.publish(cmd_vel)

    def log_estado_periodico(self, mensagem: str, periodo: float = 1.0):
        chave = f'estado_{self.estado_atual.value}'
        self.log_periodico(
            chave,
            f'[{self.estado_atual.value}] {mensagem}',
            periodo=periodo,
        )

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
