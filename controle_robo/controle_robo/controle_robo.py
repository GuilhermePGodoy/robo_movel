#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from scipy.spatial.transform import Rotation as R

from controle_robo.estimador_bandeira import EstimadorBandeira
from controle_robo.lidar import processar_scan
from controle_robo.maquina_estados import MaquinaEstadosMissao
from controle_robo.modelos_missao import DeteccaoBandeira, EstimativaBandeira
from controle_robo.planejador_grade import MapaGrade, PlanejadorGrade


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

        self.obstaculo_a_frente = False
        self.distancia_frontal = math.inf
        self.distancia_frontal_coleta = math.inf
        self.distancia_frontal_sem_centro = math.inf
        self.obstaculo_a_frente_sem_centro = False
        self.distancia_esquerda = math.inf
        self.distancia_direita = math.inf
        self.distancia_esquerda_frente = math.inf
        self.distancia_direita_frente = math.inf
        self.direcao_desvio = 1.0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_base = None

        self.mapa_grade = None
        self.ultimo_instante_mapa = None
        self.estimativa_bandeira = EstimativaBandeira()
        self.caminho_planejado = []
        self.indice_waypoint = 0
        self.destino_caminho = None
        self.alvo_caminho = None
        self.ultimo_waypoint_bloqueado_final = False
        self.ultimo_replanejamento_visual = -math.inf

        self.estimador_bandeira = EstimadorBandeira(
            self.fov_horizontal_camera,
            self.largura_real_bandeira,
            self.altura_real_bandeira,
            self.distancia_minima_estimativa,
            self.distancia_maxima_estimativa,
            self.historico_estimativas_bandeira,
        )
        self.planejador_grade = PlanejadorGrade(
            self.custo_desconhecido,
            self.inflacao_obstaculo_celulas,
            self.custo_adjacente_obstaculo,
        )

        self.qos_visualizacao = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

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
        self.caminho_pub = self.create_publisher(
            Path,
            '/caminho_planejado',
            self.qos_visualizacao,
        )
        self.alvo_estimado_pub = self.create_publisher(
            PoseStamped,
            '/bandeira_azul/alvo_estimado',
            self.qos_visualizacao,
        )

        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom_gt', self.odom_callback, 10)
        self.create_subscription(
            OccupancyGrid,
            self.topico_mapa,
            self.mapa_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            '/bandeira_azul/deteccao',
            self.deteccao_bandeira_callback,
            10,
        )

        self.maquina_estados = MaquinaEstadosMissao(self)
        self.timer = self.create_timer(0.1, self.maquina_estados.executar)

        self.get_logger().info(
            'CONTROLE | estado_inicial=EXPLORANDO | objetivo=bandeira_azul'
        )

    def configurar_parametros(self):
        """Declara e le os parametros usados pelo controle da missao."""

        self.declare_parameter('velocidade_angular_desvio', -0.4)
        self.declare_parameter('distancia_obstaculo', 0.6)
        self.declare_parameter('angulo_frontal_graus', 30.0)
        self.declare_parameter('angulo_ignorar_lidar_garra_graus', 8.0)
        self.declare_parameter('angulo_lidar_coleta_graus', 5.0)
        self.declare_parameter('distancia_lateral_desvio', 0.1)
        self.declare_parameter('velocidade_exploracao', 0.4)
        self.declare_parameter('velocidade_posicionamento', 0.08)
        self.declare_parameter('distancia_velocidade_livre', 1.8)
        self.declare_parameter('fator_velocidade_livre', 1.2)
        self.declare_parameter('fator_velocidade_proxima', 0.4)
        self.declare_parameter('amplitude_varredura_camera', 0.05)
        self.declare_parameter('velocidade_giro_busca', 0.3)
        self.declare_parameter('ganho_angular_bandeira', 0.9)
        self.declare_parameter('erro_alinhamento_bandeira', 0.12)
        self.declare_parameter('area_posicionamento_bandeira', 0.035)
        self.declare_parameter('area_coleta_bandeira', 0.07)
        self.declare_parameter('distancia_coleta_bandeira', 0.45)
        self.declare_parameter('tempo_perda_bandeira', 1.0)
        self.declare_parameter('tempo_redeteccao_bandeira', 4.0)
        self.declare_parameter('tempo_minimo_desvio', 0.8)
        self.declare_parameter('habilitar_garra', True)
        self.declare_parameter('garra_extensao_aberta', 0.0)
        self.declare_parameter('garra_direita_aberta', -0.06)
        self.declare_parameter('garra_esquerda_aberta', 0.06)
        self.declare_parameter('garra_extensao_captura', -0.8)
        self.declare_parameter('garra_direita_captura', 0.0)
        self.declare_parameter('garra_esquerda_captura', 0.0)
        self.declare_parameter('usar_planejamento_grade', True)
        self.declare_parameter('topico_mapa', '/grid_map')
        self.declare_parameter('fov_horizontal_camera', 1.57)
        self.declare_parameter('largura_real_bandeira', 0.3)
        self.declare_parameter('altura_real_bandeira', 0.48)
        self.declare_parameter('distancia_minima_estimativa', 0.2)
        self.declare_parameter('distancia_maxima_estimativa', 10.0)
        self.declare_parameter('confianca_minima_planejamento', 0.35)
        self.declare_parameter('historico_estimativas_bandeira', 5)
        self.declare_parameter('custo_desconhecido', 3.0)
        self.declare_parameter('inflacao_obstaculo_celulas', 1)
        self.declare_parameter('custo_adjacente_obstaculo', 2.0)
        self.declare_parameter('tolerancia_waypoint', 0.25)
        self.declare_parameter('tolerancia_alvo_planejado', 0.6)
        self.declare_parameter('ganho_angular_waypoint', 1.0)
        self.declare_parameter('velocidade_seguindo_caminho', 0.32)
        self.declare_parameter('deslocamento_replanejamento_alvo', 1.5)
        self.declare_parameter('intervalo_minimo_replanejamento_visual', 3.0)
        self.declare_parameter('distancia_aproximacao_bandeira', 0.55)
        self.declare_parameter('margem_borda_bandeira_px', 8.0)
        self.declare_parameter('area_minima_bandeira_inteira', 0.003)
        self.declare_parameter('frames_bandeira_inteira', 3)

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
        self.angulo_ignorar_lidar_garra_graus = abs(float(
            self.get_parameter('angulo_ignorar_lidar_garra_graus').value
        ))
        self.limite_central_garra = min(
            self.limite_frontal,
            math.radians(self.angulo_ignorar_lidar_garra_graus),
        )
        self.angulo_lidar_coleta_graus = abs(float(
            self.get_parameter('angulo_lidar_coleta_graus').value
        ))
        self.limite_central_coleta = min(
            self.limite_frontal,
            math.radians(self.angulo_lidar_coleta_graus),
        )
        self.distancia_lateral_desvio = float(
            self.get_parameter('distancia_lateral_desvio').value
        )
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
        self.distancia_coleta_bandeira = float(
            self.get_parameter('distancia_coleta_bandeira').value
        )
        self.tempo_perda_bandeira = float(
            self.get_parameter('tempo_perda_bandeira').value
        )
        self.tempo_redeteccao_bandeira = float(
            self.get_parameter('tempo_redeteccao_bandeira').value
        )
        self.tempo_minimo_desvio = float(
            self.get_parameter('tempo_minimo_desvio').value
        )
        self.habilitar_garra = bool(
            self.get_parameter('habilitar_garra').value
        )
        # Ordem real do gripper_controller:
        # [gripper_extension, right_gripper_joint, left_gripper_joint].
        # A haste fica baixa quando aberta/depositando e levantada quando
        # capturada/transportando a bandeira.
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
        self.usar_planejamento_grade = bool(
            self.get_parameter('usar_planejamento_grade').value
        )
        self.topico_mapa = str(self.get_parameter('topico_mapa').value)
        self.fov_horizontal_camera = float(
            self.get_parameter('fov_horizontal_camera').value
        )
        self.largura_real_bandeira = float(
            self.get_parameter('largura_real_bandeira').value
        )
        self.altura_real_bandeira = float(
            self.get_parameter('altura_real_bandeira').value
        )
        self.distancia_minima_estimativa = float(
            self.get_parameter('distancia_minima_estimativa').value
        )
        self.distancia_maxima_estimativa = float(
            self.get_parameter('distancia_maxima_estimativa').value
        )
        self.confianca_minima_planejamento = float(
            self.get_parameter('confianca_minima_planejamento').value
        )
        self.historico_estimativas_bandeira = int(
            self.get_parameter('historico_estimativas_bandeira').value
        )
        self.custo_desconhecido = float(
            self.get_parameter('custo_desconhecido').value
        )
        self.inflacao_obstaculo_celulas = int(
            self.get_parameter('inflacao_obstaculo_celulas').value
        )
        self.custo_adjacente_obstaculo = float(
            self.get_parameter('custo_adjacente_obstaculo').value
        )
        self.tolerancia_waypoint = float(
            self.get_parameter('tolerancia_waypoint').value
        )
        self.tolerancia_alvo_planejado = float(
            self.get_parameter('tolerancia_alvo_planejado').value
        )
        self.ganho_angular_waypoint = float(
            self.get_parameter('ganho_angular_waypoint').value
        )
        self.velocidade_seguindo_caminho = float(
            self.get_parameter('velocidade_seguindo_caminho').value
        )
        self.deslocamento_replanejamento_alvo = float(
            self.get_parameter('deslocamento_replanejamento_alvo').value
        )
        self.intervalo_minimo_replanejamento_visual = float(
            self.get_parameter('intervalo_minimo_replanejamento_visual').value
        )
        self.distancia_aproximacao_bandeira = float(
            self.get_parameter('distancia_aproximacao_bandeira').value
        )
        self.margem_borda_bandeira_px = float(
            self.get_parameter('margem_borda_bandeira_px').value
        )
        self.area_minima_bandeira_inteira = float(
            self.get_parameter('area_minima_bandeira_inteira').value
        )
        self.frames_bandeira_inteira = int(
            self.get_parameter('frames_bandeira_inteira').value
        )

    def scan_callback(self, msg: LaserScan):
        """Atualiza a leitura organizada do LIDAR."""

        if not msg.ranges:
            return

        leitura = processar_scan(
            msg,
            self.limite_frontal,
            self.limite_central_garra,
            self.limite_central_coleta,
            self.distancia_obstaculo,
        )

        self.distancia_frontal = leitura.distancia_frontal
        self.distancia_frontal_coleta = leitura.distancia_frontal_coleta
        self.distancia_frontal_sem_centro = leitura.distancia_frontal_sem_centro
        self.distancia_esquerda = leitura.distancia_esquerda
        self.distancia_direita = leitura.distancia_direita
        self.distancia_esquerda_frente = leitura.distancia_esquerda_frente
        self.distancia_direita_frente = leitura.distancia_direita_frente
        self.obstaculo_a_frente = leitura.obstaculo_a_frente
        self.obstaculo_a_frente_sem_centro = (
            leitura.obstaculo_a_frente_sem_centro
        )
        self.direcao_desvio = leitura.direcao_desvio

        if self.obstaculo_a_frente:
            self.log_periodico(
                'scan_obstaculo',
                (
                    'LIDAR | obstaculo=sim | '
                    f'fr={self.formatar_distancia(self.distancia_frontal)}, '
                    f'fr_coleta={self.formatar_distancia(self.distancia_frontal_coleta)}, '
                    f'fr_sem_centro={self.formatar_distancia(self.distancia_frontal_sem_centro)}, '
                    f'esq={self.formatar_distancia(self.distancia_esquerda)}, '
                    f'dir={self.formatar_distancia(self.distancia_direita)}, '
                    f'esq_fr={self.formatar_distancia(self.distancia_esquerda_frente)}, '
                    f'dir_fr={self.formatar_distancia(self.distancia_direita_frente)}, '
                    f'desvio={self.nome_lado_desvio()}'
                ),
                periodo=1.0,
            )

    def nome_lado_desvio(self):
        return 'esquerda' if self.direcao_desvio > 0.0 else 'direita'

    def imu_callback(self, msg: Imu):
        # A IMU fica assinada para debug e extensoes futuras. Nesta versao, a
        # orientacao usada no controle vem da odometria ground truth.
        _ = msg

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.pose_base is None:
            self.pose_base = (self.x, self.y)
            self.get_logger().info(
                f'BASE | pose_inicial=({self.x:.2f}, {self.y:.2f})'
            )

        orientation_q = msg.pose.pose.orientation
        quat = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        self.yaw = R.from_quat(quat).as_euler('xyz', degrees=False)[2]

    def mapa_callback(self, msg: OccupancyGrid):
        self.mapa_grade = msg
        self.ultimo_instante_mapa = time.monotonic()
        self.log_periodico(
            'mapa_recebido',
            (
                'MAPA | topico=/grid_map | '
                f'{msg.info.width}x{msg.info.height} '
                f'| res={msg.info.resolution:.2f}m'
            ),
            periodo=8.0,
        )

    def deteccao_bandeira_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 10:
            self.log_periodico(
                'deteccao_invalida',
                'VISAO | deteccao=invalida | motivo=mensagem_incompleta',
                periodo=2.0,
                nivel='warn',
            )
            return

        centro_x_haste = (
            float(msg.data[10])
            if len(msg.data) > 10
            else float(msg.data[4])
        )
        erro_x_haste = (
            float(msg.data[11])
            if len(msg.data) > 11
            else float(msg.data[1])
        )

        if msg.data[0] < 0.5:
            # Log ausente em todo frame polui bastante o terminal durante busca
            # e retorno. Para investigar a camera sem bandeira, use
            # debug_detector:=true no YAML/launch.
            return

        self.deteccao_bandeira = DeteccaoBandeira(
            visivel=True,
            erro_x=float(msg.data[1]),
            erro_x_haste=erro_x_haste,
            area_relativa=float(msg.data[2]),
            area=float(msg.data[3]),
            centro_x=float(msg.data[4]),
            centro_x_haste=centro_x_haste,
            centro_y=float(msg.data[5]),
            largura=int(msg.data[6]),
            altura=int(msg.data[7]),
            largura_imagem=int(msg.data[8]),
            altura_imagem=int(msg.data[9]),
            pose_robo_valida=True,
            x_robo=self.x,
            y_robo=self.y,
            yaw_robo=self.yaw,
        )

        # Tempo monotonic evita surpresas quando o Gazebo pausa ou reinicia o
        # relogio simulado durante os testes.
        self.ultimo_instante_bandeira = time.monotonic()
        self.log_periodico(
            'deteccao_bandeira',
            (
                'VISAO | bandeira=sim | '
                f'erro={self.deteccao_bandeira.erro_x:+.2f}, '
                f'erro_haste={self.deteccao_bandeira.erro_x_haste:+.2f}, '
                f'area={self.deteccao_bandeira.area_relativa:.3f}'
            ),
            periodo=1.0,
        )

    def atualizar_estimativa_bandeira(self):
        det = self.deteccao_bandeira
        if det.pose_robo_valida:
            x_referencia = det.x_robo
            y_referencia = det.y_robo
            yaw_referencia = det.yaw_robo
        else:
            x_referencia = self.x
            y_referencia = self.y
            yaw_referencia = self.yaw

        self.estimativa_bandeira = self.estimador_bandeira.estimar(
            det,
            x_referencia,
            y_referencia,
            yaw_referencia,
            self.distancia_frontal,
        )

        est = self.estimativa_bandeira
        if est.valida:
            self.publicar_alvo_estimado(est)

        delta_pose = math.hypot(self.x - x_referencia, self.y - y_referencia)
        delta_yaw = self.normalizar_angulo(self.yaw - yaw_referencia)

        self.log_periodico(
            'estimativa_bandeira',
            (
                'ESTIMATIVA | alvo=bandeira | '
                f'dist={est.distancia:.2f}m, '
                f'ang={est.angulo_relativo:+.2f}rad, '
                f'alvo=({est.x:.2f}, {est.y:.2f}), '
                f'pose_ref=({x_referencia:.2f}, {y_referencia:.2f}, '
                f'yaw={yaw_referencia:.2f}), '
                f'delta_pose={delta_pose:.2f}m, '
                f'delta_yaw={delta_yaw:+.2f}rad, '
                f'conf={est.confianca:.2f} '
                f'(tam={est.confianca_tamanho:.2f}, '
                f'centro={est.confianca_centro:.2f}, '
                f'borda={est.confianca_borda:.2f}, '
                f'lidar={est.confianca_lidar:.2f})'
            ),
            periodo=0.8,
        )
        return est

    def estimativa_bandeira_confiavel(self):
        if not self.estimativa_bandeira.valida:
            return False
        if self.estimativa_bandeira.confianca < self.confianca_minima_planejamento:
            return False
        if not self.estimativa_bandeira_dentro_do_mapa():
            alvo = self.ponto_aproximacao_bandeira()
            self.estimador_bandeira.limpar_historico()
            self.log_periodico(
                'estimativa_fora_mapa',
                (
                    'ESTIMATIVA | rejeitada=sim | motivo=fora_do_mapa | '
                    f'alvo_aprox=({alvo[0]:.2f}, {alvo[1]:.2f})'
                ),
                periodo=1.0,
                nivel='warn',
            )
            return False
        return True

    def estimativa_bandeira_dentro_do_mapa(self):
        if self.mapa_grade is None:
            return True

        mapa = MapaGrade(self.mapa_grade)
        alvo = self.ponto_aproximacao_bandeira()
        return mapa.celula_valida(mapa.world_to_grid(*alvo))

    def planejar_para_bandeira(self):
        if not self.estimativa_bandeira_confiavel():
            return False, 'estimativa da bandeira ainda nao e confiavel'

        alvo_aproximacao = self.ponto_aproximacao_bandeira()
        sucesso, motivo = self.planejar_para(alvo_aproximacao, 'bandeira')
        if sucesso:
            motivo = (
                f'{motivo}; estimativa_real='
                f'({self.estimativa_bandeira.x:.2f}, '
                f'{self.estimativa_bandeira.y:.2f})'
            )
        return sucesso, motivo

    def ponto_aproximacao_bandeira(self):
        est = self.estimativa_bandeira
        dx = self.x - est.x
        dy = self.y - est.y
        distancia = math.hypot(dx, dy)
        afastamento = max(0.0, self.distancia_aproximacao_bandeira)

        if distancia <= afastamento or distancia <= 1e-6:
            return est.x, est.y

        escala = afastamento / distancia
        return (
            est.x + dx * escala,
            est.y + dy * escala,
        )

    def replanejar_para_bandeira_congelada(self):
        """Recalcula a rota sem trocar o alvo visual ja escolhido.

        Enquanto estamos depurando o A*, a camera serve para escolher o alvo
        uma vez. Depois disso, obstaculos e mudancas no mapa podem forcar
        replanejamento, mas sempre para o mesmo ponto estimado.
        """

        if self.alvo_caminho is None or self.destino_caminho != 'bandeira':
            return False, 'nao existe alvo congelado da bandeira para replanejar'

        alvo_congelado = self.alvo_caminho
        sucesso, motivo = self.planejar_para(alvo_congelado, 'bandeira')
        if not sucesso and 'fora dos limites do mapa' not in motivo:
            self.alvo_caminho = alvo_congelado
            self.destino_caminho = 'bandeira'
        return sucesso, motivo

    def tem_alvo_bandeira_congelado(self):
        return self.destino_caminho == 'bandeira' and self.alvo_caminho is not None

    def planejar_para_base(self):
        if self.pose_base is None:
            return False, 'pose inicial da base ainda nao foi registrada'

        return self.planejar_para(self.pose_base, 'base')

    def planejar_para(self, alvo_xy, destino):
        if not self.usar_planejamento_grade:
            return False, 'planejamento por grade desabilitado'
        if self.mapa_grade is None:
            return False, 'controle ainda nao recebeu /grid_map'

        resultado = self.planejador_grade.planejar(
            self.mapa_grade,
            (self.x, self.y),
            alvo_xy,
        )
        if not resultado.sucesso:
            self.limpar_caminho()
            return False, resultado.motivo

        self.caminho_planejado = resultado.waypoints
        self.indice_waypoint = 0
        self.destino_caminho = destino
        self.alvo_caminho = (float(alvo_xy[0]), float(alvo_xy[1]))
        if destino == 'bandeira':
            self.ultimo_replanejamento_visual = time.monotonic()
        self.pular_waypoints_proximos()
        self.publicar_caminho_planejado()
        motivo = (
            f'{resultado.motivo}; '
            f'alvo=({self.alvo_caminho[0]:.2f}, {self.alvo_caminho[1]:.2f})'
        )
        self.log_periodico(
            f'planejamento_{destino}',
            (
                f'A* | destino={destino} | {motivo} | '
                f'waypoints={len(self.caminho_planejado)} | '
                f'custo={resultado.custo:.2f}'
            ),
            periodo=0.2,
        )
        return True, motivo

    def limpar_caminho(self):
        self.caminho_planejado = []
        self.indice_waypoint = 0
        self.destino_caminho = None
        self.alvo_caminho = None
        self.publicar_caminho_planejado()

    def caminho_ativo(self):
        return self.indice_waypoint < len(self.caminho_planejado)

    def waypoint_atual(self):
        if not self.caminho_ativo():
            return None
        return self.caminho_planejado[self.indice_waypoint]

    def pular_waypoints_proximos(self):
        while self.caminho_ativo():
            wx, wy = self.waypoint_atual()
            if math.hypot(wx - self.x, wy - self.y) > self.tolerancia_waypoint:
                break
            self.indice_waypoint += 1

    def comando_para_waypoint(self, distancia_frontal=None):
        self.pular_waypoints_proximos()
        waypoint = self.waypoint_atual()
        if waypoint is None:
            return None

        wx, wy = waypoint
        dx = wx - self.x
        dy = wy - self.y
        distancia = math.hypot(dx, dy)
        alvo_yaw = math.atan2(dy, dx)
        erro_yaw = self.normalizar_angulo(alvo_yaw - self.yaw)

        angular = self.limitar(
            self.ganho_angular_waypoint * erro_yaw,
            -self.velocidade_giro_busca,
            self.velocidade_giro_busca,
        )

        fator_alinhamento = max(0.0, 1.0 - abs(erro_yaw) / 1.2)
        if abs(erro_yaw) > 0.9:
            fator_alinhamento = 0.0

        linear = (
            self.velocidade_seguindo_caminho
            * fator_alinhamento
            * self.fator_velocidade_por_distancia(distancia_frontal)
        )
        return linear, angular, distancia, erro_yaw

    def waypoint_bloqueado(self):
        self.ultimo_waypoint_bloqueado_final = False
        self.pular_waypoints_proximos()
        waypoint = self.waypoint_atual()
        if waypoint is None or self.mapa_grade is None:
            return False

        mapa = MapaGrade(self.mapa_grade)
        celula = mapa.world_to_grid(*waypoint)
        valor = mapa.valor(celula)
        bloqueado = valor >= 100
        if bloqueado:
            distancia = math.hypot(waypoint[0] - self.x, waypoint[1] - self.y)
            self.ultimo_waypoint_bloqueado_final = (
                self.indice_waypoint >= len(self.caminho_planejado) - 1
            )
            if (
                not self.ultimo_waypoint_bloqueado_final
                and self.waypoint_na_vizinhanca_liberada_do_robo(mapa, celula)
            ):
                self.log_periodico(
                    'waypoint_bloqueado_perto_robo',
                    (
                        'A* | waypoint_ocupado_ignorado=sim | '
                        'motivo=vizinhanca_do_robo | '
                        f'wp={self.indice_waypoint + 1}/'
                        f'{len(self.caminho_planejado)}, '
                        f'celula=({celula.x}, {celula.y}), '
                        f'valor={valor}, dist={distancia:.2f}m'
                    ),
                    periodo=0.8,
                )
                return False

            self.log_periodico(
                'waypoint_bloqueado',
                (
                    'A* | waypoint_bloqueado=sim | '
                    f'wp={self.indice_waypoint + 1}/'
                    f'{len(self.caminho_planejado)}, '
                    f'final={self.ultimo_waypoint_bloqueado_final}, '
                    f'celula=({celula.x}, {celula.y}), '
                    f'valor={valor}, dist={distancia:.2f}m'
                ),
                periodo=0.8,
            )

        return bloqueado

    def waypoint_na_vizinhanca_liberada_do_robo(self, mapa, celula):
        atual = mapa.world_to_grid(self.x, self.y)
        raio = self.inflacao_obstaculo_celulas + 1
        return (
            abs(celula.x - atual.x) <= raio
            and abs(celula.y - atual.y) <= raio
        )

    def waypoint_bloqueado_eh_final(self):
        return self.ultimo_waypoint_bloqueado_final

    def alvo_bandeira_mudou_para_replanejar(self):
        if self.destino_caminho != 'bandeira' or self.alvo_caminho is None:
            return False
        if not self.estimativa_bandeira_confiavel():
            return False

        agora = time.monotonic()
        if (
            agora - self.ultimo_replanejamento_visual
            < self.intervalo_minimo_replanejamento_visual
        ):
            return False

        novo_alvo_caminho = self.ponto_aproximacao_bandeira()
        delta = math.hypot(
            novo_alvo_caminho[0] - self.alvo_caminho[0],
            novo_alvo_caminho[1] - self.alvo_caminho[1],
        )
        if delta < self.deslocamento_replanejamento_alvo:
            return False

        self.ultimo_replanejamento_visual = agora
        self.log_periodico(
            'replanejamento_alvo_bandeira',
            (
                'A* | replanejar=sim | motivo=alvo_visual_mudou | '
                f'delta={delta:.2f}m, '
                f'antigo=({self.alvo_caminho[0]:.2f}, '
                f'{self.alvo_caminho[1]:.2f}), '
                f'novo_aprox=({novo_alvo_caminho[0]:.2f}, '
                f'{novo_alvo_caminho[1]:.2f}), '
                f'estimativa_real=({self.estimativa_bandeira.x:.2f}, '
                f'{self.estimativa_bandeira.y:.2f})'
            ),
            periodo=0.5,
        )
        return True

    def chegou_perto_da_bandeira_planejada(self):
        if self.destino_caminho != 'bandeira' or self.alvo_caminho is None:
            return False

        distancia = math.hypot(
            self.alvo_caminho[0] - self.x,
            self.alvo_caminho[1] - self.y,
        )
        return distancia <= self.tolerancia_alvo_planejado

    def chegou_na_base(self):
        if self.pose_base is None:
            return False

        bx, by = self.pose_base
        return math.hypot(bx - self.x, by - self.y) <= self.tolerancia_alvo_planejado

    def publicar_caminho_planejado(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        for x, y in self.caminho_planejado[self.indice_waypoint:]:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.caminho_pub.publish(msg)

    def publicar_alvo_estimado(self, estimativa):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(estimativa.x)
        msg.pose.position.y = float(estimativa.y)
        msg.pose.orientation.z = math.sin(estimativa.angulo_mundo / 2.0)
        msg.pose.orientation.w = math.cos(estimativa.angulo_mundo / 2.0)
        self.alvo_estimado_pub.publish(msg)

    def fator_velocidade_por_distancia(self, distancia_frontal=None):
        if distancia_frontal is None:
            distancia_frontal = self.distancia_frontal

        if math.isinf(distancia_frontal):
            return self.fator_velocidade_livre
        if distancia_frontal <= self.distancia_obstaculo:
            return self.fator_velocidade_proxima
        if distancia_frontal >= self.distancia_velocidade_livre:
            return self.fator_velocidade_livre

        faixa = self.distancia_velocidade_livre - self.distancia_obstaculo
        progresso = (distancia_frontal - self.distancia_obstaculo) / faixa
        return (
            self.fator_velocidade_proxima
            + progresso
            * (self.fator_velocidade_livre - self.fator_velocidade_proxima)
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
    def normalizar_angulo(angulo: float):
        return math.atan2(math.sin(angulo), math.cos(angulo))

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
