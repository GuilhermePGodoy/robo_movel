"""Especificacao dos argumentos usados pelos launches da missao."""

from dataclasses import dataclass

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


@dataclass(frozen=True)
class Argumento:
    nome: str
    default: str
    descricao: str
    tipo: type | None = None


ARGUMENTOS_SIMULACAO = [
    Argumento('world', 'empty_arena.sdf', 'Mundo carregado no Gazebo'),
    Argumento(
        'atraso_carrega_robo',
        '1.0',
        'Espera antes de chamar o launch que faz o spawn do robo',
        float,
    ),
    Argumento(
        'atraso_controle',
        '10.0',
        'Espera antes de iniciar detector e controlador da missao',
        float,
    ),
]

PARAMETROS_COMUNS = [
    Argumento(
        'use_sim_time',
        'true',
        'Usa o relogio publicado pelo simulador',
        bool,
    ),
]

PARAMETROS_CONTROLE = [
    Argumento(
        'velocidade_angular_desvio',
        '-0.4',
        'Modulo da velocidade angular usada no desvio reativo',
        float,
    ),
    Argumento(
        'distancia_obstaculo',
        '0.6',
        'Distancia frontal minima para disparar desvio pelo LIDAR',
        float,
    ),
    Argumento(
        'distancia_obstaculo_retorno',
        '0.8',
        'Distancia minima para desvio ao voltar carregando a bandeira',
        float,
    ),
    Argumento(
        'angulo_frontal_graus',
        '30.0',
        'Abertura do setor frontal analisado no LIDAR',
        float,
    ),
    Argumento(
        'angulo_ignorar_lidar_garra_graus',
        '8.0',
        'Janela central do LIDAR ignorada durante o retorno com bandeira',
        float,
    ),
    Argumento(
        'angulo_lidar_coleta_graus',
        '5.0',
        'Janela central estreita usada para decidir quando fechar a garra',
        float,
    ),
    Argumento(
        'distancia_lateral_desvio',
        '0.1',
        'Folga lateral minima para encerrar uma manobra de desvio',
        float,
    ),
    Argumento(
        'velocidade_exploracao',
        '0.4',
        'Velocidade linear durante busca/exploracao',
        float,
    ),
    Argumento(
        'velocidade_posicionamento',
        '0.08',
        'Velocidade linear no ajuste fino antes de capturar',
        float,
    ),
    Argumento(
        'distancia_velocidade_livre',
        '1.8',
        'Distancia frontal a partir da qual o robo pode acelerar',
        float,
    ),
    Argumento(
        'fator_velocidade_livre',
        '1.2',
        'Multiplicador de velocidade quando o caminho esta livre',
        float,
    ),
    Argumento(
        'fator_velocidade_proxima',
        '0.4',
        'Multiplicador de velocidade quando existe algo perto',
        float,
    ),
    Argumento(
        'amplitude_varredura_camera',
        '0.05',
        'Amplitude da curva suave usada para varrer a camera',
        float,
    ),
    Argumento(
        'velocidade_giro_busca',
        '0.3',
        'Limite angular para busca, alinhamento visual e waypoints',
        float,
    ),
    Argumento(
        'ganho_angular_bandeira',
        '0.9',
        'Ganho proporcional para centralizar a haste da bandeira',
        float,
    ),
    Argumento(
        'erro_alinhamento_bandeira',
        '0.12',
        'Erro horizontal normalizado aceito como alinhado',
        float,
    ),
    Argumento(
        'area_posicionamento_bandeira',
        '0.035',
        'Area visual usada como referencia no ajuste fino da bandeira',
        float,
    ),
    Argumento(
        'distancia_posicionamento_bandeira',
        '0.9',
        'Distancia ate a estimativa para trocar A* por alinhamento visual',
        float,
    ),
    Argumento(
        'area_coleta_bandeira',
        '0.07',
        'Area relativa minima para permitir fechar a garra',
        float,
    ),
    Argumento(
        'distancia_coleta_bandeira',
        '0.45',
        'Distancia frontal para fechar a garra quando a haste esta alinhada',
        float,
    ),
    Argumento(
        'tempo_perda_bandeira',
        '1.0',
        'Tempo sem deteccao antes de considerar a bandeira perdida',
        float,
    ),
    Argumento(
        'tempo_redeteccao_bandeira',
        '4.0',
        'Tempo girando parado para reencontrar a bandeira apos desvio',
        float,
    ),
    Argumento(
        'tempo_minimo_desvio',
        '0.8',
        'Tempo minimo girando antes de tentar sair em arco do desvio',
        float,
    ),
    Argumento(
        'habilitar_garra',
        'true',
        'Permite enviar comandos para o gripper_controller',
        bool,
    ),
    Argumento(
        'garra_extensao_aberta',
        '0.0',
        'Haste baixa ao abrir/depositar a bandeira',
        float,
    ),
    Argumento(
        'garra_direita_aberta',
        '-0.06',
        'Abertura do braco direito da garra',
        float,
    ),
    Argumento(
        'garra_esquerda_aberta',
        '0.06',
        'Abertura do braco esquerdo da garra',
        float,
    ),
    Argumento(
        'garra_extensao_captura',
        '-0.8',
        'Haste levantada durante captura e transporte',
        float,
    ),
    Argumento(
        'garra_direita_captura',
        '0.0',
        'Posicao fechada do braco direito da garra',
        float,
    ),
    Argumento(
        'garra_esquerda_captura',
        '0.0',
        'Posicao fechada do braco esquerdo da garra',
        float,
    ),
    Argumento(
        'usar_planejamento_grade',
        'true',
        'Ativa navegacao por /grid_map e A*',
        bool,
    ),
    Argumento(
        'topico_mapa',
        '/grid_map',
        'Topico OccupancyGrid usado pelo planejador',
        str,
    ),
    Argumento(
        'habilitar_exploracao_desconhecida',
        'true',
        'Permite usar A* para ir ate fronteiras desconhecidas apos timeout',
        bool,
    ),
    Argumento(
        'timeout_exploracao_desconhecida',
        '120.0',
        'Tempo sem ver bandeira antes de buscar uma fronteira desconhecida',
        float,
    ),
    Argumento(
        'intervalo_exploracao_desconhecida',
        '45.0',
        'Cooldown entre tentativas de exploracao por fronteira',
        float,
    ),
    Argumento(
        'distancia_minima_alvo_desconhecido',
        '1.0',
        'Distancia minima ate a fronteira desconhecida escolhida',
        float,
    ),
    Argumento(
        'max_candidatos_exploracao_desconhecida',
        '80',
        'Numero maximo de fronteiras testadas ate achar um A* valido',
        int,
    ),
    Argumento(
        'fov_horizontal_camera',
        '1.57',
        'Campo de visao horizontal da camera em radianos',
        float,
    ),
    Argumento(
        'largura_real_bandeira',
        '0.3',
        'Largura real aproximada da bandeira em metros',
        float,
    ),
    Argumento(
        'altura_real_bandeira',
        '0.48',
        'Altura real aproximada do blob completo da bandeira',
        float,
    ),
    Argumento(
        'distancia_minima_estimativa',
        '0.2',
        'Menor distancia aceita na estimativa visual',
        float,
    ),
    Argumento(
        'distancia_maxima_estimativa',
        '10.0',
        'Maior distancia aceita na estimativa visual',
        float,
    ),
    Argumento(
        'fill_ratio_minimo_estimativa',
        '0.15',
        'Preenchimento minimo da bbox para aceitar estimativa por trigonometria',
        float,
    ),
    Argumento(
        'fill_ratio_maximo_estimativa',
        '0.82',
        'Preenchimento maximo da bbox; acima disso costuma ser so pano/haste',
        float,
    ),
    Argumento(
        'proporcao_minima_bbox_estimativa',
        '0.20',
        'Menor largura/altura da bbox aceita na estimativa visual',
        float,
    ),
    Argumento(
        'proporcao_maxima_bbox_estimativa',
        '1.35',
        'Maior largura/altura da bbox aceita na estimativa visual',
        float,
    ),
    Argumento(
        'custo_desconhecido',
        '3.0',
        'Custo de celulas desconhecidas no A*',
        float,
    ),
    Argumento(
        'inflacao_obstaculo_celulas',
        '1',
        'Inflacao dos obstaculos em celulas do grid',
        int,
    ),
    Argumento(
        'custo_adjacente_obstaculo',
        '2.0',
        'Custo extra para celulas livres encostadas na regiao bloqueada',
        float,
    ),
    Argumento(
        'tolerancia_waypoint',
        '0.25',
        'Distancia para considerar um waypoint alcancado',
        float,
    ),
    Argumento(
        'tolerancia_alvo_planejado',
        '0.6',
        'Distancia para considerar o alvo planejado alcancado',
        float,
    ),
    Argumento(
        'ganho_angular_waypoint',
        '1.0',
        'Ganho angular do seguidor de waypoints',
        float,
    ),
    Argumento(
        'distancia_lookahead_waypoint',
        '0.65',
        'Distancia a frente usada para mirar no caminho A*',
        float,
    ),
    Argumento(
        'peso_orientacao_inicial',
        '1.5',
        'Peso para preferir inicio de rota alinhado com o yaw atual',
        float,
    ),
    Argumento(
        'velocidade_seguindo_caminho',
        '0.32',
        'Velocidade linear nominal ao seguir caminho A*',
        float,
    ),
    Argumento(
        'deslocamento_replanejamento_alvo',
        '1.5',
        'Mudanca minima do alvo visual para recalcular rota',
        float,
    ),
    Argumento(
        'intervalo_minimo_replanejamento_visual',
        '3.0',
        'Cooldown entre replanejamentos causados pela camera',
        float,
    ),
    Argumento(
        'distancia_aproximacao_bandeira',
        '0.55',
        'Distancia antes da bandeira usada como alvo do A*',
        float,
    ),
    Argumento(
        'margem_borda_bandeira_px',
        '8',
        'Margem minima da bbox para considerar bandeira inteira',
        float,
    ),
    Argumento(
        'area_minima_bandeira_inteira',
        '0.003',
        'Area relativa minima para confirmar bandeira inteira',
        float,
    ),
    Argumento(
        'frames_bandeira_inteira',
        '3',
        'Frames consecutivos necessarios para confirmar bandeira inteira',
        int,
    ),
]

PARAMETROS_DETECTOR = [
    Argumento(
        'label_bandeira_azul',
        '25',
        'Label semantica da bandeira azul no labels_map',
        int,
    ),
    Argumento(
        'area_minima_bandeira',
        '5.0',
        'Area minima em pixels para aceitar uma regiao da bandeira',
        float,
    ),
    Argumento(
        'tolerancia_cor_bandeira',
        '0.0',
        'Tolerancia BGR usada apenas no fallback colored_map',
        float,
    ),
    Argumento(
        'debug_detector',
        'false',
        'Ativa logs detalhados e topicos de debug do detector',
        bool,
    ),
    Argumento(
        'publicar_mascara_debug',
        'false',
        'Publica mascara mono8 da bandeira detectada',
        bool,
    ),
    Argumento(
        'periodo_log_debug',
        '1.0',
        'Periodo dos logs detalhados do detector',
        float,
    ),
]

ARGUMENTOS_TOPICOS = [
    Argumento('topico_cmd_vel', '/diff_drive_base_controller/cmd_vel', 'Cmd_vel'),
    Argumento('topico_scan', '/scan', 'LaserScan'),
    Argumento('topico_imu', '/imu', 'IMU'),
    Argumento('topico_odom', '/odom_gt', 'Odometria ground truth'),
    Argumento('topico_camera', '/robot_cam/labels_map', 'Camera semantica'),
    Argumento(
        'topico_deteccao_bandeira',
        '/bandeira_azul/deteccao',
        'Saida numerica do detector',
    ),
    Argumento(
        'topico_debug_info_bandeira',
        '/bandeira_azul/debug_info',
        'Debug numerico do detector',
    ),
    Argumento(
        'topico_debug_mask_bandeira',
        '/bandeira_azul/debug_mask',
        'Mascara visual do detector',
    ),
    Argumento('topico_garra', '/gripper_controller/commands', 'Comando da garra'),
]

ARGUMENTOS_CONTROLE = (
    PARAMETROS_COMUNS
    + PARAMETROS_CONTROLE
    + PARAMETROS_DETECTOR
    + ARGUMENTOS_TOPICOS
)


def nomes(argumentos):
    return [arg.nome for arg in argumentos]


def declarar_argumentos(argumentos):
    return [
        DeclareLaunchArgument(
            arg.nome,
            default_value=arg.default,
            description=arg.descricao,
        )
        for arg in argumentos
    ]


def parametros_ros(argumentos):
    return {
        arg.nome: ParameterValue(
            LaunchConfiguration(arg.nome),
            value_type=arg.tipo,
        )
        for arg in argumentos
        if arg.tipo is not None
    }


def argumentos_para_include(argumentos):
    return {
        arg.nome: LaunchConfiguration(arg.nome)
        for arg in argumentos
    }.items()
