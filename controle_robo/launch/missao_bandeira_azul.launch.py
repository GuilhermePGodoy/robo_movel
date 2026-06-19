from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from controle_robo.launch_config import aplicar_config_file


CONFIGURACOES_MISSAO = [
    'world',
    'use_sim_time',
    'atraso_carrega_robo',
    'atraso_controle',
    'velocidade_linear',
    'velocidade_angular_desvio',
    'distancia_obstaculo',
    'angulo_frontal_graus',
    'distancia_lateral_desvio',
    'velocidade_exploracao',
    'velocidade_posicionamento',
    'distancia_velocidade_livre',
    'fator_velocidade_livre',
    'fator_velocidade_proxima',
    'amplitude_varredura_camera',
    'velocidade_giro_busca',
    'ganho_angular_bandeira',
    'erro_alinhamento_bandeira',
    'area_minima_bandeira',
    'area_posicionamento_bandeira',
    'area_coleta_bandeira',
    'distancia_posicionamento',
    'distancia_coleta',
    'tempo_perda_bandeira',
    'tempo_reexploracao',
    'tempo_minimo_desvio',
    'habilitar_garra',
    'garra_extensao_aberta',
    'garra_direita_aberta',
    'garra_esquerda_aberta',
    'garra_extensao_captura',
    'garra_direita_captura',
    'garra_esquerda_captura',
    'label_bandeira_azul',
    'tolerancia_cor_bandeira',
    'debug_detector',
    'publicar_mascara_debug',
    'periodo_log_debug',
    'usar_planejamento_grade',
    'topico_mapa',
    'fov_horizontal_camera',
    'largura_real_bandeira',
    'altura_real_bandeira',
    'distancia_minima_estimativa',
    'distancia_maxima_estimativa',
    'confianca_minima_planejamento',
    'historico_estimativas_bandeira',
    'custo_desconhecido',
    'inflacao_obstaculo_celulas',
    'tolerancia_waypoint',
    'tolerancia_alvo_planejado',
    'ganho_angular_waypoint',
    'velocidade_seguindo_caminho',
    'deslocamento_replanejamento_alvo',
    'intervalo_minimo_replanejamento_visual',
    'topico_cmd_vel',
    'topico_scan',
    'topico_imu',
    'topico_odom',
    'topico_camera',
    'topico_deteccao_bandeira',
    'topico_debug_info_bandeira',
    'topico_debug_mask_bandeira',
    'topico_garra',
]


def generate_launch_description():
    # ------------------------------------------------------
    # Argumentos da missao completa
    # ------------------------------------------------------
    # Este launch orquestra a simulacao do pacote robo_movel e,
    # depois de pequenos atrasos, sobe o robo e o controlador.
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('controle_robo'),
            'config',
            'missao_bandeira_azul.yaml',
        ]),
        description='Arquivo YAML com parametros da missao',
    )

    aplica_config_file = OpaqueFunction(
        function=aplicar_config_file,
        args=[CONFIGURACOES_MISSAO],
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='arena_cilindros.sdf',
        description='Mundo Gazebo usado na missao',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa o relogio publicado pelo simulador Gazebo',
    )
    atraso_carrega_robo_arg = DeclareLaunchArgument(
        'atraso_carrega_robo',
        default_value='3.0',
        description='Atraso em segundos antes de carregar o robo',
    )
    atraso_controle_arg = DeclareLaunchArgument(
        'atraso_controle',
        default_value='12.0',
        description='Atraso em segundos antes de iniciar o controle',
    )
    velocidade_linear_arg = DeclareLaunchArgument(
        'velocidade_linear',
        default_value='0.1',
        description='Velocidade linear base ao seguir a bandeira',
    )
    velocidade_angular_desvio_arg = DeclareLaunchArgument(
        'velocidade_angular_desvio',
        default_value='-0.3',
        description='Velocidade angular usada para desviar de obstaculos',
    )
    distancia_obstaculo_arg = DeclareLaunchArgument(
        'distancia_obstaculo',
        default_value='0.5',
        description='Distancia minima frontal para considerar obstaculo',
    )
    angulo_frontal_graus_arg = DeclareLaunchArgument(
        'angulo_frontal_graus',
        default_value='30.0',
        description='Abertura angular frontal analisada no laser',
    )
    distancia_lateral_desvio_arg = DeclareLaunchArgument(
        'distancia_lateral_desvio',
        default_value='0.5',
        description='Distancia lateral minima para terminar o desvio',
    )
    velocidade_exploracao_arg = DeclareLaunchArgument(
        'velocidade_exploracao',
        default_value='0.08',
        description='Velocidade linear usada enquanto procura a bandeira',
    )
    velocidade_posicionamento_arg = DeclareLaunchArgument(
        'velocidade_posicionamento',
        default_value='0.04',
        description='Velocidade linear usada no ajuste fino para coleta',
    )
    distancia_velocidade_livre_arg = DeclareLaunchArgument(
        'distancia_velocidade_livre',
        default_value='1.8',
        description='Distancia frontal a partir da qual o caminho esta livre',
    )
    fator_velocidade_livre_arg = DeclareLaunchArgument(
        'fator_velocidade_livre',
        default_value='1.35',
        description='Multiplicador de velocidade quando nao ha nada a frente',
    )
    fator_velocidade_proxima_arg = DeclareLaunchArgument(
        'fator_velocidade_proxima',
        default_value='0.45',
        description='Multiplicador de velocidade quando ha algo perto',
    )
    amplitude_varredura_camera_arg = DeclareLaunchArgument(
        'amplitude_varredura_camera',
        default_value='0.18',
        description='Amplitude angular da curva de varredura na exploracao',
    )
    velocidade_giro_busca_arg = DeclareLaunchArgument(
        'velocidade_giro_busca',
        default_value='0.25',
        description='Velocidade angular maxima para busca e alinhamento',
    )
    ganho_angular_bandeira_arg = DeclareLaunchArgument(
        'ganho_angular_bandeira',
        default_value='0.9',
        description='Ganho proporcional para alinhar o robo com a bandeira',
    )
    erro_alinhamento_bandeira_arg = DeclareLaunchArgument(
        'erro_alinhamento_bandeira',
        default_value='0.12',
        description='Erro horizontal normalizado aceito como alinhado',
    )
    area_minima_bandeira_arg = DeclareLaunchArgument(
        'area_minima_bandeira',
        default_value='25.0',
        description='Area minima em pixels para aceitar um blob de bandeira',
    )
    area_posicionamento_bandeira_arg = DeclareLaunchArgument(
        'area_posicionamento_bandeira',
        default_value='0.02',
        description='Area relativa para iniciar ajuste fino de coleta',
    )
    area_coleta_bandeira_arg = DeclareLaunchArgument(
        'area_coleta_bandeira',
        default_value='0.07',
        description='Area relativa para considerar a bandeira alcancada',
    )
    distancia_posicionamento_arg = DeclareLaunchArgument(
        'distancia_posicionamento',
        default_value='0.9',
        description='Distancia frontal para iniciar ajuste fino de coleta',
    )
    distancia_coleta_arg = DeclareLaunchArgument(
        'distancia_coleta',
        default_value='0.45',
        description='Distancia frontal para considerar a bandeira alcancada',
    )
    tempo_perda_bandeira_arg = DeclareLaunchArgument(
        'tempo_perda_bandeira',
        default_value='1.0',
        description='Tempo sem deteccao antes de considerar a bandeira perdida',
    )
    tempo_reexploracao_arg = DeclareLaunchArgument(
        'tempo_reexploracao',
        default_value='3.0',
        description='Tempo tentando redetectar antes de voltar a explorar',
    )
    tempo_minimo_desvio_arg = DeclareLaunchArgument(
        'tempo_minimo_desvio',
        default_value='0.8',
        description='Tempo minimo girando durante um desvio de obstaculo',
    )
    habilitar_garra_arg = DeclareLaunchArgument(
        'habilitar_garra',
        default_value='true',
        description='Envia comandos simples para abrir e fechar a garra',
    )
    garra_extensao_aberta_arg = DeclareLaunchArgument(
        'garra_extensao_aberta',
        default_value='0.0',
        description='Posicao da junta de extensao quando a garra abre',
    )
    garra_direita_aberta_arg = DeclareLaunchArgument(
        'garra_direita_aberta',
        default_value='-0.06',
        description='Posicao da garra direita aberta',
    )
    garra_esquerda_aberta_arg = DeclareLaunchArgument(
        'garra_esquerda_aberta',
        default_value='0.06',
        description='Posicao da garra esquerda aberta',
    )
    garra_extensao_captura_arg = DeclareLaunchArgument(
        'garra_extensao_captura',
        default_value='0.0',
        description='Posicao da junta de extensao na captura',
    )
    garra_direita_captura_arg = DeclareLaunchArgument(
        'garra_direita_captura',
        default_value='0.0',
        description='Posicao da garra direita fechada',
    )
    garra_esquerda_captura_arg = DeclareLaunchArgument(
        'garra_esquerda_captura',
        default_value='0.0',
        description='Posicao da garra esquerda fechada',
    )
    label_bandeira_azul_arg = DeclareLaunchArgument(
        'label_bandeira_azul',
        default_value='25',
        description='Label semantica da bandeira azul no labels_map',
    )
    tolerancia_cor_bandeira_arg = DeclareLaunchArgument(
        'tolerancia_cor_bandeira',
        default_value='0.0',
        description='Tolerancia BGR usada somente no fallback colored_map',
    )
    debug_detector_arg = DeclareLaunchArgument(
        'debug_detector',
        default_value='true',
        description='Ativa logs e topico numerico de debug do detector',
    )
    publicar_mascara_debug_arg = DeclareLaunchArgument(
        'publicar_mascara_debug',
        default_value='true',
        description='Publica a mascara binaria da bandeira azul',
    )
    periodo_log_debug_arg = DeclareLaunchArgument(
        'periodo_log_debug',
        default_value='1.0',
        description='Periodo em segundos dos logs detalhados do detector',
    )
    usar_planejamento_grade_arg = DeclareLaunchArgument(
        'usar_planejamento_grade',
        default_value='true',
        description='Usa /grid_map e A* para aproximar da bandeira e retornar',
    )
    fov_horizontal_camera_arg = DeclareLaunchArgument(
        'fov_horizontal_camera',
        default_value='1.57',
        description='Campo de visao horizontal da camera em radianos',
    )
    largura_real_bandeira_arg = DeclareLaunchArgument(
        'largura_real_bandeira',
        default_value='0.3',
        description='Largura real aproximada do painel da bandeira em metros',
    )
    altura_real_bandeira_arg = DeclareLaunchArgument(
        'altura_real_bandeira',
        default_value='0.48',
        description='Altura real aproximada do blob segmentado da bandeira em metros',
    )
    distancia_minima_estimativa_arg = DeclareLaunchArgument(
        'distancia_minima_estimativa',
        default_value='0.2',
        description='Menor distancia aceita na estimativa visual da bandeira',
    )
    distancia_maxima_estimativa_arg = DeclareLaunchArgument(
        'distancia_maxima_estimativa',
        default_value='5.0',
        description='Maior distancia aceita na estimativa visual da bandeira',
    )
    confianca_minima_planejamento_arg = DeclareLaunchArgument(
        'confianca_minima_planejamento',
        default_value='0.70',
        description='Confianca minima da estimativa para rodar A*',
    )
    historico_estimativas_bandeira_arg = DeclareLaunchArgument(
        'historico_estimativas_bandeira',
        default_value='5',
        description='Quantidade de estimativas visuais usadas na media movel',
    )
    custo_desconhecido_arg = DeclareLaunchArgument(
        'custo_desconhecido',
        default_value='3.0',
        description='Custo de celulas desconhecidas no A*',
    )
    inflacao_obstaculo_celulas_arg = DeclareLaunchArgument(
        'inflacao_obstaculo_celulas',
        default_value='1',
        description='Inflacao dos obstaculos em celulas do grid',
    )
    tolerancia_waypoint_arg = DeclareLaunchArgument(
        'tolerancia_waypoint',
        default_value='0.25',
        description='Distancia para considerar waypoint alcancado',
    )
    tolerancia_alvo_planejado_arg = DeclareLaunchArgument(
        'tolerancia_alvo_planejado',
        default_value='0.6',
        description='Distancia para considerar alvo planejado alcancado',
    )
    ganho_angular_waypoint_arg = DeclareLaunchArgument(
        'ganho_angular_waypoint',
        default_value='1.0',
        description='Ganho angular para seguir waypoints do A*',
    )
    velocidade_seguindo_caminho_arg = DeclareLaunchArgument(
        'velocidade_seguindo_caminho',
        default_value='0.15',
        description='Velocidade linear ao seguir caminho planejado',
    )
    deslocamento_replanejamento_alvo_arg = DeclareLaunchArgument(
        'deslocamento_replanejamento_alvo',
        default_value='1.5',
        description='Quanto o alvo visual pode mudar antes de replanejar',
    )
    intervalo_minimo_replanejamento_visual_arg = DeclareLaunchArgument(
        'intervalo_minimo_replanejamento_visual',
        default_value='3.0',
        description='Cooldown entre replanejamentos guiados pela camera',
    )
    topico_cmd_vel_arg = DeclareLaunchArgument(
        'topico_cmd_vel',
        default_value='/diff_drive_base_controller/cmd_vel',
        description='Topico de comando de velocidade do controlador das rodas',
    )
    topico_scan_arg = DeclareLaunchArgument(
        'topico_scan',
        default_value='/scan',
        description='Topico do laser scan',
    )
    topico_imu_arg = DeclareLaunchArgument(
        'topico_imu',
        default_value='/imu',
        description='Topico da IMU',
    )
    topico_odom_arg = DeclareLaunchArgument(
        'topico_odom',
        default_value='/odom_gt',
        description='Topico de odometria ground truth',
    )
    topico_camera_arg = DeclareLaunchArgument(
        'topico_camera',
        default_value='/robot_cam/labels_map',
        description='Topico da camera com labels semanticas numericas',
    )
    topico_mapa_arg = DeclareLaunchArgument(
        'topico_mapa',
        default_value='/grid_map',
        description='Topico do OccupancyGrid usado pelo A*',
    )
    topico_deteccao_bandeira_arg = DeclareLaunchArgument(
        'topico_deteccao_bandeira',
        default_value='/bandeira_azul/deteccao',
        description='Topico publicado pelo detector visual da bandeira azul',
    )
    topico_debug_info_bandeira_arg = DeclareLaunchArgument(
        'topico_debug_info_bandeira',
        default_value='/bandeira_azul/debug_info',
        description='Topico com contadores numericos do detector da bandeira',
    )
    topico_debug_mask_bandeira_arg = DeclareLaunchArgument(
        'topico_debug_mask_bandeira',
        default_value='/bandeira_azul/debug_mask',
        description='Topico com mascara mono8 da label detectada',
    )
    topico_garra_arg = DeclareLaunchArgument(
        'topico_garra',
        default_value='/gripper_controller/commands',
        description='Topico de comandos do JointGroupPositionController da garra',
    )

    inicia_simulacao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robo_movel'),
                'launch',
                'inicia_simulacao.launch.py',
            ])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
        }.items(),
    )

    carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('robo_movel'),
                'launch',
                'carrega_robo.launch.py',
            ])
        ),
    )

    controle_missao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('controle_robo'),
                'launch',
                'controle_missao.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'velocidade_linear': LaunchConfiguration('velocidade_linear'),
            'velocidade_angular_desvio': LaunchConfiguration(
                'velocidade_angular_desvio'
            ),
            'distancia_obstaculo': LaunchConfiguration('distancia_obstaculo'),
            'angulo_frontal_graus': LaunchConfiguration('angulo_frontal_graus'),
            'distancia_lateral_desvio': LaunchConfiguration(
                'distancia_lateral_desvio'
            ),
            'velocidade_exploracao': LaunchConfiguration('velocidade_exploracao'),
            'velocidade_posicionamento': LaunchConfiguration(
                'velocidade_posicionamento'
            ),
            'distancia_velocidade_livre': LaunchConfiguration(
                'distancia_velocidade_livre'
            ),
            'fator_velocidade_livre': LaunchConfiguration(
                'fator_velocidade_livre'
            ),
            'fator_velocidade_proxima': LaunchConfiguration(
                'fator_velocidade_proxima'
            ),
            'amplitude_varredura_camera': LaunchConfiguration(
                'amplitude_varredura_camera'
            ),
            'velocidade_giro_busca': LaunchConfiguration('velocidade_giro_busca'),
            'ganho_angular_bandeira': LaunchConfiguration(
                'ganho_angular_bandeira'
            ),
            'erro_alinhamento_bandeira': LaunchConfiguration(
                'erro_alinhamento_bandeira'
            ),
            'area_minima_bandeira': LaunchConfiguration('area_minima_bandeira'),
            'area_posicionamento_bandeira': LaunchConfiguration(
                'area_posicionamento_bandeira'
            ),
            'area_coleta_bandeira': LaunchConfiguration('area_coleta_bandeira'),
            'distancia_posicionamento': LaunchConfiguration(
                'distancia_posicionamento'
            ),
            'distancia_coleta': LaunchConfiguration('distancia_coleta'),
            'tempo_perda_bandeira': LaunchConfiguration('tempo_perda_bandeira'),
            'tempo_reexploracao': LaunchConfiguration('tempo_reexploracao'),
            'tempo_minimo_desvio': LaunchConfiguration('tempo_minimo_desvio'),
            'habilitar_garra': LaunchConfiguration('habilitar_garra'),
            'garra_extensao_aberta': LaunchConfiguration('garra_extensao_aberta'),
            'garra_direita_aberta': LaunchConfiguration('garra_direita_aberta'),
            'garra_esquerda_aberta': LaunchConfiguration(
                'garra_esquerda_aberta'
            ),
            'garra_extensao_captura': LaunchConfiguration(
                'garra_extensao_captura'
            ),
            'garra_direita_captura': LaunchConfiguration(
                'garra_direita_captura'
            ),
            'garra_esquerda_captura': LaunchConfiguration(
                'garra_esquerda_captura'
            ),
            'label_bandeira_azul': LaunchConfiguration('label_bandeira_azul'),
            'tolerancia_cor_bandeira': LaunchConfiguration(
                'tolerancia_cor_bandeira'
            ),
            'debug_detector': LaunchConfiguration('debug_detector'),
            'publicar_mascara_debug': LaunchConfiguration(
                'publicar_mascara_debug'
            ),
            'periodo_log_debug': LaunchConfiguration('periodo_log_debug'),
            'usar_planejamento_grade': LaunchConfiguration(
                'usar_planejamento_grade'
            ),
            'fov_horizontal_camera': LaunchConfiguration('fov_horizontal_camera'),
            'largura_real_bandeira': LaunchConfiguration('largura_real_bandeira'),
            'altura_real_bandeira': LaunchConfiguration('altura_real_bandeira'),
            'distancia_minima_estimativa': LaunchConfiguration(
                'distancia_minima_estimativa'
            ),
            'distancia_maxima_estimativa': LaunchConfiguration(
                'distancia_maxima_estimativa'
            ),
            'confianca_minima_planejamento': LaunchConfiguration(
                'confianca_minima_planejamento'
            ),
            'historico_estimativas_bandeira': LaunchConfiguration(
                'historico_estimativas_bandeira'
            ),
            'custo_desconhecido': LaunchConfiguration('custo_desconhecido'),
            'inflacao_obstaculo_celulas': LaunchConfiguration(
                'inflacao_obstaculo_celulas'
            ),
            'tolerancia_waypoint': LaunchConfiguration('tolerancia_waypoint'),
            'tolerancia_alvo_planejado': LaunchConfiguration(
                'tolerancia_alvo_planejado'
            ),
            'ganho_angular_waypoint': LaunchConfiguration(
                'ganho_angular_waypoint'
            ),
            'velocidade_seguindo_caminho': LaunchConfiguration(
                'velocidade_seguindo_caminho'
            ),
            'deslocamento_replanejamento_alvo': LaunchConfiguration(
                'deslocamento_replanejamento_alvo'
            ),
            'intervalo_minimo_replanejamento_visual': LaunchConfiguration(
                'intervalo_minimo_replanejamento_visual'
            ),
            'topico_cmd_vel': LaunchConfiguration('topico_cmd_vel'),
            'topico_scan': LaunchConfiguration('topico_scan'),
            'topico_imu': LaunchConfiguration('topico_imu'),
            'topico_odom': LaunchConfiguration('topico_odom'),
            'topico_camera': LaunchConfiguration('topico_camera'),
            'topico_mapa': LaunchConfiguration('topico_mapa'),
            'topico_deteccao_bandeira': LaunchConfiguration(
                'topico_deteccao_bandeira'
            ),
            'topico_debug_info_bandeira': LaunchConfiguration(
                'topico_debug_info_bandeira'
            ),
            'topico_debug_mask_bandeira': LaunchConfiguration(
                'topico_debug_mask_bandeira'
            ),
            'topico_garra': LaunchConfiguration('topico_garra'),
        }.items(),
    )

    return LaunchDescription([
        config_file_arg,
        aplica_config_file,
        world_arg,
        use_sim_time_arg,
        atraso_carrega_robo_arg,
        atraso_controle_arg,
        velocidade_linear_arg,
        velocidade_angular_desvio_arg,
        distancia_obstaculo_arg,
        angulo_frontal_graus_arg,
        distancia_lateral_desvio_arg,
        velocidade_exploracao_arg,
        velocidade_posicionamento_arg,
        distancia_velocidade_livre_arg,
        fator_velocidade_livre_arg,
        fator_velocidade_proxima_arg,
        amplitude_varredura_camera_arg,
        velocidade_giro_busca_arg,
        ganho_angular_bandeira_arg,
        erro_alinhamento_bandeira_arg,
        area_minima_bandeira_arg,
        area_posicionamento_bandeira_arg,
        area_coleta_bandeira_arg,
        distancia_posicionamento_arg,
        distancia_coleta_arg,
        tempo_perda_bandeira_arg,
        tempo_reexploracao_arg,
        tempo_minimo_desvio_arg,
        habilitar_garra_arg,
        garra_extensao_aberta_arg,
        garra_direita_aberta_arg,
        garra_esquerda_aberta_arg,
        garra_extensao_captura_arg,
        garra_direita_captura_arg,
        garra_esquerda_captura_arg,
        label_bandeira_azul_arg,
        tolerancia_cor_bandeira_arg,
        debug_detector_arg,
        publicar_mascara_debug_arg,
        periodo_log_debug_arg,
        usar_planejamento_grade_arg,
        fov_horizontal_camera_arg,
        largura_real_bandeira_arg,
        altura_real_bandeira_arg,
        distancia_minima_estimativa_arg,
        distancia_maxima_estimativa_arg,
        confianca_minima_planejamento_arg,
        historico_estimativas_bandeira_arg,
        custo_desconhecido_arg,
        inflacao_obstaculo_celulas_arg,
        tolerancia_waypoint_arg,
        tolerancia_alvo_planejado_arg,
        ganho_angular_waypoint_arg,
        velocidade_seguindo_caminho_arg,
        deslocamento_replanejamento_alvo_arg,
        intervalo_minimo_replanejamento_visual_arg,
        topico_cmd_vel_arg,
        topico_scan_arg,
        topico_imu_arg,
        topico_odom_arg,
        topico_camera_arg,
        topico_mapa_arg,
        topico_deteccao_bandeira_arg,
        topico_debug_info_bandeira_arg,
        topico_debug_mask_bandeira_arg,
        topico_garra_arg,
        inicia_simulacao,
        # O Gazebo precisa de um instante para criar o mundo antes do spawn.
        TimerAction(
            period=LaunchConfiguration('atraso_carrega_robo'),
            actions=[carrega_robo],
        ),
        # O controle entra por ultimo para encontrar bridge e controladores ativos.
        TimerAction(
            period=LaunchConfiguration('atraso_controle'),
            actions=[controle_missao],
        ),
    ])
