from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from controle_robo.launch_config import aplicar_config_file


CONFIGURACOES_CONTROLE = [
    'use_sim_time',
    'velocidade_linear',
    'velocidade_angular_desvio',
    'distancia_obstaculo',
    'angulo_frontal_graus',
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
    'topico_cmd_vel',
    'topico_scan',
    'topico_imu',
    'topico_odom',
    'topico_camera',
    'topico_deteccao_bandeira',
    'topico_garra',
]


def generate_launch_description():
    # ------------------------------------------------------
    # Argumentos configuraveis do controlador
    # ------------------------------------------------------
    # Mantem os topicos atuais como padrao, mas permite trocar nomes
    # pelo terminal sem modificar o codigo Python do no.
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
        args=[CONFIGURACOES_CONTROLE],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa o relogio publicado pelo simulador Gazebo',
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
    topico_deteccao_bandeira_arg = DeclareLaunchArgument(
        'topico_deteccao_bandeira',
        default_value='/bandeira_azul/deteccao',
        description='Topico publicado pelo detector visual da bandeira azul',
    )
    topico_garra_arg = DeclareLaunchArgument(
        'topico_garra',
        default_value='/gripper_controller/commands',
        description='Topico de comandos do JointGroupPositionController da garra',
    )

    detector_bandeira = Node(
        package='controle_robo',
        executable='detector_bandeira',
        name='detector_bandeira',
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'label_bandeira_azul': ParameterValue(
                    LaunchConfiguration('label_bandeira_azul'),
                    value_type=int,
                ),
                'area_minima_bandeira': ParameterValue(
                    LaunchConfiguration('area_minima_bandeira'),
                    value_type=float,
                ),
                'tolerancia_cor_bandeira': ParameterValue(
                    LaunchConfiguration('tolerancia_cor_bandeira'),
                    value_type=float,
                ),
            }
        ],
        remappings=[
            ('/robot_cam/labels_map', LaunchConfiguration('topico_camera')),
            (
                '/bandeira_azul/deteccao',
                LaunchConfiguration('topico_deteccao_bandeira'),
            ),
        ],
    )

    controle = Node(
        package='controle_robo',
        executable='controle_robo',
        name='controle_do_robo',
        output='screen',
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'),
                    value_type=bool,
                ),
                'velocidade_linear': ParameterValue(
                    LaunchConfiguration('velocidade_linear'),
                    value_type=float,
                ),
                'velocidade_angular_desvio': ParameterValue(
                    LaunchConfiguration('velocidade_angular_desvio'),
                    value_type=float,
                ),
                'distancia_obstaculo': ParameterValue(
                    LaunchConfiguration('distancia_obstaculo'),
                    value_type=float,
                ),
                'angulo_frontal_graus': ParameterValue(
                    LaunchConfiguration('angulo_frontal_graus'),
                    value_type=float,
                ),
                'velocidade_exploracao': ParameterValue(
                    LaunchConfiguration('velocidade_exploracao'),
                    value_type=float,
                ),
                'velocidade_posicionamento': ParameterValue(
                    LaunchConfiguration('velocidade_posicionamento'),
                    value_type=float,
                ),
                'distancia_velocidade_livre': ParameterValue(
                    LaunchConfiguration('distancia_velocidade_livre'),
                    value_type=float,
                ),
                'fator_velocidade_livre': ParameterValue(
                    LaunchConfiguration('fator_velocidade_livre'),
                    value_type=float,
                ),
                'fator_velocidade_proxima': ParameterValue(
                    LaunchConfiguration('fator_velocidade_proxima'),
                    value_type=float,
                ),
                'amplitude_varredura_camera': ParameterValue(
                    LaunchConfiguration('amplitude_varredura_camera'),
                    value_type=float,
                ),
                'velocidade_giro_busca': ParameterValue(
                    LaunchConfiguration('velocidade_giro_busca'),
                    value_type=float,
                ),
                'ganho_angular_bandeira': ParameterValue(
                    LaunchConfiguration('ganho_angular_bandeira'),
                    value_type=float,
                ),
                'erro_alinhamento_bandeira': ParameterValue(
                    LaunchConfiguration('erro_alinhamento_bandeira'),
                    value_type=float,
                ),
                'area_minima_bandeira': ParameterValue(
                    LaunchConfiguration('area_minima_bandeira'),
                    value_type=float,
                ),
                'area_posicionamento_bandeira': ParameterValue(
                    LaunchConfiguration('area_posicionamento_bandeira'),
                    value_type=float,
                ),
                'area_coleta_bandeira': ParameterValue(
                    LaunchConfiguration('area_coleta_bandeira'),
                    value_type=float,
                ),
                'distancia_posicionamento': ParameterValue(
                    LaunchConfiguration('distancia_posicionamento'),
                    value_type=float,
                ),
                'distancia_coleta': ParameterValue(
                    LaunchConfiguration('distancia_coleta'),
                    value_type=float,
                ),
                'tempo_perda_bandeira': ParameterValue(
                    LaunchConfiguration('tempo_perda_bandeira'),
                    value_type=float,
                ),
                'tempo_reexploracao': ParameterValue(
                    LaunchConfiguration('tempo_reexploracao'),
                    value_type=float,
                ),
                'tempo_minimo_desvio': ParameterValue(
                    LaunchConfiguration('tempo_minimo_desvio'),
                    value_type=float,
                ),
                'habilitar_garra': ParameterValue(
                    LaunchConfiguration('habilitar_garra'),
                    value_type=bool,
                ),
                'garra_extensao_aberta': ParameterValue(
                    LaunchConfiguration('garra_extensao_aberta'),
                    value_type=float,
                ),
                'garra_direita_aberta': ParameterValue(
                    LaunchConfiguration('garra_direita_aberta'),
                    value_type=float,
                ),
                'garra_esquerda_aberta': ParameterValue(
                    LaunchConfiguration('garra_esquerda_aberta'),
                    value_type=float,
                ),
                'garra_extensao_captura': ParameterValue(
                    LaunchConfiguration('garra_extensao_captura'),
                    value_type=float,
                ),
                'garra_direita_captura': ParameterValue(
                    LaunchConfiguration('garra_direita_captura'),
                    value_type=float,
                ),
                'garra_esquerda_captura': ParameterValue(
                    LaunchConfiguration('garra_esquerda_captura'),
                    value_type=float,
                ),
            }
        ],
        remappings=[
            (
                '/diff_drive_base_controller/cmd_vel',
                LaunchConfiguration('topico_cmd_vel'),
            ),
            ('/scan', LaunchConfiguration('topico_scan')),
            ('/imu', LaunchConfiguration('topico_imu')),
            ('/odom_gt', LaunchConfiguration('topico_odom')),
            (
                '/bandeira_azul/deteccao',
                LaunchConfiguration('topico_deteccao_bandeira'),
            ),
            ('/gripper_controller/commands', LaunchConfiguration('topico_garra')),
        ],
    )

    return LaunchDescription([
        config_file_arg,
        aplica_config_file,
        use_sim_time_arg,
        velocidade_linear_arg,
        velocidade_angular_desvio_arg,
        distancia_obstaculo_arg,
        angulo_frontal_graus_arg,
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
        topico_cmd_vel_arg,
        topico_scan_arg,
        topico_imu_arg,
        topico_odom_arg,
        topico_camera_arg,
        topico_deteccao_bandeira_arg,
        topico_garra_arg,
        detector_bandeira,
        controle,
    ])
