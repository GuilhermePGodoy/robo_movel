from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ------------------------------------------------------
    # Argumentos configuraveis do controlador
    # ------------------------------------------------------
    # Mantem os topicos atuais como padrao, mas permite trocar nomes
    # pelo terminal sem modificar o codigo Python do no.
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usa o relogio publicado pelo simulador Gazebo',
    )
    velocidade_linear_arg = DeclareLaunchArgument(
        'velocidade_linear',
        default_value='0.1',
        description='Velocidade linear maxima ao seguir a bandeira',
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
                'label_bandeira_azul': ParameterValue(
                    LaunchConfiguration('label_bandeira_azul'),
                    value_type=int,
                ),
                'tolerancia_cor_bandeira': ParameterValue(
                    LaunchConfiguration('tolerancia_cor_bandeira'),
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
            ('/robot_cam/labels_map', LaunchConfiguration('topico_camera')),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        velocidade_linear_arg,
        velocidade_angular_desvio_arg,
        distancia_obstaculo_arg,
        angulo_frontal_graus_arg,
        velocidade_exploracao_arg,
        velocidade_posicionamento_arg,
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
        label_bandeira_azul_arg,
        tolerancia_cor_bandeira_arg,
        topico_cmd_vel_arg,
        topico_scan_arg,
        topico_imu_arg,
        topico_odom_arg,
        topico_camera_arg,
        controle,
    ])
