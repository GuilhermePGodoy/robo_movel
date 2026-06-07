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
        description='Velocidade linear usada quando o caminho esta livre',
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
        default_value='/robot_cam/colored_map',
        description='Topico da camera de segmentacao colorida',
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
            ('/robot_cam/colored_map', LaunchConfiguration('topico_camera')),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        velocidade_linear_arg,
        velocidade_angular_desvio_arg,
        distancia_obstaculo_arg,
        angulo_frontal_graus_arg,
        topico_cmd_vel_arg,
        topico_scan_arg,
        topico_imu_arg,
        topico_odom_arg,
        topico_camera_arg,
        controle,
    ])
