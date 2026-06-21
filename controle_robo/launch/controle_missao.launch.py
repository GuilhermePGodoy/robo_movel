from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from controle_robo.launch_config import aplicar_config_file
from controle_robo.launch_parametros import (
    ARGUMENTOS_CONTROLE,
    PARAMETROS_COMUNS,
    PARAMETROS_CONTROLE,
    PARAMETROS_DETECTOR,
    declarar_argumentos,
    nomes,
    parametros_ros,
)


CONFIGURACOES_CONTROLE = nomes(ARGUMENTOS_CONTROLE)


def generate_launch_description():
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

    detector_bandeira = Node(
        package='controle_robo',
        executable='detector_bandeira',
        name='detector_bandeira',
        output='screen',
        parameters=[
            parametros_ros(PARAMETROS_COMUNS + PARAMETROS_DETECTOR),
        ],
        remappings=[
            ('/robot_cam/labels_map', LaunchConfiguration('topico_camera')),
            (
                '/bandeira_azul/deteccao',
                LaunchConfiguration('topico_deteccao_bandeira'),
            ),
            (
                '/bandeira_azul/debug_info',
                LaunchConfiguration('topico_debug_info_bandeira'),
            ),
            (
                '/bandeira_azul/debug_mask',
                LaunchConfiguration('topico_debug_mask_bandeira'),
            ),
        ],
    )

    controle_robo = Node(
        package='controle_robo',
        executable='controle_robo',
        name='controle_do_robo',
        output='screen',
        parameters=[
            parametros_ros(PARAMETROS_COMUNS + PARAMETROS_CONTROLE),
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
        *declarar_argumentos(ARGUMENTOS_CONTROLE),
        detector_bandeira,
        controle_robo,
    ])
