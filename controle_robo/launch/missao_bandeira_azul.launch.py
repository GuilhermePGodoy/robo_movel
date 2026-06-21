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
from controle_robo.launch_parametros import (
    ARGUMENTOS_CONTROLE,
    ARGUMENTOS_SIMULACAO,
    argumentos_para_include,
    declarar_argumentos,
    nomes,
)


CONFIGURACOES_MISSAO = nomes(ARGUMENTOS_SIMULACAO + ARGUMENTOS_CONTROLE)


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
        args=[CONFIGURACOES_MISSAO],
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

    argumentos_controle = {
        'config_file': LaunchConfiguration('config_file'),
    }
    argumentos_controle.update(dict(argumentos_para_include(ARGUMENTOS_CONTROLE)))

    controle_missao = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('controle_robo'),
                'launch',
                'controle_missao.launch.py',
            ])
        ),
        launch_arguments=argumentos_controle.items(),
    )

    return LaunchDescription([
        config_file_arg,
        aplica_config_file,
        *declarar_argumentos(ARGUMENTOS_SIMULACAO + ARGUMENTOS_CONTROLE),
        inicia_simulacao,
        TimerAction(
            period=LaunchConfiguration('atraso_carrega_robo'),
            actions=[carrega_robo],
        ),
        TimerAction(
            period=LaunchConfiguration('atraso_controle'),
            actions=[controle_missao],
        ),
    ])
