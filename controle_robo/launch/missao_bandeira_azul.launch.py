from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ------------------------------------------------------
    # Argumentos da missao completa
    # ------------------------------------------------------
    # Este launch orquestra a simulacao do pacote robo_movel e,
    # depois de pequenos atrasos, sobe o robo e o controlador.
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
    velocidade_linear_arg = DeclareLaunchArgument(
        'velocidade_linear',
        default_value='0.3',
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'velocidade_linear': LaunchConfiguration('velocidade_linear'),
            'velocidade_angular_desvio': LaunchConfiguration(
                'velocidade_angular_desvio'
            ),
            'distancia_obstaculo': LaunchConfiguration('distancia_obstaculo'),
            'angulo_frontal_graus': LaunchConfiguration('angulo_frontal_graus'),
        }.items(),
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        velocidade_linear_arg,
        velocidade_angular_desvio_arg,
        distancia_obstaculo_arg,
        angulo_frontal_graus_arg,
        inicia_simulacao,
        # O Gazebo precisa de um instante para criar o mundo antes do spawn.
        TimerAction(period=3.0, actions=[carrega_robo]),
        # O controle entra por ultimo para encontrar bridge e controladores ativos.
        TimerAction(period=8.0, actions=[controle_missao]),
    ])
