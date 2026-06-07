from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():
    # ------------------------------------------------------
    # Configuração de variáveis de ambiente para o Gazebo
    # ------------------------------------------------------
    # A variável GZ_SIM_SYSTEM_PLUGIN_PATH é usada para localizar plugins no Gazebo.
    # Ela é composta pelo caminho atual e pelo conteúdo de LD_LIBRARY_PATH.
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }

    # ------------------------------------------------------
    # Caminho para o mundo a ser carregado
    # ------------------------------------------------------

    # Verifica argumento com o nome do mundo que será simulado
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='arena_cilindros.sdf',
        description='Nome do arquivo .sdf do mundo a ser carregado'
    )
    gz_update_rate_arg = DeclareLaunchArgument(
        'gz_update_rate',
        default_value='2000',
        description='Taxa de atualizacao alvo do Gazebo em Hz'
    )
    gz_verbosity_arg = DeclareLaunchArgument(
        'gz_verbosity',
        default_value='3',
        description='Nivel de verbosidade do Gazebo (0 a 4)'
    )

    # Encontra o diretório de instalação do pacote 'robo_movel'.
    pkg_share = FindPackageShare("robo_movel").find("robo_movel")

    # Nome do arquivo do mundo (SDF) a ser carregado

    # Recupera dos parametros ou utiliza o default
    world_file_name = LaunchConfiguration('world')

    # Caminho completo para o arquivo do mundo
    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file_name
    ])

# Alguns teste utilizando cenários já existentes no gazebo
#    world_path='/usr/share/gz/gz-sim8/worlds/sensors_demo.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/heightmap.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/fuel.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/actor_crowd.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/auv_controls.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/buoyancy.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/fuel_textured_mesh.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/visualize_lidar.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/segmentation_camera.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/boundingbox_camera.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/spherical_coordinates.sdf'
#    world_path='/usr/share/gz/gz-sim8/worlds/rolling_shapes.sdf'

    # ------------------------------------------------------
    # Inicialização do simulador Gazebo
    # ------------------------------------------------------
    # Executa o comando: gz sim -r -v <verbosity> -z <update_rate> <world_path>
    # A opcao -z define a taxa alvo de atualizacao do servidor Gazebo.
    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            '-v',
            LaunchConfiguration('gz_verbosity'),
            '-z',
            LaunchConfiguration('gz_update_rate'),
            world_path,
        ],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )

    # ------------------------------------------------------
    # Configuração do caminho de recursos do Gazebo
    # ------------------------------------------------------
    # Define as variáveis de ambiente de recursos para que o Gazebo
    # consiga localizar os modelos personalizados armazenados no pacote.
    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models"),
        os.environ.get('GZ_SIM_RESOURCE_PATH', default=''),
    ])
    ign_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models"),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', default=''),
    ])

    gz_set_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=gz_models_path,
    )

    ign_set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=ign_models_path,
    )

    # ------------------------------------------------------
    # Ponte Gazebo <-> ROS 2
    # ------------------------------------------------------
    # Estabelece comunicação entre a câmera do céu no Gazebo e o ROS 2.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_world",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@gz.msgs.Image",
            # Necessário para controladores como diff_drive_controller
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        output="screen",
    )

    # ------------------------------------------------------
    # Descrição completa do lançamento
    # ------------------------------------------------------
    # Inclui as configurações de ambiente, a ponte e o lançamento do Gazebo.
    return LaunchDescription([
        world_file_arg,
        gz_update_rate_arg,
        gz_verbosity_arg,
        gz_set_env,
        ign_set_env,
        bridge,
        gazebo
    ])
