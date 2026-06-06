from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    declare_world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.sdf',
        description='Nome do arquivo .sdf ou .world do mundo a ser carregado'
    )

    world_file = LaunchConfiguration('world')

    pkg_share = FindPackageShare("robo_movel").find("robo_movel")

    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file
    ])

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=":".join([
            pkg_share,
            os.path.join(pkg_share, "models"),
            os.environ.get('GZ_SIM_RESOURCE_PATH', default=''),
        ])
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-v',
            '3',
            world_path
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        set_gazebo_resource_path,
        gazebo
    ])
