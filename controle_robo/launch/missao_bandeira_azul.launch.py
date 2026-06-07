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
    'velocidade_exploracao',
    'velocidade_posicionamento',
    'distancia_velocidade_livre',
    'fator_velocidade_livre',
    'fator_velocidade_proxima',
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
    'label_bandeira_azul',
    'tolerancia_cor_bandeira',
    'topico_cmd_vel',
    'topico_scan',
    'topico_imu',
    'topico_odom',
    'topico_camera',
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
        default_value='1.0',
        description='Atraso em segundos antes de carregar o robo',
    )
    atraso_controle_arg = DeclareLaunchArgument(
        'atraso_controle',
        default_value='8.0',
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
            'label_bandeira_azul': LaunchConfiguration('label_bandeira_azul'),
            'tolerancia_cor_bandeira': LaunchConfiguration(
                'tolerancia_cor_bandeira'
            ),
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
        velocidade_exploracao_arg,
        velocidade_posicionamento_arg,
        distancia_velocidade_livre_arg,
        fator_velocidade_livre_arg,
        fator_velocidade_proxima_arg,
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
