# Trabalho 1 - Robô Móvel Autônomo com ROS 2

Projeto desenvolvido para o Trabalho Avaliado 1 de Programação de Robôs
Móveis. O sistema usa ROS 2 Jazzy e Gazebo Sim para simular um robô
diferencial que explora a arena, detecta a bandeira azul por câmera de
segmentação semântica, estima a posição da bandeira no mapa, planeja caminhos
com A*, desvia de obstáculos com LIDAR, captura a bandeira e retorna para a
base inicial.

Material de apresentação:
[slides em PowerPoint](docs/robo_movel_algoritmo_ensino_medio.pptx) e
[roteiro em Markdown](docs/apresentacao_trabalho1.md).

Para entender a organização interna do controle, veja também a
[documentação de arquitetura](docs/arquitetura_controle_robo.md).

## Como Rodar

```bash
cd ~/coding/usp_grad/robos_moveis/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robo_movel controle_robo
source install/setup.bash
ros2 launch controle_robo missao_bandeira_azul.launch.py
```

Para usar outro arquivo de configuração:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py \
  config_file:=src/controle_robo/config/missao_bandeira_azul.yaml
```

Para listar os parâmetros aceitos pelo launch:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py --show-args
```

## Organização

O repositório contém dois pacotes ROS 2 principais:

- `robo_movel`: modelagem, simulação, sensores, ponte Gazebo/ROS, odometria
  ground truth e publicação do mapa `/grid_map`.
- `controle_robo`: controle autônomo da missão, detector visual da bandeira,
  launches de orquestração e arquivo YAML de parâmetros.

O launch principal `controle_robo/launch/missao_bandeira_azul.launch.py`
sobe a simulação, carrega o robô, inicia os controladores, executa o mapper,
abre o RViz e inicia os nós de percepção/controle da missão.

## Sensores e Tópicos

- LIDAR: `/scan`
  - Mensagem: `sensor_msgs/msg/LaserScan`.
  - Uso: identifica obstáculos à frente, escolhe o lado mais livre para desvio,
    monitora folga lateral e alimenta o mapa de ocupação.
- Câmera semântica: `/robot_cam/labels_map`
  - Mensagem: `sensor_msgs/msg/Image`.
  - Uso: cada pixel carrega uma label semântica do Gazebo. A bandeira azul usa
    a label `25`.
- Detecção da bandeira: `/bandeira_azul/deteccao`
  - Mensagem: `std_msgs/msg/Float32MultiArray`.
  - Campos: `visivel`, `erro_x`, `area_relativa`, `area_px`, `centro_x`,
    `centro_y`, `largura_box`, `altura_box`, `largura_imagem`,
    `altura_imagem`, `centro_x_haste`, `erro_x_haste`.
- IMU: `/imu`
  - Mensagem: `sensor_msgs/msg/Imu`.
  - Uso atual: sensor configurado e disponível para extensão. A missão usa a
    odometria ground truth para orientação/logs.
- Odometria ground truth: `/odom_gt`
  - Mensagem: `nav_msgs/msg/Odometry`.
  - Uso: logs de pose e publicação do mapa.
- Mapa: `/grid_map`
  - Mensagem: `nav_msgs/msg/OccupancyGrid`.
  - Uso: visualização em RViz e planejamento A* sobre células livres,
    ocupadas e desconhecidas.
- Caminho planejado: `/caminho_planejado`
  - Mensagem: `nav_msgs/msg/Path`.
  - Uso: visualização do caminho A* atual no RViz.
- Alvo estimado: `/bandeira_azul/alvo_estimado`
  - Mensagem: `geometry_msgs/msg/PoseStamped`.
  - Uso: mostra a hipótese de posição da bandeira estimada pela câmera.

## Máquina de Estados

A lógica da missão foi separada em alguns arquivos pequenos:

- `src/controle_robo/controle_robo/controle_robo.py`: nó ROS, parâmetros,
  publishers, subscribers e cache das leituras dos sensores.
- `src/controle_robo/controle_robo/maquina_estados.py`: transições de estado
  e comandos de movimento/garra.
- `src/controle_robo/controle_robo/modelos_missao.py`: enum dos estados e
  estrutura da detecção visual.
- `src/controle_robo/controle_robo/estimador_bandeira.py`: trigonometria da
  câmera para estimar distância, ângulo e confiança da bandeira.
- `src/controle_robo/controle_robo/planejador_grade.py`: A* sobre o
  `OccupancyGrid`, com custo maior para células desconhecidas e células perto
  de obstáculos.
- `src/controle_robo/controle_robo/lidar.py`: leitura organizada do LaserScan
  para desvio local, coleta e retorno com bandeira na garra.
- `src/controle_robo/controle_robo/garra.py`: comandos de abrir, capturar e
  depositar a bandeira.

Estados principais:

- `EXPLORANDO`: avança em curva suave para varrer a câmera sem assumir a
  posição da bandeira. Se o LIDAR detecta obstáculo, entra em desvio.
- `BANDEIRA_DETECTADA`: confirma que a detecção visual é recente e decide se
  já pode iniciar ajuste fino ou estimar a posição no mapa.
- `ESTIMANDO_POSICAO_BANDEIRA`: usa tamanho da bounding box, FOV da câmera,
  pose do robô e LIDAR para estimar `(x, y, confiança)` da bandeira.
- `PLANEJANDO_PARA_BANDEIRA`: roda A* do ponto atual até uma célula livre
  perto da estimativa da bandeira.
- `SEGUINDO_CAMINHO_PARA_BANDEIRA`: segue waypoints do A* e replaneja se o
  mapa mostrar um bloqueio novo.
- `DESVIANDO_OBSTACULO`: gira para o lado com maior distância livre medida pelo
  LIDAR. Depois replaneja, retoma busca visual ou volta a explorar.
- `REENCONTRANDO_BANDEIRA`: depois de um desvio perto da coleta, gira parado
  para reenquadrar a bandeira antes de voltar a planejar ou explorar.
- `POSICIONANDO_PARA_COLETA`: faz o ajuste fino de orientação e distância,
  aproximando devagar quando a bandeira está centralizada.
- `CAPTURANDO_BANDEIRA`: para o robô, fecha a garra e levanta a haste para
  transportar a bandeira.
- `PLANEJANDO_RETORNO_BASE` e `RETORNANDO_BASE`: usam A* para voltar até a
  pose inicial salva como base.
- `ENTREGANDO_BANDEIRA` e `MISSAO_CONCLUIDA`: abrem a garra na base e deixam o
  robô parado.

O controle é híbrido: a câmera acha e refina a bandeira, o A* aproxima o robô
pelo mapa e o LIDAR continua sendo a segurança local contra colisão.

## Configuração

Os parâmetros padrão estão em:

```text
src/controle_robo/config/missao_bandeira_azul.yaml
```

Alguns ajustes úteis:

- `atraso_carrega_robo` e `atraso_controle`: tempo para Gazebo, spawn e
  controladores iniciarem.
- `label_bandeira_azul`: label semântica da bandeira azul, atualmente `25`.
- `distancia_obstaculo`: distância frontal mínima antes de desviar.
- `distancia_lateral_desvio`: distância lateral mínima para sair do desvio e
  também para acionar o desvio caso algum objeto fique perto demais do lado.
- `fator_velocidade_livre` e `fator_velocidade_proxima`: aceleração em caminho
  livre e redução de velocidade perto de obstáculos.
- `area_posicionamento_bandeira`: área visual mínima para trocar o A* pelo
  ajuste fino pela câmera.
- `area_coleta_bandeira`, `distancia_coleta_bandeira` e
  `angulo_lidar_coleta_graus`: área visual, distância e janela central do
  LIDAR usadas para fechar a garra quando a haste está alinhada.
- `tempo_redeteccao_bandeira`: tempo máximo girando parado para reencontrar a
  bandeira depois de um desvio perto da coleta.
- `usar_planejamento_grade`: habilita/desabilita A* sobre `/grid_map`.
- `confianca_minima_planejamento`: confiança mínima da estimativa visual antes
  de planejar caminho.
- `custo_desconhecido`, `inflacao_obstaculo_celulas` e
  `custo_adjacente_obstaculo`: comportamento do A* em regiões desconhecidas e
  perto de obstáculos.
- `tolerancia_waypoint` e `tolerancia_alvo_planejado`: quando considerar um
  waypoint ou alvo planejado alcançado.
- `habilitar_garra`: habilita/desabilita comandos para a garra.

## Debug

Com a missão rodando, estes comandos ajudam a entender o comportamento:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /bandeira_azul/debug_info
ros2 topic hz /robot_cam/labels_map
ros2 run rqt_image_view rqt_image_view /bandeira_azul/debug_mask
ros2 topic echo /scan --once
ros2 topic echo /grid_map --once
ros2 topic echo /bandeira_azul/alvo_estimado --once
ros2 topic echo /caminho_planejado --once
ros2 topic echo /diff_drive_base_controller/cmd_vel
ros2 topic echo /gripper_controller/commands
```

Os logs do nó `controle_do_robo` informam transições de estado, motivo da
transição, erro visual da bandeira, distância frontal e comandos de velocidade.
Os logs do `detector_bandeira` informam encoding, formato da imagem, labels
mais comuns, quantidade de pixels da label `25`, contornos encontrados e maior
área. No `/bandeira_azul/debug_mask`, pixels brancos indicam onde o detector
enxerga a bandeira azul.

## Observações

- O detector usa `/robot_cam/labels_map` como fonte principal para evitar falso
  positivo por cor. O fallback por `colored_map` existe apenas para debug.
- A garra recebe comandos no tópico `/gripper_controller/commands`; a qualidade
  física da captura ainda depende da posição final no Gazebo.
