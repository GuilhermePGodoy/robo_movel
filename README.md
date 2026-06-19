# Trabalho 1 - Robo movel autonomo com ROS 2

Projeto desenvolvido para o Trabalho Avaliado 1 de Programacao de Robos
Moveis. O sistema usa ROS 2 Jazzy e Gazebo Sim para simular um robo
diferencial que explora a arena, detecta a bandeira azul por camera de
segmentacao semantica, estima a posicao da bandeira no mapa, planeja caminhos
com A*, desvia de obstaculos com LIDAR, captura a bandeira e retorna para a
base inicial.

Material de apresentacao:
[slides em PowerPoint](docs/robo_movel_algoritmo_ensino_medio.pptx) e
[roteiro em Markdown](docs/apresentacao_trabalho1.md).

## Como Rodar

```bash
cd ~/coding/usp_grad/robos_moveis/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robo_movel controle_robo
source install/setup.bash
ros2 launch controle_robo missao_bandeira_azul.launch.py
```

Para usar outro arquivo de configuracao:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py \
  config_file:=src/controle_robo/config/missao_bandeira_azul.yaml
```

Para listar os parametros aceitos pelo launch:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py --show-args
```

## Organizacao

O repositorio contem dois pacotes ROS 2 principais:

- `robo_movel`: modelagem, simulacao, sensores, ponte Gazebo/ROS, odometria
  ground truth e publicacao do mapa `/grid_map`.
- `controle_robo`: controle autonomo da missao, detector visual da bandeira,
  launches de orquestracao e arquivo YAML de parametros.

O launch principal `controle_robo/launch/missao_bandeira_azul.launch.py`
sobe a simulacao, carrega o robo, inicia os controladores, executa o mapper,
abre o RViz e inicia os nos de percepcao/controle da missao.

## Sensores e Topicos

- LIDAR: `/scan`
  - Mensagem: `sensor_msgs/msg/LaserScan`
  - Uso: identifica obstaculos a frente, escolhe o lado mais livre para desvio
    monitora folga lateral e alimenta o mapa de ocupacao.
- Camera semantica: `/robot_cam/labels_map`
  - Mensagem: `sensor_msgs/msg/Image`
  - Uso: cada pixel carrega uma label semantica do Gazebo. A bandeira azul usa
    a label `25`.
- Deteccao da bandeira: `/bandeira_azul/deteccao`
  - Mensagem: `std_msgs/msg/Float32MultiArray`
  - Campos: `visivel`, `erro_x`, `area_relativa`, `area_px`, `centro_x`,
    `centro_y`, `largura_box`, `altura_box`, `largura_imagem`,
    `altura_imagem`.
- IMU: `/imu`
  - Mensagem: `sensor_msgs/msg/Imu`
  - Uso atual: sensor configurado e disponivel para extensao. A missao usa a
    odometria ground truth para orientacao/logs.
- Odometria ground truth: `/odom_gt`
  - Mensagem: `nav_msgs/msg/Odometry`
  - Uso: logs de pose e publicacao do mapa.
- Mapa: `/grid_map`
  - Mensagem: `nav_msgs/msg/OccupancyGrid`
  - Uso: visualizacao em RViz e planejamento A* sobre celulas livres,
    ocupadas e desconhecidas.
- Caminho planejado: `/caminho_planejado`
  - Mensagem: `nav_msgs/msg/Path`
  - Uso: visualizacao do caminho A* atual no RViz.
- Alvo estimado: `/bandeira_azul/alvo_estimado`
  - Mensagem: `geometry_msgs/msg/PoseStamped`
  - Uso: mostra a hipotese de posicao da bandeira estimada pela camera.

## Maquina de Estados

A logica da missao foi separada em alguns arquivos pequenos:

- `src/controle_robo/controle_robo/controle_robo.py`: no ROS, parametros,
  publishers, subscribers e cache das leituras dos sensores.
- `src/controle_robo/controle_robo/maquina_estados.py`: transicoes de estado
  e comandos de movimento/garra.
- `src/controle_robo/controle_robo/modelos_missao.py`: enum dos estados e
  estrutura da deteccao visual.
- `src/controle_robo/controle_robo/estimador_bandeira.py`: trigonometria da
  camera para estimar distancia, angulo e confianca da bandeira.
- `src/controle_robo/controle_robo/planejador_grade.py`: A* sobre o
  `OccupancyGrid`, com custo maior para celulas desconhecidas.

- `EXPLORANDO`: avanca em curva suave para varrer a camera sem assumir a
  posicao da bandeira. Se o LIDAR detecta obstaculo, entra em desvio.
- `BANDEIRA_DETECTADA`: confirma que a deteccao visual e recente e decide se
  ja pode iniciar ajuste fino ou estimar a posicao no mapa.
- `ESTIMANDO_POSICAO_BANDEIRA`: usa tamanho da bounding box, FOV da camera,
  pose do robo e LIDAR para estimar `(x, y, confianca)` da bandeira.
- `PLANEJANDO_PARA_BANDEIRA`: roda A* do ponto atual ate uma celula livre
  perto da estimativa da bandeira.
- `SEGUINDO_CAMINHO_PARA_BANDEIRA`: segue waypoints do A* e replaneja se o
  mapa mostrar um bloqueio novo.
- `DESVIANDO_OBSTACULO`: gira para o lado com maior distancia livre medida
  pelo LIDAR. Depois replaneja, retoma busca visual ou volta a explorar.
- `REDETECTANDO_BANDEIRA`: se a bandeira some da camera, gira no sentido da
  ultima deteccao por alguns segundos antes de voltar a explorar.
- `POSICIONANDO_PARA_COLETA`: faz o ajuste fino de orientacao e distancia,
  aproximando devagar quando a bandeira esta centralizada.
- `CAPTURANDO_BANDEIRA`: para o robo e envia comando simples para fechar a
  garra.
- `PLANEJANDO_RETORNO_BASE` e `RETORNANDO_BASE`: usam A* para voltar ate a
  pose inicial salva como base.
- `ENTREGANDO_BANDEIRA` e `MISSAO_CONCLUIDA`: abrem a garra na base e deixam o
  robo parado.

O controle agora e hibrido: a camera acha e refina a bandeira, o A* aproxima o
robo pelo mapa e o LIDAR continua sendo a seguranca local contra colisao.

## Configuracao

Os parametros padrao estao em:

```text
src/controle_robo/config/missao_bandeira_azul.yaml
```

Alguns ajustes uteis:

- `atraso_carrega_robo` e `atraso_controle`: tempo para Gazebo, spawn e
  controladores iniciarem.
- `label_bandeira_azul`: label semantica da bandeira azul, atualmente `25`.
- `distancia_obstaculo`: distancia frontal minima antes de desviar.
- `distancia_lateral_desvio`: distancia lateral minima para sair do desvio e
  tambem para acionar o desvio caso algum objeto fique perto demais do lado.
- `fator_velocidade_livre` e `fator_velocidade_proxima`: aceleracao em caminho
  livre e reducao de velocidade perto de obstaculos.
- `area_posicionamento_bandeira` e `area_coleta_bandeira`: limiares visuais
  para aproximacao final.
- `usar_planejamento_grade`: habilita/desabilita A* sobre `/grid_map`.
- `confianca_minima_planejamento`: confianca minima da estimativa visual antes
  de planejar caminho.
- `custo_desconhecido` e `inflacao_obstaculo_celulas`: comportamento do A* em
  regioes desconhecidas e margem de seguranca ao redor de obstaculos.
- `tolerancia_waypoint` e `tolerancia_alvo_planejado`: quando considerar um
  waypoint ou alvo planejado alcancado.
- `habilitar_garra`: habilita/desabilita comandos para a garra.

## Debug

Com a missao rodando, estes comandos ajudam a entender o comportamento:

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

Os logs do no `controle_do_robo` informam transicoes de estado, motivo da
transicao, erro visual da bandeira, distancia frontal e comandos de velocidade.
Os logs do `detector_bandeira` informam encoding, formato da imagem, labels
mais comuns, quantidade de pixels da label `25`, contornos encontrados e maior
area. No `/bandeira_azul/debug_mask`, pixels brancos indicam onde o detector
enxerga a bandeira azul.

## Observacoes

- O detector usa `/robot_cam/labels_map` como fonte principal para evitar falso
  positivo por cor. O fallback por `colored_map` existe apenas para debug.
- A garra recebe comandos no topico `/gripper_controller/commands`; a qualidade
  fisica da captura ainda depende da posicao final no Gazebo.
- Para acelerar a iteracao durante testes, ajuste primeiro velocidades e
  limiares no YAML em vez de alterar o codigo.
