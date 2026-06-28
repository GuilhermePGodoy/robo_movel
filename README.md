# Trabalho 2 - Robô Móvel Autônomo com ROS 2

Projeto desenvolvido para o Trabalho Avaliado 2 de Programação de Robôs
Móveis. O sistema usa ROS 2 Jazzy e Gazebo Sim para simular um robô diferencial
capaz de explorar uma arena, detectar uma bandeira azul por câmera semântica,
estimar sua posição no mapa, planejar caminhos com A*, desviar de obstáculos
com LIDAR, capturar a bandeira com a garra e retornar para a base inicial.

Este repositório parte do template disponibilizado pelo professor em
[matheusbg8/prm_2026](https://github.com/matheusbg8/prm_2026). Boa parte da
estrutura inicial da simulação, especialmente o pacote base que aqui chamamos
de `robo_movel`, vem desse material. As principais contribuições deste
trabalho foram a adaptação do projeto para ROS 2 Jazzy/Gazebo Sim, a criação do
pacote `controle_robo` e a implementação da lógica autônoma da missão.

Material de apresentação:
[slides em PowerPoint](docs/robo_movel_algoritmo_ensino_medio.pptx) e
[roteiro em Markdown](docs/apresentacao_trabalho2.md).

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

Se você adicionar novos mundos em `robo_movel/world`, rode o `colcon build`
novamente e dê `source install/setup.bash`; é isso que copia os arquivos para o
diretório instalado que o launch usa.

## Organização

O repositório contém dois pacotes ROS 2 principais:

- `robo_movel`: modelo do robô, mundos da simulação, sensores, bridges
  Gazebo/ROS, odometria ground truth e mapper simples que publica `/grid_map`.
- `controle_robo`: controle autônomo da missão, detector visual da bandeira,
  estimador de posição, planejador A*, controle da garra, launches e YAML de
  parâmetros.

O launch principal `controle_robo/launch/missao_bandeira_azul.launch.py` sobe
a simulação, carrega o robô, inicia os controladores, executa o mapper, abre o
RViz e inicia os nós de percepção/controle da missão.

## Sensores e Tópicos

- LIDAR: `/scan`
  - Mensagem: `sensor_msgs/msg/LaserScan`.
  - Uso: identifica obstáculos à frente e aos lados, escolhe lado de desvio,
    monitora folga lateral, ajuda a decidir a captura e alimenta o mapa.
- Câmera semântica: `/robot_cam/labels_map`
  - Mensagem: `sensor_msgs/msg/Image`.
  - Uso: cada pixel carrega uma label semântica do Gazebo. A bandeira azul usa
    a label `25`.
- Detecção da bandeira: `/bandeira_azul/deteccao`
  - Mensagem: `std_msgs/msg/Float32MultiArray`.
  - Campos: `visivel`, `erro_x`, `area_relativa`, `area_px`, `centro_x`,
    `centro_y`, `largura_box`, `altura_box`, `largura_imagem`,
    `altura_imagem`, `centro_x_haste`, `erro_x_haste`, `fill_ratio` e
    `proporcao_bbox`.
- Mapa: `/grid_map`
  - Mensagem: `nav_msgs/msg/OccupancyGrid`.
  - Uso: visualização em RViz e planejamento A* sobre células livres,
    ocupadas e desconhecidas.
- Congelamento do mapper: `/mapper/congelar`
  - Mensagem: `std_msgs/msg/Bool`.
  - Uso: após capturar a bandeira, o controle congela o mapa para evitar que a
    bandeira carregada apareça como um rastro falso de obstáculos.
- Caminho planejado: `/caminho_planejado`
  - Mensagem: `nav_msgs/msg/Path`.
  - Uso: visualização do caminho A* atual no RViz.
- Alvo estimado: `/bandeira_azul/alvo_estimado`
  - Mensagem: `geometry_msgs/msg/PoseStamped`.
  - Uso: mostra a melhor hipótese de posição da bandeira estimada pela câmera.
- Odometria ground truth: `/odom_gt`
  - Mensagem: `nav_msgs/msg/Odometry`.
  - Uso: pose do robô, base inicial, logs e referência para o planejamento.
- Garra: `/gripper_controller/commands`
  - Mensagem: `std_msgs/msg/Float64MultiArray`.
  - Uso: controla elevação da haste e abertura dos braços da garra.

## Máquina de Estados

A lógica da missão foi separada em arquivos menores:

- `controle_robo/controle_robo/controle_robo.py`: nó ROS, parâmetros,
  publishers, subscribers e cache das leituras dos sensores.
- `controle_robo/controle_robo/maquina_estados.py`: transições de estado e
  comandos de movimento/garra.
- `controle_robo/controle_robo/modelos_missao.py`: enum dos estados e modelos
  de dados usados na missão.
- `controle_robo/controle_robo/detector_bandeira.py`: segmentação da label
  semântica da bandeira azul.
- `controle_robo/controle_robo/criterios_visuais.py`: regras visuais pequenas,
  como bbox válida para estimativa e bandeira útil para posicionamento.
- `controle_robo/controle_robo/estimador_bandeira.py`: trigonometria da câmera
  para estimar distância, ângulo e posição da bandeira no mapa.
- `controle_robo/controle_robo/planejador_grade.py`: A* sobre `OccupancyGrid`,
  com custo maior para células desconhecidas e células próximas de obstáculos.
- `controle_robo/controle_robo/lidar.py`: leitura organizada do LaserScan para
  desvio local, coleta e retorno com bandeira na garra.
- `controle_robo/controle_robo/garra.py`: comandos de abrir, capturar,
  levantar, baixar e soltar a bandeira.

Estados principais:

- `EXPLORANDO`: avança em curva suave e procura a bandeira. Se passar muito
  tempo sem vê-la, pode planejar uma exploração de região desconhecida.
- `BANDEIRA_DETECTADA`: confirma que a leitura visual é recente e decide se já
  há informação suficiente para posicionamento ou estimativa.
- `ESTIMANDO_POSICAO_BANDEIRA`: usa bbox, FOV da câmera, pose do robô e filtros
  visuais para estimar `(x, y)` da bandeira.
- `PLANEJANDO_PARA_BANDEIRA`: roda A* do ponto atual até uma célula próxima da
  melhor estimativa da bandeira.
- `SEGUINDO_CAMINHO_PARA_BANDEIRA`: segue waypoints e replaneja se o mapa
  mostrar bloqueio novo.
- `PLANEJANDO_EXPLORACAO_DESCONHECIDA` e `SEGUINDO_CAMINHO_EXPLORACAO`: levam
  o robô para fronteiras do mapa quando a bandeira fica escondida.
- `DESVIANDO_OBSTACULO`: gira para o lado com maior folga medida pelo LIDAR e
  faz uma pequena manobra antes de retomar o plano.
- `REENCONTRANDO_BANDEIRA`: perto da coleta, gira parado para recuperar a
  bandeira no campo de visão.
- `POSICIONANDO_PARA_COLETA`: abandona o A* e usa câmera + LIDAR para alinhar a
  haste e aproximar devagar.
- `CAPTURANDO_BANDEIRA`: para o robô, fecha a garra, levanta a haste e congela
  o mapper.
- `PLANEJANDO_RETORNO_BASE` e `RETORNANDO_BASE`: usam A* para voltar até a pose
  inicial salva como base.
- `ENTREGANDO_BANDEIRA`: abaixa a haste aos poucos, abre a garra e dá uma
  pequena ré para deixar a bandeira no chão.
- `MISSAO_CONCLUIDA`: encerra a missão com o robô parado.

O controle é híbrido: a câmera acha e refina a bandeira, o A* aproxima o robô
pelo mapa e o LIDAR continua sendo a segurança local contra colisão.

## Mapa e A*

O mapper publica um `OccupancyGrid` simples:

- `-1`: célula desconhecida.
- `0`: célula livre observada.
- `100`: célula ocupada confirmada.

Obstáculos descobertos pelo LIDAR são persistentes: depois que uma célula é
marcada como ocupada, raios livres posteriores não apagam essa informação. Isso
deixa o A* mais estável em arenas com obstáculos fixos.

Durante o retorno com a bandeira, o mapper é congelado. Sem isso, a própria
bandeira presa na garra seria mapeada como obstáculo na frente do robô e
criaria um rastro falso no `/grid_map`.

O planejador A*:

- bloqueia células ocupadas;
- aceita células desconhecidas com custo maior;
- pode inflar obstáculos como margem dura;
- aumenta o custo de células livres adjacentes a obstáculos;
- evita escolher um ponto de seguimento mais perto que a tolerância de
  waypoint, reduzindo oscilações perto de waypoints já alcançados.

## Configuração

Os parâmetros padrão estão em:

```text
controle_robo/config/missao_bandeira_azul.yaml
```

Alguns ajustes úteis:

- `world`: mundo SDF usado no Gazebo.
- `atraso_carrega_robo` e `atraso_controle`: tempo para Gazebo, spawn e
  controladores iniciarem.
- `label_bandeira_azul`: label semântica da bandeira azul, atualmente `25`.
- `distancia_obstaculo`: distância frontal mínima antes de desviar.
- `distancia_obstaculo_retorno`: distância mínima usada no retorno, maior que
  a normal para carregar a bandeira com mais folga.
- `distancia_lateral_desvio`: distância lateral mínima para sair do desvio e
  para acionar desvio quando algo fica perto demais do lado.
- `fill_ratio_*_estimativa` e `proporcao_*_bbox_estimativa`: filtros que dizem
  quando a bbox da bandeira é boa o bastante para estimar posição no mapa.
- `distancia_posicionamento_bandeira`: distância até a melhor estimativa para
  abandonar o A* e iniciar o ajuste visual.
- `area_aproximacao_bandeira`, `area_coleta_bandeira` e
  `distancia_coleta_bandeira`: critérios para aproximação e fechamento da
  garra.
- `tempo_sem_ver_bandeira_para_explorar`: timeout para buscar fronteira
  desconhecida quando a bandeira não aparece.
- `custo_desconhecido`, `inflacao_obstaculo_celulas` e
  `custo_adjacente_obstaculo`: comportamento do A* em regiões desconhecidas e
  perto de obstáculos.
- `tempo_descida_garra_base`, `tempo_abertura_garra_base`,
  `velocidade_re_entrega` e `tempo_re_entrega`: sequência final de entrega da
  bandeira na base.

## Debug

Com a missão rodando, estes comandos ajudam a entender o comportamento:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /bandeira_azul/debug_info
ros2 run rqt_image_view rqt_image_view /bandeira_azul/debug_mask
ros2 topic echo /scan --once
ros2 topic echo /grid_map --once
ros2 topic echo /bandeira_azul/alvo_estimado --once
ros2 topic echo /caminho_planejado --once
ros2 topic echo /mapper/congelar
ros2 topic echo /diff_drive_base_controller/cmd_vel
ros2 topic echo /gripper_controller/commands
```

Os logs do nó `controle_do_robo` mostram transições de estado, motivo da
transição, pose, leituras importantes do LIDAR e detecção visual. Os logs do
`detector_bandeira` mostram a bbox, `fill_ratio` e proporção da bbox, que são
as informações usadas para aceitar ou rejeitar estimativas da bandeira.

No RViz, os tópicos mais úteis são `/grid_map`, `/caminho_planejado` e
`/bandeira_azul/alvo_estimado`.

## Observações

- O detector usa `/robot_cam/labels_map` como fonte principal para evitar falso
  positivo por cor. O fallback por `colored_map` existe apenas para debug.
- A captura física depende da geometria da bandeira no Gazebo e do alinhamento
  final da haste com a garra.
- O robô usa heurísticas simples. Em mapas mais fechados, o resultado depende
  bastante da posição inicial, da visibilidade da bandeira e da qualidade do
  mapa publicado durante a exploração.
