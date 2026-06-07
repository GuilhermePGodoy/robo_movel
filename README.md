# Trabalho 1 - Robo movel autonomo com ROS 2

Projeto desenvolvido para o Trabalho Avaliado 1 de Programacao de Robos
Moveis. O sistema usa ROS 2 Jazzy e Gazebo Sim para simular um robo
diferencial que explora a arena, detecta a bandeira azul por camera de
segmentacao semantica, desvia de obstaculos com LIDAR e se posiciona para
captura-la com uma maquina de estados.

Material de apresentacao: [docs/apresentacao_trabalho1.md](docs/apresentacao_trabalho1.md)

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
    e alimenta o mapa de ocupacao.
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
  - Uso: visualizacao em RViz de celulas livres, ocupadas e desconhecidas.

## Maquina de Estados

A maquina de estados fica em
`src/controle_robo/controle_robo/controle_robo.py`.

- `EXPLORANDO`: avanca em curva suave para varrer a camera sem assumir a
  posicao da bandeira. Se o LIDAR detecta obstaculo, entra em desvio.
- `BANDEIRA_DETECTADA`: confirma que a deteccao visual e recente e decide se
  ja pode iniciar ajuste fino ou se ainda precisa navegar ate a bandeira.
- `NAVIGANDO_PARA_BANDEIRA`: usa o erro horizontal da deteccao para alinhar o
  robo com a bandeira e avanca proporcionalmente ao alinhamento.
- `DESVIANDO_OBSTACULO`: gira para o lado com maior distancia livre medida
  pelo LIDAR. Depois retoma navegacao para a bandeira ou volta a explorar.
- `REDETECTANDO_BANDEIRA`: se a bandeira some da camera, gira no sentido da
  ultima deteccao por alguns segundos antes de voltar a explorar.
- `POSICIONANDO_PARA_COLETA`: faz o ajuste fino de orientacao e distancia,
  aproximando devagar quando a bandeira esta centralizada.
- `CAPTURANDO_BANDEIRA`: para o robo e envia comando simples para fechar a
  garra.

O controle e reativo. Nao ha A* no caminho principal porque a arena e os
requisitos permitem uma estrategia mais simples: buscar a bandeira por visao,
desviar localmente com LIDAR e reposicionar caso a deteccao seja perdida. O
mapa `/grid_map` fica disponivel para visualizacao e para uma evolucao futura
com planejamento global.

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
- `fator_velocidade_livre` e `fator_velocidade_proxima`: aceleracao em caminho
  livre e reducao de velocidade perto de obstaculos.
- `area_posicionamento_bandeira` e `area_coleta_bandeira`: limiares visuais
  para aproximacao final.
- `habilitar_garra`: habilita/desabilita comandos para a garra.

## Debug

Com a missao rodando, estes comandos ajudam a entender o comportamento:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /scan --once
ros2 topic echo /grid_map --once
ros2 topic echo /diff_drive_base_controller/cmd_vel
ros2 topic echo /gripper_controller/commands
```

Os logs do no `controle_do_robo` informam transicoes de estado, motivo da
transicao, erro visual da bandeira, distancia frontal e comandos de velocidade.
Os logs do `detector_bandeira` informam quando a label 25 aparece na camera.

## Observacoes

- O detector usa `/robot_cam/labels_map` como fonte principal para evitar falso
  positivo por cor. O fallback por `colored_map` existe apenas para debug.
- A garra recebe comandos no topico `/gripper_controller/commands`; a qualidade
  fisica da captura ainda depende da posicao final no Gazebo.
- Para acelerar a iteracao durante testes, ajuste primeiro velocidades e
  limiares no YAML em vez de alterar o codigo.
