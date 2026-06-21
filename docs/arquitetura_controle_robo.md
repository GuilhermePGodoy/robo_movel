# Arquitetura do Controle da Missão

Este documento explica a parte implementada no pacote `controle_robo`. A ideia
é ajudar alguém novo no projeto a entender onde cada decisão mora antes de
mudar o código.

## Visão Geral

O sistema ficou dividido em dois pacotes:

- `robo_movel`: sobe a simulação, o modelo do robô, os sensores, os
  controladores e o mapa `/grid_map`.
- `controle_robo`: detecta a bandeira azul, decide a missão, planeja caminhos
  com A*, desvia de obstáculos e controla a garra.

O launch principal é:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py
```

Ele lê `controle_robo/config/missao_bandeira_azul.yaml`, sobe a simulação do
pacote `robo_movel`, espera o robô nascer e inicia os nós de missão.

## Arquivos Principais

- `controle_robo.py`
  - É o nó ROS principal.
  - Declara parâmetros, cria publishers/subscribers e guarda a última leitura
    de cada sensor.
  - Não deveria concentrar regras grandes da missão. Quando uma regra crescer,
    vale mover para um módulo menor.

- `maquina_estados.py`
  - Contém a máquina de estados da missão.
  - Lê o estado atual dos sensores guardado em `ControleRobo`.
  - Decide quando publicar velocidades, abrir/fechar a garra, planejar ou mudar
    de estado.

- `detector_bandeira.py`
  - Recebe a imagem semântica `/robot_cam/labels_map`.
  - Procura pixels com a label `25`, que representam a bandeira azul.
  - Publica `/bandeira_azul/deteccao` como um vetor numérico simples.

- `visao_bandeira.py`
  - Funções auxiliares para extrair informações visuais da bandeira.
  - Inclui a estimativa da posição da haste, que é melhor para alinhar a garra
    do que o centro da bounding box inteira.

- `criterios_visuais.py`
  - Heurísticas pequenas sobre a detecção visual.
  - Exemplos: "tem bandeira parcial?", "tem bandeira suficiente para
    posicionamento?", "a bandeira parece inteira por alguns frames?".

- `estimador_bandeira.py`
  - Usa o tamanho da bbox na câmera para estimar distância.
  - Usa o deslocamento horizontal da haste/bbox para estimar ângulo relativo.
  - Transforma essa hipótese em coordenada do mapa usando a pose do robô.

- `planejador_grade.py`
  - Implementa A* em cima do `nav_msgs/OccupancyGrid`.
  - Células livres custam pouco, desconhecidas custam mais, ocupadas bloqueiam.
  - Infla obstáculos para o robô não passar raspando.
  - Também aumenta o custo de células livres adjacentes aos obstáculos, para
    favorecer caminhos com mais folga.

- `lidar.py`
  - Organiza o `LaserScan` em regiões: frente, frente sem centro, frente
    estreita de coleta, esquerda e direita.
  - Escolhe o lado de desvio olhando qual lado/frente está mais livre.
  - A leitura "frente sem centro" é usada no retorno, porque a bandeira presa
    na garra aparece no centro do LIDAR.
  - A leitura "frente de coleta" é uma faixa estreita central usada para decidir
    quando fechar a garra.

- `garra.py`
  - Esconde os detalhes do vetor enviado para
    `/gripper_controller/commands`.
  - A ordem do vetor é `[haste, garra_direita, garra_esquerda]`.

- `launch_parametros.py`
  - Lista única dos argumentos dos launchers.
  - Evita duplicar dezenas de `DeclareLaunchArgument` em mais de um arquivo.

## Sensores Usados

### Câmera Semântica

Tópico: `/robot_cam/labels_map`

Cada pixel da imagem carrega uma classe semântica do Gazebo. O detector usa a
label `25` para achar a bandeira azul. A partir dessa região, ele calcula:

- centro da bbox;
- centro aproximado da haste;
- erro horizontal normalizado;
- área relativa da bandeira na imagem;
- largura e altura da bbox.

A haste é importante porque a garra precisa chegar no poste, não no centro do
painel azul.

### LIDAR

Tópico: `/scan`

O LIDAR é a segurança local. Ele responde perguntas como:

- tem algo perto na frente?
- tem algo exatamente no centro da frente para a garra fechar?
- qual lado parece mais livre para desviar?
- depois da captura, ignorando o centro do scan, ainda tem obstáculo real na
  frente?

Durante o retorno para a base, a bandeira fica na frente do robô e poderia ser
confundida com obstáculo. Por isso `lidar.py` também calcula a frente sem a
janela central.

### Mapa

Tópico: `/grid_map`

O mapa é usado pelo A*. O controlador converte posições `(x, y)` do mundo para
células do grid e vice-versa.

No planejador existem duas margens ao redor de obstáculos:

- `inflacao_obstaculo_celulas`: margem dura; essas células viram bloqueadas.
- `custo_adjacente_obstaculo`: margem suave; células livres encostadas na
  região bloqueada continuam transitáveis, mas custam mais caro.

### Odometria Ground Truth

Tópico: `/odom_gt`

Usada para saber a pose do robô na simulação. A primeira pose recebida vira a
base: depois de capturar a bandeira, o A* planeja o retorno para essa pose.

## Máquina de Estados

Estados atuais:

```text
EXPLORANDO
BANDEIRA_DETECTADA
ESTIMANDO_POSICAO_BANDEIRA
PLANEJANDO_PARA_BANDEIRA
SEGUINDO_CAMINHO_PARA_BANDEIRA
DESVIANDO_OBSTACULO
REPLANEJANDO_CAMINHO
FALHA_PLANEJAMENTO
REENCONTRANDO_BANDEIRA
POSICIONANDO_PARA_COLETA
CAPTURANDO_BANDEIRA
PLANEJANDO_RETORNO_BASE
RETORNANDO_BASE
ENTREGANDO_BANDEIRA
MISSAO_CONCLUIDA
```

Fluxo principal:

```text
EXPLORANDO
  -> BANDEIRA_DETECTADA
  -> ESTIMANDO_POSICAO_BANDEIRA
  -> PLANEJANDO_PARA_BANDEIRA
  -> SEGUINDO_CAMINHO_PARA_BANDEIRA
  -> POSICIONANDO_PARA_COLETA
  -> CAPTURANDO_BANDEIRA
  -> PLANEJANDO_RETORNO_BASE
  -> RETORNANDO_BASE
  -> ENTREGANDO_BANDEIRA
  -> MISSAO_CONCLUIDA
```

Estados de recuperação:

- `DESVIANDO_OBSTACULO` pode interromper quase qualquer estado de movimento.
  Ao sair do desvio, a máquina tenta voltar ao que fazia antes: replanejar
  rota, retomar posicionamento visual ou continuar buscando.
- `REENCONTRANDO_BANDEIRA` é usado quando um desvio perto da coleta tira a
  bandeira do centro da câmera. O robô gira parado para reenquadrar a bandeira
  antes de estimar uma nova posição ou voltar ao A*.
- `FALHA_PLANEJAMENTO` não significa fim da missão. Ele é um estado de espera e
  recuperação: o robô tenta melhorar a estimativa, receber mapa novo ou
  replanejar para o alvo congelado.

## Como a Bandeira é Capturada

1. A câmera detecta a bandeira azul.
2. `estimador_bandeira.py` calcula uma hipótese de posição no mapa.
3. O A* leva o robô para perto dessa posição, mirando um ponto antes da
   bandeira para evitar bater direto na haste.
4. Quando a bandeira ocupa área suficiente na imagem, o controle visual assume
   e o A* para de interferir no ajuste final.
5. O robô alinha a haste pelo `erro_x_haste`.
6. Com a haste alinhada, a bandeira grande o bastante
   (`area_coleta_bandeira`) e a janela central estreita do LIDAR abaixo de
   `distancia_coleta_bandeira`, a garra fecha e a haste levanta.

## Como o Retorno Funciona

Depois da captura:

1. a pose inicial salva no primeiro callback de odometria é tratada como base;
2. o A* planeja um caminho até essa pose;
3. o robô segue os waypoints usando o mesmo seguidor de caminho;
4. se aparecer obstáculo, entra em desvio e replaneja;
5. ao chegar na base, a garra abre e a haste desce para depositar a bandeira.

No retorno, a leitura central do LIDAR pode ser ignorada porque a própria
bandeira fica na frente do sensor.

## Parâmetros

Os valores padrão ficam em:

```text
controle_robo/config/missao_bandeira_azul.yaml
```

Cada grupo do YAML tem comentários. Em geral:

- parâmetros de LIDAR mexem em desvio e segurança;
- parâmetros visuais mexem em detecção, alinhamento e captura;
- parâmetros de A* mexem em custo, inflação, folga perto de obstáculo,
  tolerâncias e velocidade;
- parâmetros da garra mexem na abertura e na altura da haste;
- parâmetros de launch mexem em mundo, atrasos e nomes de tópicos.

Quando for ajustar comportamento, prefira primeiro mudar o YAML. Mexa no código
só quando o parâmetro existente não representar a regra que você precisa.

## Como Validar Alterações

Antes de testar no Gazebo:

```bash
python3 -m py_compile src/controle_robo/controle_robo/*.py src/controle_robo/launch/*.launch.py
pytest -q src/controle_robo/test/test_estimador_bandeira.py src/controle_robo/test/test_detector_bandeira.py src/controle_robo/test/test_planejador_grade.py
colcon build --symlink-install --packages-select robo_movel controle_robo
```

Durante a simulação, os tópicos mais úteis para debug são:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /bandeira_azul/debug_info
ros2 topic echo /bandeira_azul/alvo_estimado --once
ros2 topic echo /caminho_planejado --once
ros2 topic echo /gripper_controller/commands
```

No RViz, adicione `/grid_map`, `/caminho_planejado` e
`/bandeira_azul/alvo_estimado` para ver se a estimativa e o A* fazem sentido.
