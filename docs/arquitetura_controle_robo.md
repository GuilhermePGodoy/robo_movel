# Arquitetura do controle da missao

Este documento explica a parte que foi implementada no pacote
`controle_robo`. A ideia e ajudar alguem novo no projeto a entender onde cada
decisao mora antes de mudar o codigo.

## Visao geral

O sistema ficou dividido em dois pacotes:

- `robo_movel`: sobe a simulacao, o modelo do robo, os sensores, os
  controladores e o mapa `/grid_map`.
- `controle_robo`: detecta a bandeira azul, decide a missao, planeja caminhos
  com A*, desvia de obstaculos e controla a garra.

O launch principal e:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py
```

Ele le `controle_robo/config/missao_bandeira_azul.yaml`, sobe a simulacao do
pacote `robo_movel`, espera o robo nascer e inicia os nos de missao.

## Arquivos principais

- `controle_robo.py`
  - E o no ROS principal.
  - Declara parametros, cria publishers/subscribers e guarda a ultima leitura
    de cada sensor.
  - Nao deveria concentrar regras grandes da missao. Quando uma regra crescer,
    vale mover para um modulo menor.

- `maquina_estados.py`
  - Contem a maquina de estados da missao.
  - Le o estado atual dos sensores guardado em `ControleRobo`.
  - Decide quando publicar velocidades, abrir/fechar garra, planejar ou mudar
    de estado.

- `detector_bandeira.py`
  - Recebe a imagem semantica `/robot_cam/labels_map`.
  - Procura pixels com a label `25`, que representam a bandeira azul.
  - Publica `/bandeira_azul/deteccao` como um vetor numerico simples.

- `visao_bandeira.py`
  - Funcoes auxiliares para extrair informacoes visuais da bandeira.
  - Inclui a estimativa da posicao da haste, que e melhor para alinhar a garra
    do que o centro da bounding box inteira.

- `criterios_visuais.py`
  - Heuristicas pequenas sobre a deteccao visual.
  - Exemplos: "tem bandeira parcial?", "tem bandeira suficiente para
    posicionamento?", "a bandeira parece inteira por alguns frames?".

- `estimador_bandeira.py`
  - Usa o tamanho da bbox na camera para estimar distancia.
  - Usa o deslocamento horizontal da haste/bbox para estimar angulo relativo.
  - Transforma essa hipotese em coordenada do mapa usando a pose do robo.

- `planejador_grade.py`
  - Implementa A* em cima do `nav_msgs/OccupancyGrid`.
  - Celulas livres custam pouco, desconhecidas custam mais, ocupadas bloqueiam.
  - Tambem infla obstaculos para o robo nao passar raspando.

- `lidar.py`
  - Organiza o `LaserScan` em regioes: frente, frente sem centro, esquerda e
    direita.
  - Escolhe o lado de desvio olhando qual lado/frente esta mais livre.
  - A leitura "frente sem centro" e usada no retorno, porque a bandeira presa
    na garra aparece no centro do LIDAR.

- `garra.py`
  - Esconde os detalhes do vetor enviado para
    `/gripper_controller/commands`.
  - A ordem do vetor e `[haste, garra_direita, garra_esquerda]`.

- `launch_parametros.py`
  - Lista unica dos argumentos dos launchers.
  - Evita duplicar dezenas de `DeclareLaunchArgument` em mais de um arquivo.

## Sensores usados

### Camera semantica

Topico: `/robot_cam/labels_map`

Cada pixel da imagem carrega uma classe semantica do Gazebo. O detector usa a
label `25` para achar a bandeira azul. A partir dessa regiao, ele calcula:

- centro da bbox;
- centro aproximado da haste;
- erro horizontal normalizado;
- area relativa da bandeira na imagem;
- largura e altura da bbox.

A haste e importante porque a garra precisa chegar no poste, nao no centro do
painel azul.

### LIDAR

Topico: `/scan`

O LIDAR e a seguranca local. Ele responde perguntas como:

- tem algo perto na frente?
- qual lado parece mais livre para desviar?
- depois da captura, ignorando o centro do scan, ainda tem obstaculo real na
  frente?

Durante o retorno para a base, a bandeira fica na frente do robo e poderia ser
confundida com obstaculo. Por isso `lidar.py` tambem calcula a frente sem a
janela central.

### Mapa

Topico: `/grid_map`

O mapa e usado pelo A*. O controlador converte posicoes `(x, y)` do mundo para
celulas do grid e vice-versa.

### Odometria ground truth

Topico: `/odom_gt`

Usada para saber a pose do robo na simulacao. A primeira pose recebida vira a
base: depois de capturar a bandeira, o A* planeja o retorno para essa pose.

## Maquina de estados

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

`DESVIANDO_OBSTACULO` pode interromper quase qualquer estado de movimento. Ao
sair do desvio, a maquina tenta voltar ao que fazia antes: replanejar rota,
retomar posicionamento visual ou continuar buscando.

`FALHA_PLANEJAMENTO` nao significa fim da missao. Ele e um estado de espera e
recuperacao: o robo tenta melhorar a estimativa, receber mapa novo ou replanejar
para o alvo congelado.

## Como a bandeira e capturada

1. A camera detecta a bandeira azul.
2. `estimador_bandeira.py` calcula uma hipotese de posicao no mapa.
3. O A* leva o robo para perto dessa posicao, mirando um ponto antes da
   bandeira para evitar bater direto na haste.
4. Quando a imagem mostra bandeira suficiente e pouco obstaculo central, o
   controle visual assume.
5. O robo alinha a haste pelo `erro_x_haste`.
6. Com a haste alinhada e o LIDAR frontal abaixo de
   `distancia_coleta_bandeira`, a garra fecha e a haste levanta.

## Como o retorno funciona

Depois da captura:

1. a pose inicial salva no primeiro callback de odometria e tratada como base;
2. o A* planeja um caminho ate essa pose;
3. o robo segue os waypoints usando o mesmo seguidor de caminho;
4. se aparecer obstaculo, entra em desvio e replaneja;
5. ao chegar na base, a garra abre e a haste desce para depositar a bandeira.

No retorno, a leitura central do LIDAR pode ser ignorada porque a propria
bandeira fica na frente do sensor.

## Parametros

Os valores padrao ficam em:

```text
controle_robo/config/missao_bandeira_azul.yaml
```

Cada grupo do YAML tem comentarios. Em geral:

- parametros de LIDAR mexem em desvio e seguranca;
- parametros visuais mexem em deteccao, alinhamento e captura;
- parametros de A* mexem em custo, inflacao, tolerancias e velocidade;
- parametros da garra mexem na abertura e na altura da haste;
- parametros de launch mexem em mundo, atrasos e nomes de topicos.

Quando for ajustar comportamento, prefira primeiro mudar o YAML. Mexa no codigo
so quando o parametro existente nao representar a regra que voce precisa.

## Como validar alteracoes

Antes de testar no Gazebo:

```bash
python3 -m py_compile src/controle_robo/controle_robo/*.py src/controle_robo/launch/*.launch.py
pytest -q src/controle_robo/test/test_estimador_bandeira.py src/controle_robo/test/test_detector_bandeira.py src/controle_robo/test/test_planejador_grade.py
colcon build --symlink-install --packages-select robo_movel controle_robo
```

Durante a simulacao, os topicos mais uteis para debug sao:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /bandeira_azul/debug_info
ros2 topic echo /bandeira_azul/alvo_estimado --once
ros2 topic echo /caminho_planejado --once
ros2 topic echo /gripper_controller/commands
```

No RViz, adicione `/grid_map`, `/caminho_planejado` e
`/bandeira_azul/alvo_estimado` para ver se a estimativa e o A* fazem sentido.


