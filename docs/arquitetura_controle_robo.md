# Arquitetura do Controle do Robô

Este documento explica a organização do projeto na versão final do Trabalho 2.
A intenção é deixar claro onde cada decisão mora, para que outra pessoa consiga
entrar no código sem precisar reconstruir todo o histórico de desenvolvimento.

## Origem do Projeto

O projeto foi desenvolvido a partir do template disponibilizado pelo professor
no repositório [matheusbg8/prm_2026](https://github.com/matheusbg8/prm_2026).
O pacote `robo_movel` deste repositório é uma evolução direta dessa base:
mantém a ideia original de simulação, robô, sensores e mundos, mas foi adaptado
e expandido para o fluxo usado neste trabalho.

O pacote `controle_robo` concentra o desenvolvimento da missão autônoma:
detecção da bandeira, máquina de estados, planejamento, desvio, captura e
retorno à base.

## Visão Geral

O sistema está dividido em dois pacotes ROS 2:

- `robo_movel`: simulação, modelo do robô, sensores, controladores, mundos,
  odometria ground truth e mapper.
- `controle_robo`: percepção da bandeira, controle da missão, planejamento A*,
  controle da garra e launches de execução.

O launch principal é:

```bash
ros2 launch controle_robo missao_bandeira_azul.launch.py
```

Ele lê `controle_robo/config/missao_bandeira_azul.yaml`, sobe a simulação do
pacote `robo_movel`, espera o robô nascer e inicia os nós da missão.

## Arquivos Principais

- `controle_robo.py`
  - Nó ROS principal.
  - Declara parâmetros, cria publishers/subscribers e guarda a leitura mais
    recente de cada sensor.
  - Também publica comandos de velocidade, garra, congelamento do mapper,
    caminho planejado e alvo estimado.

- `maquina_estados.py`
  - Coração da missão.
  - Decide transições de estado, comandos de movimento, uso do A*, comandos de
    garra e recuperação em caso de obstáculo ou perda visual da bandeira.

- `modelos_missao.py`
  - Define o enum `EstadoMissao`.
  - Define estruturas simples para detecção visual e estimativa da bandeira.

- `detector_bandeira.py`
  - Recebe `/robot_cam/labels_map`.
  - Procura a label semântica `25`, correspondente à bandeira azul.
  - Publica `/bandeira_azul/deteccao`, `/bandeira_azul/debug_info` e
    `/bandeira_azul/debug_mask`.

- `criterios_visuais.py`
  - Agrupa regras pequenas de visão.
  - Decide se uma bbox é boa para estimar a bandeira e se a bandeira visível já
    é útil para entrar no posicionamento de coleta.

- `estimador_bandeira.py`
  - Usa trigonometria da câmera para transformar bbox em distância, ângulo e
    posição `(x, y)` no mapa.
  - Mantém a melhor estimativa visual: leituras válidas mais próximas, com bbox
    mais alta e coerente, substituem leituras antigas.

- `planejador_grade.py`
  - Implementa A* sobre `nav_msgs/OccupancyGrid`.
  - Bloqueia células ocupadas, permite células desconhecidas com custo maior e
    aumenta o custo de regiões próximas de obstáculos.

- `lidar.py`
  - Organiza o `LaserScan` em regiões úteis: frente, frente sem centro, faixa
    estreita de coleta, esquerda e direita.
  - Escolhe o lado de desvio com base na folga real disponível.

- `garra.py`
  - Esconde o formato do vetor enviado para `/gripper_controller/commands`.
  - Centraliza comandos como abrir, fechar, levantar, baixar e depositar.

- `launch_parametros.py`
  - Lista única de argumentos dos launchers.
  - Evita duplicar `DeclareLaunchArgument` em vários arquivos.

## Percepção Visual

A câmera semântica publica `/robot_cam/labels_map`. Cada pixel contém uma label
do objeto visto no Gazebo. O detector filtra a label `25` e calcula:

- centro do maior blob azul;
- centro aproximado da haste;
- erro horizontal da bbox;
- erro horizontal da haste;
- área relativa;
- largura e altura da bbox;
- `fill_ratio`, isto é, quanto da bbox realmente está preenchido por pixels da
  bandeira;
- proporção largura/altura da bbox.

A posição da haste é mais importante que o centro da bbox inteira, porque a
garra precisa agarrar o poste, não o meio do tecido.

Para aceitar uma estimativa geométrica, o código não usa qualquer blob azul. A
bbox precisa passar por filtros simples de `fill_ratio` e proporção. Isso evita
casos em que o robô enxerga só um pedaço do tecido ou só um recorte estranho da
haste e joga a estimativa para muito longe.

## LIDAR

O LIDAR publica `/scan` e serve como segurança local. Ele responde perguntas
como:

- existe obstáculo na frente?
- qual lado tem mais espaço para desviar?
- a haste da bandeira está logo à frente da garra?
- durante o retorno, ignorando a faixa central ocupada pela bandeira carregada,
  ainda existe obstáculo real no caminho?

Durante a coleta, a faixa central estreita do LIDAR ajuda a decidir quando
fechar a garra. Durante o retorno, essa faixa central pode ser ignorada, porque
a bandeira capturada fica na frente do robô e aparece como obstáculo no sensor.

## Mapa e Mapper

O mapper publica `/grid_map` como `nav_msgs/msg/OccupancyGrid`:

- `-1`: célula desconhecida.
- `0`: célula livre observada.
- `100`: célula ocupada confirmada.

Obstáculos são persistentes. Quando o LIDAR confirma uma célula ocupada, ela
continua ocupada no mapa; raios livres posteriores não apagam esse obstáculo.
Isso deixa o planejamento mais estável em arenas com objetos fixos.

A posição atual do robô não é marcada como obstáculo no grid. Isso evita que o
planejador tente fugir da própria célula do robô.

Após capturar a bandeira, o controle publica `true` em `/mapper/congelar`. O
mapper passa a republicar o último mapa conhecido sem integrar novas leituras.
Esse congelamento evita que a bandeira presa na garra seja registrada como um
rastro de obstáculos durante o retorno.

## Planejamento A*

O A* trabalha sobre o `/grid_map`:

- células `100` são bloqueadas;
- células `0` têm custo normal;
- células `-1` são permitidas com custo maior, permitindo exploração;
- células próximas de obstáculos recebem custo adicional;
- a inflação de obstáculos pode bloquear uma margem dura ao redor deles.

O mesmo planejador é usado para:

- ir até a região estimada da bandeira;
- explorar uma fronteira desconhecida quando a bandeira fica escondida;
- retornar para a base depois da captura.

O seguidor de waypoints evita escolher um ponto de seguimento mais perto que
`tolerancia_waypoint`. Esse detalhe reduz loops em que o robô fica em cima de
um waypoint, tenta mirar o próximo quase colado e começa a girar sem avançar.

## Máquina de Estados

Estados atuais:

```text
EXPLORANDO
BANDEIRA_DETECTADA
ESTIMANDO_POSICAO_BANDEIRA
PLANEJANDO_PARA_BANDEIRA
SEGUINDO_CAMINHO_PARA_BANDEIRA
PLANEJANDO_EXPLORACAO_DESCONHECIDA
SEGUINDO_CAMINHO_EXPLORACAO
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

- `DESVIANDO_OBSTACULO`: interrompe estados de movimento quando o LIDAR acusa
  risco de colisão. Ao terminar, replaneja ou volta ao ajuste visual, conforme
  o que estava acontecendo antes.
- `REENCONTRANDO_BANDEIRA`: usado perto da coleta. Se um desvio tira a
  bandeira da câmera, o robô gira parado tentando reenquadrá-la.
- `FALHA_PLANEJAMENTO`: espera mapa novo, melhora estimativa ou tenta
  replanejar para o alvo conhecido. Não encerra a missão.
- `PLANEJANDO_EXPLORACAO_DESCONHECIDA`: busca uma fronteira do mapa quando a
  bandeira não aparece por muito tempo.

## Captura da Bandeira

O processo de captura é dividido em etapas:

1. A câmera detecta a bandeira azul.
2. O estimador calcula a melhor hipótese de posição no mapa.
3. O A* aproxima o robô da região estimada.
4. Quando a bandeira está útil para posicionamento, o A* para de mandar no
   ajuste final.
5. O robô alinha a haste usando `erro_x_haste`.
6. O robô avança devagar, usando a faixa central do LIDAR para saber quando a
   haste está próxima da garra.
7. A garra fecha e a haste levanta.
8. O mapper é congelado.

## Retorno e Entrega

A primeira pose recebida em `/odom_gt` é salva como base. Depois da captura:

1. O A* planeja um caminho até essa base.
2. O robô segue os waypoints com uma distância de obstáculo própria para o
   retorno, um pouco mais conservadora.
3. A faixa central do LIDAR é ignorada para não confundir a bandeira carregada
   com obstáculo.
4. Ao chegar na base, a haste desce gradualmente.
5. A garra abre.
6. O robô dá uma pequena ré.
7. A missão entra em `MISSAO_CONCLUIDA`.

## Parâmetros

Os valores padrão ficam em:

```text
controle_robo/config/missao_bandeira_azul.yaml
```

O YAML está comentado por grupos:

- simulação e launch;
- tópicos ROS;
- busca e exploração;
- LIDAR e desvio;
- detecção visual;
- estimativa da bandeira;
- A* e seguimento de caminho;
- garra e entrega.

Quando for ajustar o comportamento, prefira primeiro mudar o YAML. Mexa no
código quando a regra nova não couber em um parâmetro existente.

## Como Validar Alterações

Antes de testar no Gazebo:

```bash
cd ~/coding/usp_grad/robos_moveis/ros2_ws
source /opt/ros/jazzy/setup.bash
python3 -m py_compile src/controle_robo/controle_robo/*.py src/controle_robo/launch/*.launch.py src/robo_movel/robo_movel/*.py
colcon build --symlink-install --packages-select robo_movel controle_robo
source install/setup.bash
```

Durante a simulação, os tópicos mais úteis para debug são:

```bash
ros2 topic echo /bandeira_azul/deteccao
ros2 topic echo /bandeira_azul/debug_info
ros2 topic echo /bandeira_azul/alvo_estimado --once
ros2 topic echo /caminho_planejado --once
ros2 topic echo /mapper/congelar
ros2 topic echo /gripper_controller/commands
```

No RViz, adicione `/grid_map`, `/caminho_planejado` e
`/bandeira_azul/alvo_estimado` para verificar se o mapa, a estimativa e o A*
fazem sentido.

Testes locais podem ser criados em `controle_robo/test/` durante o
desenvolvimento. Essa pasta fica ignorada pelo Git para não misturar testes
temporários com a entrega.
