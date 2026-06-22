# Trabalho 2 - Sistema Autônomo de Captura da Bandeira Azul

## Objetivo

Implementar um robô móvel autônomo em ROS 2 capaz de:

- Explorar a arena simulada no Gazebo.
- Detectar a bandeira azul por visão computacional semântica.
- Desviar de obstáculos usando LIDAR.
- Estimar a posição da bandeira no mapa, planejar caminho com A*, capturá-la
  e retornar para a base inicial.

## Arquitetura

O projeto foi dividido em dois pacotes:

- `robo_movel`
  - URDF/Xacro do robô diferencial.
  - Sensores: LIDAR, câmera semântica e IMU.
  - Launches de simulação, spawn, bridges e controladores.
  - Mapper simples com `OccupancyGrid`.
- `controle_robo`
  - Detector da bandeira azul.
  - Nó ROS de controle, com parâmetros e leituras recentes dos sensores.
  - Máquina de estados da missão em arquivo separado.
  - Estimador geométrico da posição da bandeira.
  - Planejador A* sobre o mapa de ocupação.
  - Modelos simples para estado e detecção visual.
  - Launch principal da missão.
  - YAML de configuração.

## Percepção Visual

A câmera semântica do Gazebo publica `labels_map`.

- Cada pixel contém a label semântica do objeto visto.
- Bandeira vermelha: label `20`.
- Bandeira azul: label `25`.
- Obstáculos e outras regiões usam labels diferentes.

O nó `detector_bandeira` filtra a label `25`, encontra o maior blob válido e
publica:

- erro horizontal normalizado da bandeira;
- erro horizontal da haste, usado para mirar a garra;
- área relativa da bandeira na imagem;
- centro e tamanho da caixa detectada.

Com o tamanho real aproximado da bandeira e o campo de visão da câmera, o
controle estima distância, ângulo e posição `(x, y)` da bandeira no mapa.

## Percepção por LIDAR

O LIDAR publica `/scan`.

O controle usa algumas regiões:

- frente: decide se há obstáculo no caminho;
- frente estreita: ajuda a decidir quando a haste está perto o bastante para
  fechar a garra;
- frente sem centro: usada no retorno, porque a bandeira carregada fica no
  meio do LIDAR;
- esquerda e direita: medem espaço livre para desvio.

Quando a frente fica bloqueada, o robô gira para o lado com maior distância
livre. A mesma leitura lateral também impede que o robô retome a missão
enquanto ainda estiver muito perto de um obstáculo ao lado.

## Mapa

O nó `robo_mapper` publica `/grid_map`.

- `-1`: célula desconhecida.
- `0`: célula livre observada por LIDAR.
- `100`: célula ocupada ou posição atual do robô.

O mapa também é usado pelo A*. Células ocupadas são bloqueadas, células
desconhecidas são permitidas com custo maior e os obstáculos são inflados para
evitar caminhos raspando nas bordas. Células livres adjacentes aos obstáculos
continuam possíveis, mas recebem custo maior.

## Máquina de Estados

Implementada em `controle_robo/controle_robo/maquina_estados.py`. O arquivo
`controle_robo.py` fica mais enxuto e cuida apenas da parte ROS: parâmetros,
subscribers, publishers e timer de controle.

Estados implementados:

- `EXPLORANDO`
  - Avança em curva suave.
  - Faz varredura com a câmera sem assumir a posição da bandeira.
- `BANDEIRA_DETECTADA`
  - Confirma a detecção e escolhe a próxima ação.
- `ESTIMANDO_POSICAO_BANDEIRA`
  - Calcula uma hipótese da posição da bandeira no mapa.
- `PLANEJANDO_PARA_BANDEIRA`
  - Roda A* até uma célula livre perto da bandeira estimada.
- `SEGUINDO_CAMINHO_PARA_BANDEIRA`
  - Segue waypoints e replaneja quando necessário.
- `DESVIANDO_OBSTACULO`
  - Usa LIDAR para girar para o lado mais livre.
  - Só termina quando há folga frontal e lateral.
- `REENCONTRANDO_BANDEIRA`
  - Após um desvio perto da coleta, gira parado para reenquadrar a bandeira.
- `REPLANEJANDO_CAMINHO` e `FALHA_PLANEJAMENTO`
  - Tentam recuperar a rota quando o mapa muda ou o A* falha.
- `POSICIONANDO_PARA_COLETA`
  - Ajusta orientação e aproxima devagar.
- `CAPTURANDO_BANDEIRA`
  - Para o robô, fecha a garra e levanta a haste.
- `PLANEJANDO_RETORNO_BASE` e `RETORNANDO_BASE`
  - Usam A* para voltar até a pose inicial do robô.
- `ENTREGANDO_BANDEIRA` e `MISSAO_CONCLUIDA`
  - Abrem a garra na base e encerram a missão.

## Estratégia de Navegação

A solução principal é híbrida:

- busca reativa por curva/varredura de câmera;
- detecção visual da label correta;
- estimativa geométrica da posição da bandeira;
- planejamento A* no `/grid_map` para aproximação e retorno;
- controle proporcional pelo erro horizontal da haste no ajuste fino;
- desvio local de obstáculos por LIDAR;
- redetecção caso a bandeira saia do campo de visão durante uma manobra.

## Pontos Fortes

- Separação clara entre simulação/modelagem e controle.
- Detector visual dedicado, facilitando debug.
- Máquina de estados documentada no código.
- Parâmetros concentrados em YAML.
- Logs explicativos para entender o estado atual do robô.
- Caminho planejado e alvo estimado disponíveis para debug no RViz.

## Limitações e Próximos Passos

- A captura física depende do ajuste fino da posição final no Gazebo.
- A posição da bandeira vem de uma estimativa visual, então pode variar quando
  a bandeira está cortada, longe ou parcialmente escondida.
- O A* aproxima o robô do alvo; a câmera ainda é essencial no ajuste final.
