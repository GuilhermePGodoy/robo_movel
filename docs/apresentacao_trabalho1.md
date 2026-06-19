# Trabalho 1 - Sistema Autonomo de Captura da Bandeira Azul

## Objetivo

Implementar um robo movel autonomo em ROS 2 capaz de:

- Explorar a arena simulada no Gazebo.
- Detectar a bandeira azul por visao computacional semantica.
- Desviar de obstaculos usando LIDAR.
- Estimar a posicao da bandeira no mapa, planejar caminho com A*, captura-la
  e retornar para a base inicial.

## Arquitetura

O projeto foi dividido em dois pacotes:

- `robo_movel`
  - URDF/Xacro do robo diferencial.
  - Sensores: LIDAR, camera semantica e IMU.
  - Launches de simulacao, spawn, bridges e controladores.
  - Mapper simples com `OccupancyGrid`.
- `controle_robo`
  - Detector da bandeira azul.
  - No ROS de controle, com parametros e leituras recentes dos sensores.
  - Maquina de estados da missao em arquivo separado.
  - Estimador geometrico da posicao da bandeira.
  - Planejador A* sobre o mapa de ocupacao.
  - Modelos simples para estado e deteccao visual.
  - Launch principal da missao.
  - YAML de configuracao.

## Percepcao Visual

A camera semantica do Gazebo publica `labels_map`.

- Cada pixel contem a label semantica do objeto visto.
- Bandeira vermelha: label `20`.
- Bandeira azul: label `25`.
- Obstaculos e outras regioes usam labels diferentes.

O no `detector_bandeira` filtra a label `25`, encontra o maior blob valido e
publica:

- Erro horizontal normalizado da bandeira.
- Area relativa da bandeira na imagem.
- Centro e tamanho da caixa detectada.

Com o tamanho real aproximado da bandeira e o campo de visao da camera, o
controle estima distancia, angulo e posicao `(x, y)` da bandeira no mapa.

## Percepcao por LIDAR

O LIDAR publica `/scan`.

O controle usa tres regioes:

- Frente: decide se ha obstaculo no caminho.
- Esquerda: mede espaco livre para desvio.
- Direita: mede espaco livre para desvio.

Quando a frente fica bloqueada, o robo gira para o lado com maior distancia
livre. A mesma leitura lateral tambem impede que o robo retome a missao
enquanto ainda estiver muito perto de um obstaculo ao lado.

## Mapa

O no `robo_mapper` publica `/grid_map`.

- `-1`: celula desconhecida.
- `0`: celula livre observada por LIDAR.
- `100`: celula ocupada ou posicao atual do robo.

O mapa agora tambem e usado pelo A*. Celulas ocupadas sao bloqueadas, celulas
desconhecidas sao permitidas com custo maior e os obstaculos sao inflados para
evitar caminhos raspando nas bordas.

## Maquina de Estados

Implementada em `controle_robo/controle_robo/maquina_estados.py`. O arquivo
`controle_robo.py` fica mais enxuto e cuida apenas da parte ROS: parametros,
subscribers, publishers e timer de controle.

Estados implementados:

- `EXPLORANDO`
  - Avanca em curva suave.
  - Faz varredura com a camera sem assumir a posicao da bandeira.
- `BANDEIRA_DETECTADA`
  - Confirma a deteccao e escolhe a proxima acao.
- `ESTIMANDO_POSICAO_BANDEIRA`
  - Calcula uma hipotese da posicao da bandeira no mapa.
- `PLANEJANDO_PARA_BANDEIRA`
  - Roda A* ate uma celula livre perto da bandeira estimada.
- `SEGUINDO_CAMINHO_PARA_BANDEIRA`
  - Segue waypoints e replaneja quando necessario.
- `DESVIANDO_OBSTACULO`
  - Usa LIDAR para girar para o lado mais livre.
  - So termina quando ha folga frontal e lateral.
- `REDETECTANDO_BANDEIRA`
  - Gira na direcao da ultima deteccao se a bandeira sumir.
- `POSICIONANDO_PARA_COLETA`
  - Ajusta orientacao e aproxima devagar.
- `CAPTURANDO_BANDEIRA`
  - Para o robo e fecha a garra.
- `PLANEJANDO_RETORNO_BASE` e `RETORNANDO_BASE`
  - Usam A* para voltar ate a pose inicial do robo.
- `ENTREGANDO_BANDEIRA` e `MISSAO_CONCLUIDA`
  - Abrem a garra na base e encerram a missao.

## Estrategia de Navegacao

A solucao principal e hibrida:

- Busca reativa por curva/varredura de camera.
- Deteccao visual da label correta.
- Estimativa geometrica da posicao da bandeira.
- Planejamento A* no `/grid_map` para aproximacao e retorno.
- Controle proporcional pelo erro horizontal da imagem no ajuste fino.
- Desvio local de obstaculos por LIDAR.
- Redeteccao caso a bandeira saia do campo de visao.

## Pontos Fortes

- Separacao clara entre simulacao/modelagem e controle.
- Detector visual dedicado, facilitando debug.
- Maquina de estados documentada no codigo.
- Parametros concentrados em YAML.
- Logs explicativos para entender o estado atual do robo.
- Caminho planejado e alvo estimado disponiveis para debug no RViz.

## Limitacoes e Proximos Passos

- A captura fisica depende do ajuste fino da posicao final no Gazebo.
- A posicao da bandeira vem de uma estimativa visual, entao pode variar quando
  a bandeira esta cortada, longe ou parcialmente escondida.
- O A* aproxima o robo do alvo; a camera ainda e essencial no ajuste final.
