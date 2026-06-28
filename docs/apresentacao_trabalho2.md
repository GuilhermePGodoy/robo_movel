# Trabalho 2 - Robô que Procura, Pega e Leva a Bandeira Azul

Este roteiro resume o sistema desenvolvido para a apresentação do Trabalho 2.
Ele pode ser usado como apoio para explicar o projeto em formato de slides ou
pôster.

## Ideia do Projeto

O objetivo foi construir, em simulação, um robô móvel capaz de:

- andar sozinho pela arena;
- enxergar a bandeira azul;
- montar um mapa simples dos obstáculos;
- planejar caminho até a bandeira;
- desviar de objetos no caminho;
- agarrar a bandeira com a garra;
- voltar para a base inicial;
- soltar a bandeira no chão.

O projeto usa ROS 2 Jazzy e Gazebo Sim.

## Origem

O trabalho foi desenvolvido a partir do template disponibilizado pelo professor
no repositório [matheusbg8/prm_2026](https://github.com/matheusbg8/prm_2026).
Esse template forneceu a base inicial da simulação, principalmente o pacote que
neste repositório chamamos de `robo_movel`.

A partir dessa base, criamos e evoluímos o pacote `controle_robo`, responsável
pela autonomia da missão.

## Pacotes do Projeto

### `robo_movel`

Responsável pela parte de simulação:

- modelo do robô;
- mundos do Gazebo;
- sensores;
- controladores;
- ponte entre Gazebo e ROS;
- mapper que publica o mapa `/grid_map`.

### `controle_robo`

Responsável pela inteligência da missão:

- detector da bandeira azul;
- máquina de estados;
- estimativa da posição da bandeira;
- planejamento A*;
- desvio de obstáculos;
- controle da garra;
- retorno para a base.

## Sensores

### Câmera Semântica

A câmera não vê uma imagem comum. Ela publica uma imagem semântica, em que cada
pixel representa a classe do objeto visto.

No nosso caso:

- a bandeira azul tem label `25`;
- o detector procura pixels com essa label;
- depois calcula a posição aproximada da bandeira e da haste.

A haste é importante porque a garra precisa mirar no poste, não no meio do
tecido azul.

### LIDAR

O LIDAR mede distância ao redor do robô.

Ele é usado para:

- detectar obstáculos na frente;
- escolher se é melhor desviar pela esquerda ou pela direita;
- evitar passar raspando nos objetos;
- ajudar a saber quando a bandeira está perto o bastante para fechar a garra.

Depois que o robô pega a bandeira, a própria bandeira aparece no centro do
LIDAR. Por isso, no retorno, o controle ignora essa faixa central e presta mais
atenção nas laterais.

### Odometria

A odometria informa a posição do robô na simulação.

A primeira posição recebida é salva como base. Depois de pegar a bandeira, o
robô usa essa pose para planejar o caminho de volta.

## Mapa

O robô monta um mapa em grade, publicado em `/grid_map`.

Cada célula pode ser:

- `-1`: desconhecida;
- `0`: livre;
- `100`: ocupada por obstáculo.

Quando o robô confirma um obstáculo, ele mantém essa informação no mapa. Isso
ajuda o A* a evitar caminhos que passam por objetos já descobertos.

Depois que a bandeira é capturada, o mapper é congelado. Isso evita que a
bandeira carregada seja confundida com novos obstáculos no mapa.

## Planejamento com A*

O A* é usado para encontrar caminhos no mapa.

Ele ajuda em três momentos:

- aproximar o robô da bandeira;
- explorar regiões desconhecidas quando a bandeira fica escondida;
- voltar para a base depois da captura.

O planejador evita células ocupadas, aceita células desconhecidas com custo
maior e prefere caminhos com mais distância dos obstáculos.

## Máquina de Estados

A missão é organizada por estados. Isso torna o comportamento mais fácil de
entender e depurar.

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

- `DESVIANDO_OBSTACULO`: usado quando o LIDAR detecta algo perigoso.
- `REENCONTRANDO_BANDEIRA`: usado quando a bandeira sai da câmera perto da
  coleta.
- `REPLANEJANDO_CAMINHO`: recalcula o A* quando o caminho fica ruim.
- `PLANEJANDO_EXPLORACAO_DESCONHECIDA`: procura uma região ainda não vista do
  mapa quando a bandeira não aparece por muito tempo.

## Como o Robô Pega a Bandeira

1. A câmera encontra a bandeira azul.
2. O sistema calcula uma estimativa da posição da bandeira no mapa.
3. O A* leva o robô até perto.
4. Quando a bandeira já está próxima o suficiente, o robô para de seguir o A*.
5. O controle passa a usar a câmera para alinhar a haste no centro.
6. O LIDAR confirma a distância final.
7. A garra fecha.
8. A haste levanta a bandeira.

## Como o Robô Volta

1. A base é a posição onde o robô nasceu.
2. Depois da captura, o A* calcula um caminho até essa base.
3. O robô segue os waypoints.
4. Se encontra obstáculo, desvia e replaneja.
5. Na base, a haste desce devagar.
6. A garra abre.
7. O robô dá uma pequena ré para deixar a bandeira no chão.

## Resultado

O robô consegue executar a missão completa em simulação: detectar a bandeira,
aproximar, agarrar, retornar e depositar.

O comportamento ainda depende bastante do mapa, da posição inicial dos
obstáculos e de como a bandeira aparece na câmera. Em alguns cenários, a
heurística visual pode estimar a posição com erro ou demorar para reencontrar a
bandeira. Mesmo assim, a solução mostra uma integração funcional entre visão,
LIDAR, mapa, A*, máquina de estados e controle da garra.
