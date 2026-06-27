"""Planejamento simples em grade para o mapa OccupancyGrid.

O mapa do trabalho e pequeno, entao da para manter o A* direto e legivel.
Desconhecido nao e proibido: ele so custa mais caro. Isso deixa o robo
arriscar atravessar regioes ainda nao vistas quando nao existe rota totalmente
observada.
"""

from dataclasses import dataclass
import heapq
import math

from controle_robo.modelos_missao import ResultadoPlanejamento


@dataclass(frozen=True)
class Celula:
    x: int
    y: int


class MapaGrade:
    """Adaptador pequeno em cima de nav_msgs/OccupancyGrid."""

    def __init__(self, msg):
        self.resolution = float(msg.info.resolution)
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = list(msg.data)

    def world_to_grid(self, x: float, y: float):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return Celula(gx, gy)

    def grid_to_world(self, celula: Celula):
        x = self.origin_x + (celula.x + 0.5) * self.resolution
        y = self.origin_y + (celula.y + 0.5) * self.resolution
        return x, y

    def celula_valida(self, celula: Celula):
        return 0 <= celula.x < self.width and 0 <= celula.y < self.height

    def indice(self, celula: Celula):
        return celula.y * self.width + celula.x

    def valor(self, celula: Celula):
        if not self.celula_valida(celula):
            return 100
        return self.data[self.indice(celula)]


class PlanejadorGrade:
    """A* em cima do OccupancyGrid publicado pelo mapper."""

    def __init__(
        self,
        custo_desconhecido: float = 3.0,
        inflacao_obstaculo_celulas: int = 1,
        custo_adjacente_obstaculo: float = 2.0,
    ):
        self.custo_desconhecido = max(1.0, float(custo_desconhecido))
        self.inflacao_obstaculo_celulas = max(
            0,
            int(inflacao_obstaculo_celulas),
        )
        self.custo_adjacente_obstaculo = max(
            1.0,
            float(custo_adjacente_obstaculo),
        )

    def planejar(
        self,
        mapa_msg,
        inicio_xy,
        alvo_xy,
        yaw_inicial=None,
        peso_orientacao_inicial=0.0,
        distancia_orientacao_inicial=0.0,
    ):
        mapa = MapaGrade(mapa_msg)
        inicio = mapa.world_to_grid(*inicio_xy)
        alvo = mapa.world_to_grid(*alvo_xy)

        if not mapa.celula_valida(inicio):
            return ResultadoPlanejamento(
                motivo='pose atual fora dos limites do mapa',
            )
        if not mapa.celula_valida(alvo):
            return ResultadoPlanejamento(
                motivo='alvo fora dos limites do mapa',
            )

        bloqueadas = self.criar_mascara_bloqueada(mapa)
        bloqueadas_para_destino = set(bloqueadas)
        self.liberar_vizinhanca_do_robo(bloqueadas, inicio)
        adjacentes_obstaculo = self.criar_mascara_adjacente_obstaculo(
            mapa,
            bloqueadas,
        )

        destino = self.celula_livre_mais_proxima(
            mapa,
            alvo,
            bloqueadas_para_destino,
        )
        if destino is None:
            return ResultadoPlanejamento(
                motivo='nao ha celula livre perto do alvo estimado',
            )
        bloqueadas.discard(destino)

        caminho, custo = self.executar_a_estrela(
            mapa,
            inicio,
            destino,
            bloqueadas,
            adjacentes_obstaculo,
            yaw_inicial,
            peso_orientacao_inicial,
            distancia_orientacao_inicial,
        )
        if not caminho:
            return ResultadoPlanejamento(
                motivo='A* nao encontrou caminho ate o alvo',
            )

        caminho = self.compactar_caminho(caminho)
        waypoints = [mapa.grid_to_world(celula) for celula in caminho]
        return ResultadoPlanejamento(
            sucesso=True,
            waypoints=waypoints,
            custo=custo,
            motivo=f'caminho encontrado com {len(waypoints)} waypoints',
        )

    def criar_mascara_bloqueada(self, mapa: MapaGrade):
        ocupadas = set()
        bloqueadas = set()

        for y in range(mapa.height):
            for x in range(mapa.width):
                celula = Celula(x, y)
                if mapa.valor(celula) >= 100:
                    ocupadas.add(celula)

        raio = self.inflacao_obstaculo_celulas
        for celula in ocupadas:
            for dy in range(-raio, raio + 1):
                for dx in range(-raio, raio + 1):
                    vizinha = Celula(celula.x + dx, celula.y + dy)
                    if mapa.celula_valida(vizinha):
                        bloqueadas.add(vizinha)

        return bloqueadas

    def criar_mascara_adjacente_obstaculo(self, mapa: MapaGrade, bloqueadas):
        """Marca celulas livres logo ao lado da regiao bloqueada.

        A inflacao continua sendo a margem dura: celulas infladas sao
        proibidas. Esta mascara e uma margem suave, usada como custo maior
        para o A* preferir caminhos com mais folga quando existir alternativa.
        """

        adjacentes = set()
        for celula in bloqueadas:
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue

                    vizinha = Celula(celula.x + dx, celula.y + dy)
                    if not mapa.celula_valida(vizinha):
                        continue
                    if vizinha in bloqueadas:
                        continue

                    adjacentes.add(vizinha)

        return adjacentes

    def liberar_vizinhanca_do_robo(self, bloqueadas, inicio):
        # O mapper pinta a posicao atual do robo como 100 para visualizacao.
        # Sem esta folga, a inflacao trataria o proprio robo como obstaculo.
        raio = self.inflacao_obstaculo_celulas + 1
        for dy in range(-raio, raio + 1):
            for dx in range(-raio, raio + 1):
                bloqueadas.discard(Celula(inicio.x + dx, inicio.y + dy))

    def celula_livre_mais_proxima(self, mapa, alvo, bloqueadas, raio_max=12):
        if alvo not in bloqueadas:
            return alvo

        melhor = None
        melhor_distancia = math.inf
        for raio in range(1, raio_max + 1):
            for dy in range(-raio, raio + 1):
                for dx in range(-raio, raio + 1):
                    if abs(dx) != raio and abs(dy) != raio:
                        continue

                    candidata = Celula(alvo.x + dx, alvo.y + dy)
                    if not mapa.celula_valida(candidata):
                        continue
                    if candidata in bloqueadas:
                        continue

                    distancia = self.heuristica(candidata, alvo)
                    if distancia < melhor_distancia:
                        melhor = candidata
                        melhor_distancia = distancia

            if melhor is not None:
                return melhor

        return None

    def executar_a_estrela(
        self,
        mapa,
        inicio,
        destino,
        bloqueadas,
        adjacentes_obstaculo,
        yaw_inicial,
        peso_orientacao_inicial,
        distancia_orientacao_inicial,
    ):
        fronteira = []
        contador = 0
        heapq.heappush(fronteira, (0.0, contador, inicio))

        veio_de = {inicio: None}
        custo_ate = {inicio: 0.0}

        while fronteira:
            _, _, atual = heapq.heappop(fronteira)

            if atual == destino:
                return self.reconstruir_caminho(veio_de, atual), custo_ate[atual]

            for vizinha, custo_movimento in self.vizinhas(mapa, atual):
                if vizinha in bloqueadas:
                    continue

                novo_custo = (
                    custo_ate[atual]
                    + custo_movimento
                    * self.custo_da_celula(
                        mapa,
                        vizinha,
                        adjacentes_obstaculo,
                    )
                    + self.custo_orientacao_inicial(
                        mapa,
                        inicio,
                        vizinha,
                        yaw_inicial,
                        peso_orientacao_inicial,
                        distancia_orientacao_inicial,
                    )
                )
                if novo_custo >= custo_ate.get(vizinha, math.inf):
                    continue

                custo_ate[vizinha] = novo_custo
                prioridade = novo_custo + self.heuristica(vizinha, destino)
                contador += 1
                heapq.heappush(fronteira, (prioridade, contador, vizinha))
                veio_de[vizinha] = atual

        return [], math.inf

    def vizinhas(self, mapa, celula):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue

                vizinha = Celula(celula.x + dx, celula.y + dy)
                if not mapa.celula_valida(vizinha):
                    continue

                custo = math.sqrt(2.0) if dx and dy else 1.0
                yield vizinha, custo

    def custo_da_celula(self, mapa, celula, adjacentes_obstaculo=None):
        valor = mapa.valor(celula)
        custo = 1.0
        if valor == -1:
            custo = self.custo_desconhecido

        if adjacentes_obstaculo and celula in adjacentes_obstaculo:
            custo += self.custo_adjacente_obstaculo - 1.0

        return custo

    def custo_orientacao_inicial(
        self,
        mapa,
        inicio,
        celula,
        yaw_inicial,
        peso,
        distancia_limite,
    ):
        """Da preferencia aos primeiros passos na direcao atual do robo.

        O A* continua partindo da celula real do robo. A diferenca e que,
        quando existem rotas equivalentes, os primeiros metros recebem custo
        menor se apontam para onde o robo ja esta virado. Isso evita que o
        retorno com a bandeira comece com um giro puro para um waypoint curto.
        """

        if yaw_inicial is None or peso <= 0.0 or distancia_limite <= 0.0:
            return 0.0

        dx = (celula.x - inicio.x) * mapa.resolution
        dy = (celula.y - inicio.y) * mapa.resolution
        distancia = math.hypot(dx, dy)
        if distancia <= 1e-9 or distancia > distancia_limite:
            return 0.0

        angulo = math.atan2(dy, dx)
        erro = abs(self.normalizar_angulo(angulo - yaw_inicial))
        peso_distancia = 1.0 - (distancia / distancia_limite)
        return (
            float(peso)
            * (erro / math.pi) ** 2
            * (0.5 + 0.5 * peso_distancia)
        )

    @staticmethod
    def heuristica(a, b):
        return math.hypot(a.x - b.x, a.y - b.y)

    @staticmethod
    def normalizar_angulo(angulo):
        return math.atan2(math.sin(angulo), math.cos(angulo))

    @staticmethod
    def reconstruir_caminho(veio_de, atual):
        caminho = [atual]
        while veio_de[atual] is not None:
            atual = veio_de[atual]
            caminho.append(atual)
        caminho.reverse()
        return caminho

    @staticmethod
    def compactar_caminho(caminho):
        if len(caminho) <= 2:
            return caminho

        compacto = [caminho[0]]
        direcao_anterior = None
        for anterior, atual in zip(caminho, caminho[1:]):
            direcao = (
                atual.x - anterior.x,
                atual.y - anterior.y,
            )
            if direcao_anterior is not None and direcao != direcao_anterior:
                compacto.append(anterior)
            direcao_anterior = direcao

        compacto.append(caminho[-1])
        return compacto
