"""Leitura organizada do LaserScan.

O LIDAR e usado em dois papeis: seguranca local contra obstaculos e controle
de velocidade conforme a frente esta livre. Depois da captura, a bandeira fica
na frente do robo e aparece no meio do scan; por isso tambem calculamos uma
leitura frontal ignorando uma pequena janela central.
"""

from dataclasses import dataclass
import math


@dataclass
class LeituraLidar:
    distancia_frontal: float = math.inf
    distancia_frontal_sem_centro: float = math.inf
    distancia_esquerda: float = math.inf
    distancia_direita: float = math.inf
    distancia_esquerda_frente: float = math.inf
    distancia_direita_frente: float = math.inf
    obstaculo_a_frente: bool = False
    obstaculo_a_frente_sem_centro: bool = False
    direcao_desvio: float = 1.0

    def nome_lado_desvio(self):
        return 'esquerda' if self.direcao_desvio > 0.0 else 'direita'


def processar_scan(
    msg,
    limite_frontal: float,
    limite_central_ignorado: float,
    distancia_obstaculo: float,
):
    """Separa o scan em regioes usadas pelo controle reativo."""

    distancias_frente = []
    distancias_frente_sem_centro = []
    distancias_esquerda = []
    distancias_direita = []

    for indice, distancia in enumerate(msg.ranges):
        angulo = msg.angle_min + indice * msg.angle_increment
        angulo = math.atan2(math.sin(angulo), math.cos(angulo))

        leitura_valida = (
            math.isfinite(distancia)
            and msg.range_min <= distancia <= msg.range_max
        )
        if not leitura_valida:
            continue

        if abs(angulo) <= limite_frontal:
            distancias_frente.append(distancia)
            if abs(angulo) >= limite_central_ignorado:
                distancias_frente_sem_centro.append(distancia)
        elif limite_frontal < angulo <= math.radians(90):
            distancias_esquerda.append(distancia)
        elif -math.radians(90) <= angulo < -limite_frontal:
            distancias_direita.append(distancia)

    leitura = LeituraLidar(
        distancia_frontal=min(distancias_frente, default=math.inf),
        distancia_frontal_sem_centro=min(
            distancias_frente_sem_centro,
            default=math.inf,
        ),
        distancia_esquerda=min(distancias_esquerda, default=math.inf),
        distancia_direita=min(distancias_direita, default=math.inf),
    )

    metade_frente = len(distancias_frente) // 2
    leitura.distancia_esquerda_frente = min(
        distancias_esquerda + distancias_frente[:metade_frente],
        default=math.inf,
    )
    leitura.distancia_direita_frente = min(
        distancias_direita + distancias_frente[metade_frente:],
        default=math.inf,
    )

    leitura.obstaculo_a_frente = tem_obstaculo(
        leitura.distancia_frontal,
        leitura.distancia_esquerda,
        leitura.distancia_direita,
        distancia_obstaculo,
    )
    leitura.obstaculo_a_frente_sem_centro = tem_obstaculo(
        leitura.distancia_frontal_sem_centro,
        leitura.distancia_esquerda,
        leitura.distancia_direita,
        distancia_obstaculo,
    )
    leitura.direcao_desvio = escolher_direcao_desvio(
        leitura.distancia_esquerda_frente,
        leitura.distancia_direita_frente,
    )
    return leitura


def tem_obstaculo(frente, esquerda, direita, distancia_obstaculo):
    return (
        frente < distancia_obstaculo
        or direita < 0.35 * distancia_obstaculo
        or esquerda < 0.35 * distancia_obstaculo
    )


def escolher_direcao_desvio(esquerda_frente, direita_frente):
    # Z angular positivo gira para a esquerda; negativo gira para a direita.
    if esquerda_frente >= direita_frente:
        return 1.0
    return -1.0
