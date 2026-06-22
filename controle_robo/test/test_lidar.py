import math
import sys
from pathlib import Path

import pytest
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controle_robo.lidar import processar_scan


def scan_com_angulos(angulos_graus, ranges):
    msg = LaserScan()
    msg.angle_min = math.radians(angulos_graus[0])
    msg.angle_increment = math.radians(angulos_graus[1] - angulos_graus[0])
    msg.range_min = 0.01
    msg.range_max = 20.0
    msg.ranges = list(ranges)
    return msg


def test_direcao_desvio_usa_sinal_do_angulo():
    # Angulos negativos ficam a direita do robo; positivos ficam a esquerda.
    # Neste cenario, a direita/frente esta apertada e a esquerda esta livre,
    # entao o robo deve girar para a esquerda.
    msg = scan_com_angulos(
        [-90, -60, -30, 0, 30, 60, 90],
        [0.24, 0.24, 0.24, 0.24, 1.40, 1.43, 1.43],
    )

    leitura = processar_scan(
        msg,
        limite_frontal=math.radians(45),
        limite_central_ignorado=math.radians(8),
        limite_central_coleta=math.radians(5),
        distancia_obstaculo=0.6,
    )

    assert leitura.distancia_direita_frente == pytest.approx(0.24)
    assert leitura.distancia_esquerda_frente == pytest.approx(1.40)
    assert leitura.nome_lado_desvio() == 'esquerda'


def test_obstaculo_central_nao_inverte_lado_do_desvio():
    # Obstaculo no centro dispara desvio, mas nao deve decidir sozinho o lado.
    # Como a lateral esquerda esta mais livre, a escolha continua sendo esquerda.
    msg = scan_com_angulos(
        [-90, -60, -30, 0, 30, 60, 90],
        [0.35, 0.35, 1.20, 0.20, 1.20, 1.50, 1.50],
    )

    leitura = processar_scan(
        msg,
        limite_frontal=math.radians(45),
        limite_central_ignorado=math.radians(8),
        limite_central_coleta=math.radians(5),
        distancia_obstaculo=0.6,
    )

    assert leitura.distancia_frontal == pytest.approx(0.20)
    assert leitura.distancia_direita_frente == pytest.approx(0.35)
    assert leitura.distancia_esquerda_frente == pytest.approx(1.20)
    assert leitura.nome_lado_desvio() == 'esquerda'


def test_distancia_de_coleta_usa_apenas_faixa_central_estreita():
    msg = scan_com_angulos(
        [-30, -15, 0, 15, 30],
        [0.20, 1.30, 1.10, 1.30, 0.22],
    )

    leitura = processar_scan(
        msg,
        limite_frontal=math.radians(45),
        limite_central_ignorado=math.radians(8),
        limite_central_coleta=math.radians(5),
        distancia_obstaculo=0.6,
    )

    assert leitura.distancia_frontal == pytest.approx(0.20)
    assert leitura.distancia_frontal_coleta == pytest.approx(1.10)
