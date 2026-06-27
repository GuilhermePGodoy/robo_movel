import sys
from pathlib import Path

from nav_msgs.msg import OccupancyGrid
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controle_robo.planejador_grade import Celula, MapaGrade, PlanejadorGrade


def mapa(width, height, resolution=1.0, ocupadas=None):
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * (width * height)

    for x, y in ocupadas or []:
        msg.data[y * width + x] = 100

    return msg


def test_mapa_vazio_encontra_caminho():
    planejador = PlanejadorGrade(inflacao_obstaculo_celulas=0)

    resultado = planejador.planejar(
        mapa(6, 6),
        inicio_xy=(0.5, 0.5),
        alvo_xy=(5.5, 5.5),
    )

    assert resultado.sucesso
    assert resultado.waypoints


def test_obstaculo_no_meio_gera_desvio():
    planejador = PlanejadorGrade(inflacao_obstaculo_celulas=0)

    resultado = planejador.planejar(
        mapa(6, 6, ocupadas={(2, 2), (3, 3)}),
        inicio_xy=(0.5, 0.5),
        alvo_xy=(5.5, 5.5),
    )

    assert resultado.sucesso
    assert all(
        (round(x - 0.5), round(y - 0.5)) not in {(2, 2), (3, 3)}
        for x, y in resultado.waypoints
    )


def test_destino_ocupado_perto_do_robo_nao_vira_waypoint_final():
    planejador = PlanejadorGrade(inflacao_obstaculo_celulas=0)
    msg = mapa(6, 6, ocupadas={(3, 2)})

    resultado = planejador.planejar(
        msg,
        inicio_xy=(2.5, 2.5),
        alvo_xy=(3.5, 2.5),
    )

    assert resultado.sucesso
    assert resultado.waypoints[-1] != (3.5, 2.5)


def test_celulas_adjacentes_a_obstaculo_recebem_custo_maior():
    planejador = PlanejadorGrade(
        inflacao_obstaculo_celulas=0,
        custo_adjacente_obstaculo=2.5,
    )
    grade = MapaGrade(mapa(5, 5, ocupadas={(2, 2)}))
    bloqueadas = planejador.criar_mascara_bloqueada(grade)
    adjacentes = planejador.criar_mascara_adjacente_obstaculo(
        grade,
        bloqueadas,
    )

    assert Celula(2, 2) in bloqueadas
    assert Celula(2, 2) not in adjacentes
    assert Celula(1, 1) in adjacentes
    assert Celula(2, 1) in adjacentes
    assert Celula(3, 3) in adjacentes
    assert Celula(0, 0) not in adjacentes

    assert planejador.custo_da_celula(
        grade,
        Celula(1, 1),
        adjacentes,
    ) == pytest.approx(2.5)
    assert planejador.custo_da_celula(
        grade,
        Celula(0, 0),
        adjacentes,
    ) == pytest.approx(1.0)


def test_inflacao_e_custo_suave_formam_duas_margens():
    planejador = PlanejadorGrade(
        inflacao_obstaculo_celulas=1,
        custo_adjacente_obstaculo=2.0,
    )
    grade = MapaGrade(mapa(7, 7, ocupadas={(3, 3)}))
    bloqueadas = planejador.criar_mascara_bloqueada(grade)
    adjacentes = planejador.criar_mascara_adjacente_obstaculo(
        grade,
        bloqueadas,
    )

    assert Celula(2, 2) in bloqueadas
    assert Celula(4, 4) in bloqueadas
    assert Celula(1, 1) in adjacentes
    assert Celula(5, 5) in adjacentes
    assert Celula(3, 3) not in adjacentes


def test_custo_orientacao_inicial_prefere_frente_do_robo():
    planejador = PlanejadorGrade(inflacao_obstaculo_celulas=0)
    grade = MapaGrade(mapa(5, 5, resolution=0.25))
    inicio = Celula(2, 2)

    custo_frente = planejador.custo_orientacao_inicial(
        grade,
        inicio,
        Celula(3, 2),
        yaw_inicial=0.0,
        peso=2.0,
        distancia_limite=1.0,
    )
    custo_atras = planejador.custo_orientacao_inicial(
        grade,
        inicio,
        Celula(1, 2),
        yaw_inicial=0.0,
        peso=2.0,
        distancia_limite=1.0,
    )

    assert custo_frente == pytest.approx(0.0)
    assert custo_atras > custo_frente
