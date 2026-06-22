import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controle_robo.visao_bandeira import calcular_centro_x_haste


def test_centro_da_haste_usa_parte_baixa_da_mascara():
    mask = np.zeros((100, 120), dtype=np.uint8)

    # Painel da bandeira puxado para a esquerda, como nas imagens da camera.
    mask[10:45, 18:92] = 255

    # Haste e base ficam mais baixas; e nelas que a garra precisa mirar.
    mask[35:94, 76:86] = 255
    mask[88:96, 72:90] = 255

    centro = calcular_centro_x_haste(
        mask,
        x=18,
        y=10,
        w=74,
        h=86,
        centro_x_fallback=48.0,
    )

    assert 78.0 <= centro <= 84.0


def test_centro_da_haste_cai_no_fallback_sem_pixels_validos():
    mask = np.zeros((80, 100), dtype=np.uint8)

    centro = calcular_centro_x_haste(
        mask,
        x=10,
        y=5,
        w=50,
        h=30,
        centro_x_fallback=35.0,
    )

    assert centro == 35.0
