import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from controle_robo.criterios_visuais import (
    ConfirmadorBandeiraInteira,
    bandeira_inteira_no_frame,
    bandeira_parcial_visivel,
    bandeira_util_para_posicionamento,
)
from controle_robo.estimador_bandeira import EstimadorBandeira
from controle_robo.modelos_missao import DeteccaoBandeira


def criar_estimador():
    return EstimadorBandeira(
        fov_horizontal_camera=1.57,
        largura_real_bandeira=0.3,
        altura_real_bandeira=0.48,
        distancia_minima=0.2,
        distancia_maxima=10.0,
        tamanho_historico=3,
    )


def deteccao(
    altura=60,
    largura=40,
    centro_x=160,
    centro_y=120,
    centro_x_haste=None,
):
    if centro_x_haste is None:
        centro_x_haste = centro_x

    return DeteccaoBandeira(
        visivel=True,
        centro_x=centro_x,
        centro_y=centro_y,
        erro_x=(centro_x - 160) / 160,
        centro_x_haste=centro_x_haste,
        erro_x_haste=(centro_x_haste - 160) / 160,
        area=altura * largura,
        area_relativa=(altura * largura) / (320 * 240),
        largura=largura,
        altura=altura,
        largura_imagem=320,
        altura_imagem=240,
    )


def test_bbox_inteira_e_central_passa():
    det = deteccao(altura=70, largura=45, centro_x=160, centro_y=120)

    assert bandeira_inteira_no_frame(
        det,
        margem_borda_px=8,
        area_minima_relativa=0.003,
    )


def test_bbox_encostando_na_borda_falha():
    det = deteccao(altura=70, largura=45, centro_x=15, centro_y=120)

    assert not bandeira_inteira_no_frame(
        det,
        margem_borda_px=8,
        area_minima_relativa=0.003,
    )


def test_bbox_pequena_demais_falha():
    det = deteccao(altura=6, largura=6, centro_x=160, centro_y=120)

    assert not bandeira_inteira_no_frame(
        det,
        margem_borda_px=8,
        area_minima_relativa=0.003,
    )


def test_fresta_nao_e_util_para_posicionamento():
    det = deteccao(altura=80, largura=8, centro_x=160, centro_y=120)

    assert bandeira_parcial_visivel(det)
    assert not bandeira_util_para_posicionamento(
        det,
        area_minima_relativa=0.02,
    )


def test_bbox_com_area_suficiente_e_util_para_posicionamento():
    det = deteccao(altura=70, largura=45, centro_x=160, centro_y=120)

    assert bandeira_util_para_posicionamento(
        det,
        area_minima_relativa=0.02,
    )


def test_confirmacao_exige_frames_consecutivos():
    confirmador = ConfirmadorBandeiraInteira()
    det = deteccao(altura=70, largura=45, centro_x=160, centro_y=120)

    confirmador.atualizar(det, margem_borda_px=8, area_minima_relativa=0.003)
    assert not confirmador.confirmada(frames_necessarios=3)

    confirmador.atualizar(det, margem_borda_px=8, area_minima_relativa=0.003)
    assert not confirmador.confirmada(frames_necessarios=3)

    confirmador.atualizar(det, margem_borda_px=8, area_minima_relativa=0.003)
    assert confirmador.confirmada(frames_necessarios=3)


def test_lado_da_imagem_vira_lado_correto_no_mapa():
    direita = criar_estimador().estimar(
        deteccao(altura=55, centro_x=240),
        0.0,
        0.0,
        0.0,
        float('inf'),
    )
    esquerda = criar_estimador().estimar(
        deteccao(altura=55, centro_x=80),
        0.0,
        0.0,
        0.0,
        float('inf'),
    )

    assert direita.y < 0.0
    assert esquerda.y > 0.0


def test_estimativa_mira_na_haste_nao_no_centro_do_pano():
    estimativa = criar_estimador().estimar(
        deteccao(
            altura=70,
            largura=50,
            centro_x=105,
            centro_x_haste=160,
        ),
        0.0,
        0.0,
        0.0,
        float('inf'),
    )

    assert abs(estimativa.y) < 0.05
