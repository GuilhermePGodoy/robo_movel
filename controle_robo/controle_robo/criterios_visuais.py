"""Criterios pequenos para interpretar a deteccao visual da bandeira."""


def bandeira_parcial_visivel(det):
    """Retorna True quando o detector publicou uma caixa minima da bandeira."""

    return (
        det.visivel
        and det.area_relativa > 0.0
        and det.largura > 0
        and det.altura > 0
        and det.largura_imagem > 0
        and det.altura_imagem > 0
    )


def bandeira_util_para_posicionamento(det, area_minima_relativa):
    """Indica se a imagem ja tem bandeira suficiente para guiar o robo.

    Uma fresta da bandeira e util para saber que ela existe, mas nao e boa para
    aproximacao fina: o centro da bbox fica muito instavel e o robo passa a
    perseguir uma amostra ruim. Por isso usamos a area como filtro simples.
    """

    return (
        bandeira_parcial_visivel(det)
        and det.area_relativa >= area_minima_relativa
    )


def bandeira_inteira_no_frame(det, margem_borda_px, area_minima_relativa):
    """Heuristica de um frame: bbox da bandeira inteira dentro da imagem."""

    if not bandeira_parcial_visivel(det):
        return False
    if det.area_relativa < area_minima_relativa:
        return False
    margem = max(0.0, float(margem_borda_px))
    esquerda = det.centro_x - det.largura / 2.0
    direita = det.centro_x + det.largura / 2.0
    topo = det.centro_y - det.altura / 2.0
    base = det.centro_y + det.altura / 2.0

    return (
        esquerda > margem
        and direita < det.largura_imagem - margem
        and topo > margem
        and base < det.altura_imagem - margem
    )


class ConfirmadorBandeiraInteira:
    """Conta frames seguidos em que a bandeira parece inteira na imagem."""

    def __init__(self):
        self.frames_confirmados = 0

    def atualizar(self, det, margem_borda_px, area_minima_relativa):
        if bandeira_inteira_no_frame(
            det,
            margem_borda_px,
            area_minima_relativa,
        ):
            self.frames_confirmados += 1
        else:
            self.frames_confirmados = 0

        return self.frames_confirmados

    def confirmada(self, frames_necessarios):
        return self.frames_confirmados >= max(1, int(frames_necessarios))
