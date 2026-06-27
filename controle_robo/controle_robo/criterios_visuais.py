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


def bbox_tem_geometria_de_bandeira(
    det,
    fill_ratio_minimo,
    fill_ratio_maximo,
    proporcao_minima,
    proporcao_maxima,
):
    """Checa se a bbox parece representar a bandeira completa.

    Uma bbox quase toda preenchida por azul costuma ser so o tecido ou so a
    haste. Uma bbox com proporcao extrema tambem costuma ser uma fresta. Esses
    dois filtros sao simples, mas evitam varias estimativas ruins.
    """

    area_bbox = float(det.largura * det.altura)
    if area_bbox <= 0.0:
        return False

    fill_ratio = det.area / area_bbox
    proporcao = det.largura / float(det.altura)
    return (
        float(fill_ratio_minimo)
        <= fill_ratio
        <= float(fill_ratio_maximo)
        and float(proporcao_minima)
        <= proporcao
        <= float(proporcao_maxima)
    )


def bandeira_util_para_posicionamento(
    det,
    bandeira_inteira_confirmada,
    perto_da_estimativa,
):
    """Indica se a imagem ja tem bandeira suficiente para guiar o robo.

    Nao usamos mais area como gatilho direto: ela muda muito com o angulo da
    bandeira. Para assumir o ajuste fino, precisamos ver alguma parte da
    bandeira e ter uma evidencia melhor: a bbox inteira confirmada ou uma
    distancia curta ate a melhor estimativa disponivel.
    """

    return (
        bandeira_parcial_visivel(det)
        and (bandeira_inteira_confirmada or perto_da_estimativa)
    )


def bandeira_inteira_no_frame(
    det,
    margem_borda_px,
    area_minima_relativa,
    fill_ratio_minimo,
    fill_ratio_maximo,
    proporcao_minima,
    proporcao_maxima,
):
    """Heuristica de um frame: bbox da bandeira inteira dentro da imagem."""

    if not bandeira_parcial_visivel(det):
        return False
    if det.area_relativa < area_minima_relativa:
        return False
    if not bbox_tem_geometria_de_bandeira(
        det,
        fill_ratio_minimo,
        fill_ratio_maximo,
        proporcao_minima,
        proporcao_maxima,
    ):
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

    def atualizar(
        self,
        det,
        margem_borda_px,
        area_minima_relativa,
        fill_ratio_minimo,
        fill_ratio_maximo,
        proporcao_minima,
        proporcao_maxima,
    ):
        if bandeira_inteira_no_frame(
            det,
            margem_borda_px,
            area_minima_relativa,
            fill_ratio_minimo,
            fill_ratio_maximo,
            proporcao_minima,
            proporcao_maxima,
        ):
            self.frames_confirmados += 1
        else:
            self.frames_confirmados = 0

        return self.frames_confirmados

    def confirmada(self, frames_necessarios):
        return self.frames_confirmados >= max(1, int(frames_necessarios))
