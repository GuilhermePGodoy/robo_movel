import numpy as np


def calcular_centro_x_haste(mask, x, y, w, h, centro_x_fallback):
    """Estima o x da haste usando a parte baixa do blob da bandeira.

    O centro da bounding box costuma cair no pano da bandeira. Para a garra,
    interessa mirar na haste. Por isso olhamos primeiro a parte baixa da
    mascara azul, onde aparecem a haste e a base, e so usamos o centro geral
    como fallback.
    """

    if w <= 0 or h <= 0:
        return centro_x_fallback

    # O painel fica mais alto; haste e base aparecem melhor na parte baixa.
    # Tentamos faixas cada vez maiores caso a bandeira esteja cortada.
    for inicio_relativo in (0.62, 0.55, 0.45):
        y_inicio = y + int(h * inicio_relativo)
        recorte = mask[y_inicio:y + h, x:x + w]
        if recorte.size == 0:
            continue

        colunas = np.count_nonzero(recorte > 0, axis=0)
        if colunas.size == 0 or int(colunas.max()) == 0:
            continue

        limite_coluna = max(2, int(colunas.max() * 0.25))
        indices = np.where(colunas >= limite_coluna)[0]
        if indices.size == 0:
            continue

        grupos = np.split(indices, np.where(np.diff(indices) > 1)[0] + 1)
        grupo = max(grupos, key=lambda g: int(colunas[g].sum()))
        pesos = colunas[grupo].astype(float)
        if pesos.sum() <= 0.0:
            continue

        centro_local = float(np.average(grupo, weights=pesos))
        return x + centro_local

    return centro_x_fallback


def calcular_ocupacao_label_central(labels, label_obstaculo):
    """Calcula quanto da faixa central da imagem contem obstaculo."""

    altura, largura = labels.shape[:2]
    x0 = int(largura * 0.25)
    x1 = int(largura * 0.75)
    y0 = int(altura * 0.35)
    y1 = int(altura * 0.95)
    recorte = labels[y0:y1, x0:x1]
    if recorte.size == 0:
        return 0.0

    pixels_obstaculo = int(np.count_nonzero(recorte == label_obstaculo))
    return pixels_obstaculo / float(recorte.size)
