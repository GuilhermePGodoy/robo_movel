"""Estimativa da posicao da bandeira a partir da camera.

A camera nao entrega profundidade. O que temos e a caixa da bandeira na imagem.
Como sabemos o tamanho real aproximado do painel, usamos o modelo pinhole para
chutar distancia e angulo. O resultado e uma hipotese: boa o bastante para A*,
mas ainda refinada pela visao quando o robo chega perto.
"""

import math
import time

from controle_robo.criterios_visuais import bbox_tem_geometria_de_bandeira
from controle_robo.modelos_missao import EstimativaBandeira


class EstimadorBandeira:
    """Calcula uma estimativa geometrica simples da posicao da bandeira."""

    def __init__(
        self,
        fov_horizontal_camera: float,
        largura_real_bandeira: float,
        altura_real_bandeira: float,
        distancia_minima: float,
        distancia_maxima: float,
        fill_ratio_minimo: float = 0.15,
        fill_ratio_maximo: float = 0.82,
        proporcao_minima_bbox: float = 0.20,
        proporcao_maxima_bbox: float = 1.35,
    ):
        self.fov_horizontal_camera = float(fov_horizontal_camera)
        self.largura_real_bandeira = float(largura_real_bandeira)
        self.altura_real_bandeira = float(altura_real_bandeira)
        self.distancia_minima = float(distancia_minima)
        self.distancia_maxima = float(distancia_maxima)
        self.fill_ratio_minimo = float(fill_ratio_minimo)
        self.fill_ratio_maximo = float(fill_ratio_maximo)
        self.proporcao_minima_bbox = float(proporcao_minima_bbox)
        self.proporcao_maxima_bbox = float(proporcao_maxima_bbox)

    def estimar(self, det, x_robo, y_robo, yaw_robo):
        if not det.visivel or det.altura <= 0 or det.largura <= 0:
            return EstimativaBandeira(instante=time.monotonic())
        if det.largura_imagem <= 0 or det.altura_imagem <= 0:
            return EstimativaBandeira(instante=time.monotonic())
        if not self.bbox_boa_para_estimativa(det):
            return EstimativaBandeira(instante=time.monotonic())

        fx = self.focal_pixels(det.largura_imagem)
        fy = fx

        distancia_altura = self.altura_real_bandeira * fy / det.altura
        distancia_largura = self.largura_real_bandeira * fx / det.largura

        # A largura sofre mais com perspectiva. A altura e a referencia
        # principal; a largura so entra suavemente para amortecer ruido.
        distancia = 0.75 * distancia_altura + 0.25 * distancia_largura
        distancia_valida = (
            self.distancia_minima <= distancia <= self.distancia_maxima
        )

        centro_x_alvo, erro_x_alvo = self.centro_x_para_estimativa(det)
        deslocamento_x = centro_x_alvo - det.largura_imagem / 2.0
        # Na imagem, x cresce para a direita. No plano do robo/mapa, yaw
        # positivo gira para a esquerda. Por isso o sinal precisa ser invertido:
        # bandeira a direita da imagem significa angulo relativo negativo.
        angulo_relativo = -math.atan2(deslocamento_x, fx)
        angulo_mundo = yaw_robo + angulo_relativo

        x_alvo = x_robo + distancia * math.cos(angulo_mundo)
        y_alvo = y_robo + distancia * math.sin(angulo_mundo)

        return EstimativaBandeira(
            valida=distancia_valida,
            x=x_alvo,
            y=y_alvo,
            distancia=distancia,
            angulo_relativo=angulo_relativo,
            angulo_mundo=angulo_mundo,
            altura_bbox=det.altura,
            instante=time.monotonic(),
        )

    def focal_pixels(self, largura_imagem):
        return largura_imagem / (2.0 * math.tan(self.fov_horizontal_camera / 2.0))

    def bbox_boa_para_estimativa(self, det):
        """Filtra recortes que nao representam a bandeira inteira.

        Quando a camera ve so o pano ou so a haste, a bbox fica quase toda
        preenchida por azul. Se usarmos essa altura pequena na trigonometria,
        a distancia estimada vai para longe demais. Uma bandeira completa tem
        buracos/recortes dentro da bbox e uma proporcao menos extrema.
        """

        return bbox_tem_geometria_de_bandeira(
            det,
            self.fill_ratio_minimo,
            self.fill_ratio_maximo,
            self.proporcao_minima_bbox,
            self.proporcao_maxima_bbox,
        )

    @staticmethod
    def centro_x_para_estimativa(det):
        """Usa a haste como referencia horizontal quando ela foi calculada."""

        haste_disponivel = (
            det.centro_x_haste > 0.0
            or abs(det.erro_x_haste) > 1e-9
        )
        if haste_disponivel:
            return det.centro_x_haste, det.erro_x_haste

        return det.centro_x, det.erro_x
