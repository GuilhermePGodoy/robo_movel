"""Estimativa da posicao da bandeira a partir da camera.

A camera nao entrega profundidade. O que temos e a caixa da bandeira na imagem.
Como sabemos o tamanho real aproximado do painel, usamos o modelo pinhole para
chutar distancia e angulo. O resultado e uma hipotese: boa o bastante para A*,
mas ainda refinada pela visao quando o robo chega perto.
"""

from collections import deque
import math
import time

from controle_robo.modelos_missao import EstimativaBandeira


class EstimadorBandeira:
    """Calcula e suaviza estimativas de posicao da bandeira."""

    def __init__(
        self,
        fov_horizontal_camera: float,
        largura_real_bandeira: float,
        altura_real_bandeira: float,
        distancia_minima: float,
        distancia_maxima: float,
        tamanho_historico: int,
    ):
        self.fov_horizontal_camera = float(fov_horizontal_camera)
        self.largura_real_bandeira = float(largura_real_bandeira)
        self.altura_real_bandeira = float(altura_real_bandeira)
        self.distancia_minima = float(distancia_minima)
        self.distancia_maxima = float(distancia_maxima)
        self.historico = deque(maxlen=max(1, int(tamanho_historico)))

    def estimar(self, det, x_robo, y_robo, yaw_robo, distancia_frontal):
        if not det.visivel or det.altura <= 0 or det.largura <= 0:
            return EstimativaBandeira(instante=time.monotonic())
        if det.largura_imagem <= 0 or det.altura_imagem <= 0:
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

        confianca_tamanho = self.confianca_tamanho(det)
        confianca_centro = max(0.0, 1.0 - abs(erro_x_alvo))
        confianca_borda = self.confianca_borda(det)
        confianca_lidar = self.confianca_lidar(
            distancia,
            erro_x_alvo,
            distancia_frontal,
        )

        confianca = (
            0.35 * confianca_tamanho
            + 0.25 * confianca_centro
            + 0.20 * confianca_borda
            + 0.20 * confianca_lidar
        )
        if not distancia_valida:
            confianca *= 0.25

        estimativa = EstimativaBandeira(
            valida=distancia_valida and confianca > 0.0,
            x=x_alvo,
            y=y_alvo,
            distancia=distancia,
            angulo_relativo=angulo_relativo,
            angulo_mundo=angulo_mundo,
            confianca=confianca,
            confianca_tamanho=confianca_tamanho,
            confianca_centro=confianca_centro,
            confianca_borda=confianca_borda,
            confianca_lidar=confianca_lidar,
            instante=time.monotonic(),
        )

        if estimativa.valida and estimativa.confianca >= 0.25:
            self.historico.append(estimativa)
            return self.media_historico()

        return estimativa

    def focal_pixels(self, largura_imagem):
        return largura_imagem / (2.0 * math.tan(self.fov_horizontal_camera / 2.0))

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

    @staticmethod
    def confianca_tamanho(det):
        # Quando a bandeira aparece longe, a regiao segmentada pode ter poucos
        # pixels, mas ainda e uma pista boa porque vem da label semantica 25.
        # Por isso damos uma confianca pequena, mas nao nula, para caixas que
        # ja passaram pelo filtro do detector.
        conf_altura = min(1.0, max(0.0, (det.altura - 3.0) / 37.0))
        conf_largura = min(1.0, max(0.0, (det.largura - 3.0) / 37.0))
        conf = 0.65 * conf_altura + 0.35 * conf_largura
        if det.area >= 12.0:
            conf = max(0.22, conf)
        return conf

    @staticmethod
    def confianca_borda(det):
        margem_x = min(det.centro_x, det.largura_imagem - det.centro_x)
        margem_y = min(det.centro_y, det.altura_imagem - det.centro_y)
        conf_x = min(1.0, max(0.0, margem_x / (0.18 * det.largura_imagem)))
        conf_y = min(1.0, max(0.0, margem_y / (0.12 * det.altura_imagem)))
        return min(conf_x, conf_y)

    @staticmethod
    def confianca_lidar(distancia_camera, erro_x, distancia_frontal):
        if abs(erro_x) > 0.25 or not math.isfinite(distancia_frontal):
            return 0.5

        erro = abs(distancia_camera - distancia_frontal)
        return min(1.0, max(0.0, 1.0 - erro / 1.0))

    def media_historico(self):
        if not self.historico:
            return EstimativaBandeira(instante=time.monotonic())

        peso_total = sum(max(0.05, e.confianca) for e in self.historico)
        x = sum(e.x * max(0.05, e.confianca) for e in self.historico) / peso_total
        y = sum(e.y * max(0.05, e.confianca) for e in self.historico) / peso_total
        distancia = (
            sum(e.distancia * max(0.05, e.confianca) for e in self.historico)
            / peso_total
        )
        angulo_relativo = (
            sum(e.angulo_relativo * max(0.05, e.confianca) for e in self.historico)
            / peso_total
        )
        angulo_mundo = (
            sum(e.angulo_mundo * max(0.05, e.confianca) for e in self.historico)
            / peso_total
        )
        confianca = sum(e.confianca for e in self.historico) / len(self.historico)

        ultima = self.historico[-1]
        return EstimativaBandeira(
            valida=True,
            x=x,
            y=y,
            distancia=distancia,
            angulo_relativo=angulo_relativo,
            angulo_mundo=angulo_mundo,
            confianca=confianca,
            confianca_tamanho=ultima.confianca_tamanho,
            confianca_centro=ultima.confianca_centro,
            confianca_borda=ultima.confianca_borda,
            confianca_lidar=ultima.confianca_lidar,
            instante=ultima.instante,
        )

    def limpar_historico(self):
        self.historico.clear()
