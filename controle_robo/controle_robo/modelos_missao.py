from dataclasses import dataclass
from enum import Enum


class EstadoMissao(Enum):
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    ESTIMANDO_POSICAO_BANDEIRA = 'ESTIMANDO_POSICAO_BANDEIRA'
    PLANEJANDO_PARA_BANDEIRA = 'PLANEJANDO_PARA_BANDEIRA'
    SEGUINDO_CAMINHO_PARA_BANDEIRA = 'SEGUINDO_CAMINHO_PARA_BANDEIRA'
    PLANEJANDO_EXPLORACAO_DESCONHECIDA = 'PLANEJANDO_EXPLORACAO_DESCONHECIDA'
    SEGUINDO_CAMINHO_EXPLORACAO = 'SEGUINDO_CAMINHO_EXPLORACAO'
    DESVIANDO_OBSTACULO = 'DESVIANDO_OBSTACULO'
    REPLANEJANDO_CAMINHO = 'REPLANEJANDO_CAMINHO'
    FALHA_PLANEJAMENTO = 'FALHA_PLANEJAMENTO'
    REENCONTRANDO_BANDEIRA = 'REENCONTRANDO_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    CAPTURANDO_BANDEIRA = 'CAPTURANDO_BANDEIRA'
    PLANEJANDO_RETORNO_BASE = 'PLANEJANDO_RETORNO_BASE'
    RETORNANDO_BASE = 'RETORNANDO_BASE'
    ENTREGANDO_BANDEIRA = 'ENTREGANDO_BANDEIRA'
    MISSAO_CONCLUIDA = 'MISSAO_CONCLUIDA'


@dataclass
class DeteccaoBandeira:
    """Leitura visual ja processada pelo detector da bandeira azul."""

    visivel: bool = False
    centro_x: float = 0.0
    centro_y: float = 0.0
    erro_x: float = 0.0
    centro_x_haste: float = 0.0
    erro_x_haste: float = 0.0
    area: float = 0.0
    area_relativa: float = 0.0
    largura: int = 0
    altura: int = 0
    largura_imagem: int = 0
    altura_imagem: int = 0
    pose_robo_valida: bool = False
    x_robo: float = 0.0
    y_robo: float = 0.0
    yaw_robo: float = 0.0


@dataclass
class EstimativaBandeira:
    """Hipotese da posicao da bandeira no mapa."""

    valida: bool = False
    x: float = 0.0
    y: float = 0.0
    distancia: float = 0.0
    angulo_relativo: float = 0.0
    angulo_mundo: float = 0.0
    altura_bbox: int = 0
    instante: float = 0.0


@dataclass
class ResultadoPlanejamento:
    """Resultado do A* usado pela maquina de estados."""

    sucesso: bool = False
    waypoints: list = None
    custo: float = 0.0
    motivo: str = ''

    def __post_init__(self):
        if self.waypoints is None:
            self.waypoints = []
