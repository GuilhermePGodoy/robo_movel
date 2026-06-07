from dataclasses import dataclass
from enum import Enum


class EstadoMissao(Enum):
    EXPLORANDO = 'EXPLORANDO'
    BANDEIRA_DETECTADA = 'BANDEIRA_DETECTADA'
    NAVIGANDO_PARA_BANDEIRA = 'NAVIGANDO_PARA_BANDEIRA'
    DESVIANDO_OBSTACULO = 'DESVIANDO_OBSTACULO'
    REDETECTANDO_BANDEIRA = 'REDETECTANDO_BANDEIRA'
    POSICIONANDO_PARA_COLETA = 'POSICIONANDO_PARA_COLETA'
    CAPTURANDO_BANDEIRA = 'CAPTURANDO_BANDEIRA'


@dataclass
class DeteccaoBandeira:
    """Leitura visual ja processada pelo detector da bandeira azul."""

    visivel: bool = False
    centro_x: float = 0.0
    centro_y: float = 0.0
    erro_x: float = 0.0
    area: float = 0.0
    area_relativa: float = 0.0
    largura: int = 0
    altura: int = 0
