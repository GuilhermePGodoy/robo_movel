#!/usr/bin/env python3
import math
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from controle_robo.visao_bandeira import (
    calcular_centro_x_haste,
    calcular_ocupacao_label_central,
)


class DetectorBandeira(Node):
    """Detecta a bandeira azul na imagem segmentada do Gazebo.

    O plugin de segmentacao semantica publica duas imagens principais:
    labels_map, com o numero inteiro da label de cada pixel, e colored_map,
    com uma cor de visualizacao. Para a missao usamos labels_map, porque a
    label 25 identifica a bandeira azul sem confundir obstaculos azuis ou a
    base azul.
    """

    CAMPOS_DETECCAO = (
        'visivel',
        'erro_x',
        'area_relativa',
        'area_px',
        'centro_x',
        'centro_y',
        'largura_box',
        'altura_box',
        'largura_imagem',
        'altura_imagem',
        'centro_x_haste',
        'erro_x_haste',
        'obstaculo_central_relativo',
    )

    def __init__(self):
        super().__init__('detector_bandeira')

        self.declare_parameter('label_bandeira_azul', 25)
        self.declare_parameter('label_obstaculo', 30)
        self.declare_parameter('area_minima_bandeira', 5.0)
        self.declare_parameter('tolerancia_cor_bandeira', 0.0)
        self.declare_parameter('debug_detector', False)
        self.declare_parameter('publicar_mascara_debug', False)
        self.declare_parameter('periodo_log_debug', 1.0)

        self.label_bandeira_azul = int(
            self.get_parameter('label_bandeira_azul').value
        )
        self.label_obstaculo = int(
            self.get_parameter('label_obstaculo').value
        )
        self.area_minima_bandeira = float(
            self.get_parameter('area_minima_bandeira').value
        )
        self.tolerancia_cor_bandeira = float(
            self.get_parameter('tolerancia_cor_bandeira').value
        )
        self.debug_detector = bool(
            self.get_parameter('debug_detector').value
        )
        self.publicar_mascara_debug = bool(
            self.get_parameter('publicar_mascara_debug').value
        )
        self.periodo_log_debug = float(
            self.get_parameter('periodo_log_debug').value
        )

        self.bridge = CvBridge()
        self.ultimo_log_por_chave = {}

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/bandeira_azul/deteccao',
            10,
        )
        self.debug_info_pub = self.create_publisher(
            Float32MultiArray,
            '/bandeira_azul/debug_info',
            10,
        )
        self.mascara_debug_pub = self.create_publisher(
            Image,
            '/bandeira_azul/debug_mask',
            10,
        )
        self.create_subscription(
            Image,
            '/robot_cam/labels_map',
            self.camera_callback,
            10,
        )

        debug_status = 'ativo' if self.debug_detector else 'inativo'
        self.get_logger().info(
            'DETECTOR | alvo=label_'
            f'{self.label_bandeira_azul} | debug={debug_status}'
        )

    def camera_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.log_periodico(
                'erro_camera',
                f'CAMERA | erro=converter_imagem_segmentada | detalhe={exc}',
                periodo=2.0,
                nivel='warn',
            )
            return

        altura, largura = frame.shape[:2]
        mask, origem_segmentacao = self.criar_mascara_bandeira(frame, msg.encoding)
        obstaculo_central_relativo = self.calcular_obstaculo_central_relativo(
            frame
        )
        pixels_mascara = int(cv2.countNonZero(mask))

        contornos, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        contornos_validos = [
            contorno for contorno in contornos
            if cv2.contourArea(contorno) >= self.area_minima_bandeira
        ]
        maior_area = max(
            [cv2.contourArea(contorno) for contorno in contornos],
            default=0.0,
        )

        if self.publicar_mascara_debug:
            self.publicar_mascara(mask, msg)

        self.publicar_debug_info(
            pixels_mascara=pixels_mascara,
            total_contornos=len(contornos),
            contornos_validos=len(contornos_validos),
            maior_area=maior_area,
            largura=largura,
            altura=altura,
        )
        self.log_debug_frame(
            frame=frame,
            encoding=msg.encoding,
            origem_segmentacao=origem_segmentacao,
            pixels_mascara=pixels_mascara,
            total_contornos=len(contornos),
            contornos_validos=len(contornos_validos),
            maior_area=maior_area,
        )

        if not contornos_validos:
            self.publicar_sem_deteccao(
                largura=largura,
                altura=altura,
                obstaculo_central_relativo=obstaculo_central_relativo,
            )
            # Mensagem util quando a deteccao falha, mas barulhenta durante a
            # exploracao normal. Ative debug_detector no YAML para ve-la.
            if self.debug_detector:
                self.log_periodico(
                    'sem_bandeira',
                    (
                        'CAMERA | bandeira=nao | '
                        f'origem={origem_segmentacao}, '
                        f'pixels_label={pixels_mascara}, '
                        f'contornos={len(contornos)}, '
                        f'maior_area={maior_area:.1f}, '
                        f'area_min={self.area_minima_bandeira:.1f}, '
                        f'obst_img={obstaculo_central_relativo:.3f}'
                    ),
                    periodo=3.0,
                )
            return

        maior_contorno = max(contornos_validos, key=cv2.contourArea)
        area = cv2.contourArea(maior_contorno)
        x, y, w, h = cv2.boundingRect(maior_contorno)
        momentos = cv2.moments(maior_contorno)

        if momentos['m00'] != 0:
            centro_x = momentos['m10'] / momentos['m00']
            centro_y = momentos['m01'] / momentos['m00']
        else:
            centro_x = x + w / 2
            centro_y = y + h / 2

        erro_x = (centro_x - largura / 2) / (largura / 2)
        centro_x_haste = self.calcular_centro_x_haste(
            mask,
            x,
            y,
            w,
            h,
            centro_x,
        )
        erro_x_haste = (centro_x_haste - largura / 2) / (largura / 2)
        area_relativa = area / float(largura * altura)

        self.publicar_deteccao(
            erro_x=erro_x,
            area_relativa=area_relativa,
            area=area,
            centro_x=centro_x,
            centro_y=centro_y,
            largura_box=w,
            altura_box=h,
            largura_imagem=largura,
            altura_imagem=altura,
            centro_x_haste=centro_x_haste,
            erro_x_haste=erro_x_haste,
            obstaculo_central_relativo=obstaculo_central_relativo,
        )

        self.log_periodico(
            'bandeira_visivel',
            (
                'CAMERA | bandeira=sim | '
                f'origem={origem_segmentacao}, '
                f'cx={centro_x:.0f}/{largura}, erro={erro_x:+.2f}, '
                f'haste_x={centro_x_haste:.0f}, '
                f'erro_haste={erro_x_haste:+.2f}, '
                f'area={area:.0f}px/{area_relativa:.3f}, '
                f'obst_img={obstaculo_central_relativo:.3f}'
            ),
            periodo=1.0,
        )

    @staticmethod
    def calcular_centro_x_haste(mask, x, y, w, h, centro_x_fallback):
        return calcular_centro_x_haste(mask, x, y, w, h, centro_x_fallback)

    def calcular_obstaculo_central_relativo(self, frame):
        """Mede obstaculos na faixa central da imagem semantica."""

        if not self.imagem_tem_labels_numericos(frame):
            return 0.0

        labels = self.extrair_canal_de_labels(frame)
        return calcular_ocupacao_label_central(labels, self.label_obstaculo)

    def criar_mascara_bandeira(self, frame, encoding: str):
        if self.imagem_tem_labels_numericos(frame):
            labels = self.extrair_canal_de_labels(frame)
            mask = np.where(
                labels == self.label_bandeira_azul,
                255,
                0,
            ).astype(np.uint8)
            return mask, f'labels_map={self.label_bandeira_azul}'

        # Fallback para colored_map, util para debug caso o topico da camera
        # seja trocado no launch. O fluxo normal da missao usa labels_map.
        frame_bgr = self.garantir_bgr(frame, encoding)
        target_color = np.array([171, 242, 0], dtype=np.uint8)
        tolerancia = int(self.tolerancia_cor_bandeira)
        lower = np.clip(
            target_color.astype(int) - tolerancia,
            0,
            255,
        ).astype(np.uint8)
        upper = np.clip(
            target_color.astype(int) + tolerancia,
            0,
            255,
        ).astype(np.uint8)
        return cv2.inRange(frame_bgr, lower, upper), 'colored_map=#00f2ab'

    def log_debug_frame(
        self,
        frame,
        encoding: str,
        origem_segmentacao: str,
        pixels_mascara: int,
        total_contornos: int,
        contornos_validos: int,
        maior_area: float,
    ):
        if not self.debug_detector:
            return

        labels_resumo = self.resumir_labels(frame)
        canais = frame.shape[2] if frame.ndim == 3 else 1
        self.log_periodico(
            'debug_frame',
            (
                'DEBUG_CAMERA | '
                f'encoding={encoding}, dtype={frame.dtype}, '
                f'shape={frame.shape}, canais={canais}, '
                f'origem={origem_segmentacao}, '
                f'label_alvo={self.label_bandeira_azul}, '
                f'pixels_alvo={pixels_mascara}, '
                f'contornos={total_contornos}, validos={contornos_validos}, '
                f'maior_area={maior_area:.1f}, '
                f'labels_mais_comuns={labels_resumo}.'
            ),
            periodo=self.periodo_log_debug,
        )

    def resumir_labels(self, frame):
        if not self.imagem_tem_labels_numericos(frame):
            return 'n/a: imagem colorida'

        labels = self.extrair_canal_de_labels(frame)
        valores, contagens = np.unique(labels, return_counts=True)
        ordem = np.argsort(contagens)[::-1][:8]
        pares = [
            f'{int(valores[i])}:{int(contagens[i])}'
            for i in ordem
        ]
        return ','.join(pares)

    def publicar_debug_info(
        self,
        pixels_mascara: int,
        total_contornos: int,
        contornos_validos: int,
        maior_area: float,
        largura: int,
        altura: int,
    ):
        if not self.debug_detector:
            return

        msg = Float32MultiArray()
        msg.data = [
            float(self.label_bandeira_azul),
            float(pixels_mascara),
            float(total_contornos),
            float(contornos_validos),
            float(maior_area),
            float(self.area_minima_bandeira),
            float(largura),
            float(altura),
        ]
        self.debug_info_pub.publish(msg)

    def publicar_mascara(self, mask, msg_original: Image):
        try:
            msg_mask = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        except Exception as exc:
            self.log_periodico(
                'erro_mascara_debug',
                f'DEBUG_CAMERA | erro=publicar_mascara | detalhe={exc}',
                periodo=2.0,
                nivel='warn',
            )
            return

        msg_mask.header = msg_original.header
        self.mascara_debug_pub.publish(msg_mask)

    def publicar_sem_deteccao(
        self,
        largura: int,
        altura: int,
        obstaculo_central_relativo: float,
    ):
        """Publica a semantica central mesmo quando a bandeira nao aparece."""

        msg = Float32MultiArray()
        msg.layout.dim.clear()
        centro_x = largura / 2.0
        centro_y = altura / 2.0
        msg.data = [
            0.0,
            0.0,
            0.0,
            0.0,
            float(centro_x),
            float(centro_y),
            0.0,
            0.0,
            float(largura),
            float(altura),
            float(centro_x),
            0.0,
            float(obstaculo_central_relativo),
        ]
        self.publisher.publish(msg)

    def publicar_deteccao(
        self,
        erro_x: float,
        area_relativa: float,
        area: float,
        centro_x: float,
        centro_y: float,
        largura_box: int,
        altura_box: int,
        largura_imagem: int,
        altura_imagem: int,
        centro_x_haste: float,
        erro_x_haste: float,
        obstaculo_central_relativo: float,
    ):
        msg = Float32MultiArray()
        msg.layout.dim.clear()
        msg.data = [
            1.0,
            float(erro_x),
            float(area_relativa),
            float(area),
            float(centro_x),
            float(centro_y),
            float(largura_box),
            float(altura_box),
            float(largura_imagem),
            float(altura_imagem),
            float(centro_x_haste),
            float(erro_x_haste),
            float(obstaculo_central_relativo),
        ]
        self.publisher.publish(msg)

    def imagem_tem_labels_numericos(self, frame):
        if frame.ndim == 2:
            return True

        if frame.ndim == 3 and frame.shape[2] == 1:
            return True

        # Em alguns fluxos do ros_gz_bridge / cv_bridge, o labels_map chega
        # como rgb8/bgr8 cinza: a label 25 aparece como pixel (25, 25, 25).
        # Ainda e um mapa numerico, so veio repetido nos tres canais.
        if frame.ndim == 3 and frame.shape[2] >= 3:
            return (
                np.array_equal(frame[:, :, 0], frame[:, :, 1])
                and np.array_equal(frame[:, :, 1], frame[:, :, 2])
            )

        return False

    def extrair_canal_de_labels(self, frame):
        if frame.ndim == 3:
            return frame[:, :, 0]
        return frame

    def garantir_bgr(self, frame, encoding: str):
        encoding = encoding.lower()

        if frame.ndim == 2:
            return cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        if encoding == 'rgb8':
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == 'rgba8':
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if encoding == 'bgra8':
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        return frame

    def log_periodico(
        self,
        chave: str,
        mensagem: str,
        periodo: float = 1.0,
        nivel: str = 'info',
    ):
        agora = time.monotonic()
        ultimo_log = self.ultimo_log_por_chave.get(chave, -math.inf)
        if agora - ultimo_log < periodo:
            return

        self.ultimo_log_por_chave[chave] = agora
        logger = self.get_logger()
        if nivel == 'warn':
            logger.warn(mensagem)
        else:
            logger.info(mensagem)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorBandeira()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
