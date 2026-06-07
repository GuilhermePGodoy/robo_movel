"""Maquina de estados da missao da bandeira azul.

O arquivo ficou separado do no ROS para deixar claro o que e decisao de
controle e o que e infraestrutura de ROS. Assim fica mais facil testar ideias
de navegacao sem mexer em publisher, subscriber ou parametros.
"""

import math
import time

from controle_robo.modelos_missao import EstadoMissao


class MaquinaEstadosMissao:
    """Controla as fases da missao da bandeira azul.

    Esta classe nao cria publishers nem subscribers. Ela recebe uma referencia
    para o no ROS e usa as leituras ja atualizadas pelos callbacks. Na pratica:
    o no cuida de ROS; esta classe cuida de decidir o que o robo deve fazer.
    """

    def __init__(self, robo):
        self.robo = robo
        self.estado_atual = EstadoMissao.EXPLORANDO
        self.instante_inicio_estado = time.monotonic()
        self.garra_aberta = False
        self.garra_fechada = False

        # Tabela simples de despacho: cada estado aponta para o metodo que
        # executa sua regra. Quando entrar um estado novo, ele aparece aqui.
        self.acoes_por_estado = {
            EstadoMissao.EXPLORANDO: self.estado_explorando,
            EstadoMissao.BANDEIRA_DETECTADA: self.estado_bandeira_detectada,
            EstadoMissao.NAVIGANDO_PARA_BANDEIRA: (
                self.estado_navegando_para_bandeira
            ),
            EstadoMissao.DESVIANDO_OBSTACULO: self.estado_desviando_obstaculo,
            EstadoMissao.REDETECTANDO_BANDEIRA: (
                self.estado_redetectando_bandeira
            ),
            EstadoMissao.POSICIONANDO_PARA_COLETA: (
                self.estado_posicionando_para_coleta
            ),
            EstadoMissao.CAPTURANDO_BANDEIRA: self.estado_capturando_bandeira,
        }

    def executar(self):
        acao = self.acoes_por_estado.get(self.estado_atual)
        if acao is None:
            self.robo.publicar_velocidade(0.0, 0.0)
            self.robo.get_logger().warn(
                f'Estado desconhecido: {self.estado_atual}. Robo parado.'
            )
            return

        acao()

    def estado_explorando(self):
        robo = self.robo
        self.abrir_garra()

        if self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'a camera segmentada encontrou a label da bandeira azul',
            )
            robo.publicar_velocidade(0.0, 0.0)
            return

        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo no caminho durante exploracao '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        # Busca simples: anda em uma curva leve para a camera varrer a arena.
        # Nao usamos a coordenada da bandeira; a camera decide quando sair daqui.
        fase = math.sin(time.monotonic() * 0.55)
        angular = robo.limitar(
            robo.amplitude_varredura_camera * fase,
            -robo.velocidade_giro_busca,
            robo.velocidade_giro_busca,
        )
        fator_obstaculo = self.fator_velocidade_por_obstaculo()
        linear = robo.velocidade_exploracao * fator_obstaculo

        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'explorando em curva suave; '
                f'pose=({robo.x:.2f}, {robo.y:.2f}, yaw={robo.yaw:.2f}), '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=1.5,
        )

    def estado_bandeira_detectada(self):
        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                'deteccao visual ficou antiga logo apos encontrar a bandeira',
            )
            return

        det = self.robo.deteccao_bandeira
        self.log_estado_periodico(
            (
                'bandeira detectada; calculando direcao relativa '
                f'(erro={det.erro_x:+.2f}, area={det.area_relativa:.3f})'
            ),
            periodo=0.5,
        )

        if self.bandeira_pronta_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira ja esta centralizada e proxima',
            )
        else:
            self.trocar_estado(
                EstadoMissao.NAVIGANDO_PARA_BANDEIRA,
                'bandeira detectada, iniciando aproximacao visual',
            )

    def estado_navegando_para_bandeira(self):
        robo = self.robo

        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                f'bandeira perdida ha {self.tempo_desde_bandeira():.1f} s',
            )
            return

        if self.bandeira_pronta_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira centralizada/proxima o bastante para ajuste fino',
            )
            return

        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo entre robo e bandeira '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        det = robo.deteccao_bandeira
        angular = self.controle_angular_para_bandeira()
        fator_alinhamento = max(0.25, 1.0 - abs(det.erro_x))
        fator_obstaculo = self.fator_velocidade_por_obstaculo()
        linear = robo.velocidade_linear * fator_alinhamento * fator_obstaculo

        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'navegando para bandeira; '
                f'erro={det.erro_x:+.2f}, area={det.area_relativa:.3f}, '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=1.0,
        )

    def estado_desviando_obstaculo(self):
        robo = self.robo
        tempo_no_estado = time.monotonic() - self.instante_inicio_estado

        if (
            not robo.obstaculo_a_frente
            and tempo_no_estado >= robo.tempo_minimo_desvio
        ):
            if self.bandeira_recente():
                self.trocar_estado(
                    EstadoMissao.NAVIGANDO_PARA_BANDEIRA,
                    'obstaculo liberado e bandeira continua visivel',
                )
            else:
                self.trocar_estado(
                    EstadoMissao.EXPLORANDO,
                    'obstaculo liberado; retomando busca pela bandeira',
                )
            return

        angular = robo.velocidade_angular_desvio * robo.direcao_desvio
        sentido = 'esquerda' if robo.direcao_desvio > 0 else 'direita'
        robo.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                f'desviando: girando para {sentido}; '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
                f'dir={robo.formatar_distancia(robo.distancia_direita)}'
            ),
            periodo=0.8,
        )

    def estado_redetectando_bandeira(self):
        robo = self.robo

        if self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'bandeira reapareceu no campo de visao',
            )
            return

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= robo.tempo_reexploracao:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'tempo de redeteccao esgotado; voltando para exploracao',
            )
            return

        # Gira para o ultimo lado onde a bandeira apareceu.
        direcao_busca = -1.0 if robo.ultimo_erro_bandeira > 0 else 1.0
        angular = robo.velocidade_giro_busca * direcao_busca
        robo.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                'redetectando bandeira; '
                f'ultimo_erro={robo.ultimo_erro_bandeira:+.2f}, '
                f'tempo={tempo_no_estado:.1f}/{robo.tempo_reexploracao:.1f}s'
            ),
            periodo=0.8,
        )

    def estado_posicionando_para_coleta(self):
        robo = self.robo

        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.REDETECTANDO_BANDEIRA,
                'bandeira perdida durante ajuste fino',
            )
            return

        if self.bandeira_pronta_para_coleta():
            self.trocar_estado(
                EstadoMissao.CAPTURANDO_BANDEIRA,
                'distancia e alinhamento suficientes para captura',
            )
            return

        if (
            robo.obstaculo_a_frente
            and self.obstaculo_deve_ser_desviado_no_posicionamento()
        ):
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo inesperado durante posicionamento '
                    f'({robo.distancia_frontal:.2f} m); '
                    'bandeira ainda nao esta enquadrada como alvo de coleta'
                ),
            )
            return

        det = robo.deteccao_bandeira
        angular = self.controle_angular_para_bandeira()

        if abs(det.erro_x) > robo.erro_alinhamento_bandeira:
            linear = 0.0
            fator_obstaculo = 1.0
            acao = 'ajustando orientacao'
        else:
            fator_obstaculo = self.fator_velocidade_por_obstaculo(
                permitir_aceleracao=False
            )
            linear = robo.velocidade_posicionamento * fator_obstaculo
            acao = 'aproximando devagar'

        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                f'{acao}; erro={det.erro_x:+.2f}, '
                f'area={det.area_relativa:.3f}, '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=0.7,
        )

    def estado_capturando_bandeira(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        self.fechar_garra()
        self.log_estado_periodico(
            (
                'missao concluida: robo parado de frente para a bandeira '
                f'azul em pose=({robo.x:.2f}, {robo.y:.2f}).'
            ),
            periodo=2.0,
        )

    # Predicados pequenos deixam as transicoes mais legiveis: os estados leem
    # quase como uma frase, e os detalhes ficam concentrados aqui embaixo.
    def bandeira_recente(self):
        return (
            self.robo.deteccao_bandeira.visivel
            and self.tempo_desde_bandeira() <= self.robo.tempo_perda_bandeira
        )

    def tempo_desde_bandeira(self):
        if self.robo.ultimo_instante_bandeira is None:
            return math.inf

        return time.monotonic() - self.robo.ultimo_instante_bandeira

    def bandeira_pronta_para_posicionamento(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        centralizada = abs(det.erro_x) <= robo.erro_alinhamento_bandeira * 1.5
        proxima_por_imagem = (
            det.area_relativa >= robo.area_posicionamento_bandeira
        )
        proxima_por_lidar = (
            robo.distancia_frontal <= robo.distancia_posicionamento
            and det.area_relativa >= robo.area_posicionamento_bandeira * 0.4
        )
        return centralizada and (proxima_por_imagem or proxima_por_lidar)

    def bandeira_pronta_para_coleta(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        centralizada = abs(det.erro_x) <= robo.erro_alinhamento_bandeira
        proxima_por_imagem = det.area_relativa >= robo.area_coleta_bandeira
        area_minima_para_lidar = max(
            robo.area_posicionamento_bandeira,
            robo.area_coleta_bandeira * 0.5,
        )
        proxima_por_lidar = (
            robo.distancia_frontal <= robo.distancia_coleta
            and det.area_relativa >= area_minima_para_lidar
        )
        return centralizada and (proxima_por_imagem or proxima_por_lidar)

    def obstaculo_deve_ser_desviado_no_posicionamento(self):
        robo = self.robo
        det = robo.deteccao_bandeira

        # Aqui o LIDAR pode estar vendo a propria bandeira. Se a imagem
        # confirma que ela esta bem enquadrada, melhor terminar a aproximacao.
        alvo_de_coleta_provavel = (
            abs(det.erro_x) <= robo.erro_alinhamento_bandeira
            and det.area_relativa >= robo.area_posicionamento_bandeira
        )
        if alvo_de_coleta_provavel:
            robo.log_periodico(
                'posicionamento_lidar_alvo',
                (
                    'Posicionamento: LIDAR viu algo a frente, mas a bandeira '
                    'esta centralizada/grande; mantendo aproximacao fina. '
                    f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                    f'area={det.area_relativa:.3f}, erro={det.erro_x:+.2f}.'
                ),
                periodo=1.0,
            )
            return False

        return True

    def controle_angular_para_bandeira(self):
        robo = self.robo
        erro = robo.deteccao_bandeira.erro_x

        # Erro positivo significa bandeira a direita da imagem. Em ROS,
        # angular.z negativo gira o robo para a direita.
        angular = -robo.ganho_angular_bandeira * erro
        return robo.limitar(
            angular,
            -robo.velocidade_giro_busca,
            robo.velocidade_giro_busca,
        )

    def fator_velocidade_por_obstaculo(self, permitir_aceleracao: bool = True):
        robo = self.robo

        if math.isinf(robo.distancia_frontal):
            fator = robo.fator_velocidade_livre
        elif robo.distancia_frontal >= robo.distancia_velocidade_livre:
            fator = robo.fator_velocidade_livre
        elif robo.distancia_frontal <= robo.distancia_obstaculo:
            fator = robo.fator_velocidade_proxima
        else:
            faixa = robo.distancia_velocidade_livre - robo.distancia_obstaculo
            progresso = (
                robo.distancia_frontal - robo.distancia_obstaculo
            ) / faixa
            fator = (
                robo.fator_velocidade_proxima
                + progresso
                * (robo.fator_velocidade_livre - robo.fator_velocidade_proxima)
            )

        if not permitir_aceleracao:
            fator = min(1.0, fator)

        return fator

    # Comandos auxiliares. A garra fica protegida para nao ficar reenviando a
    # mesma ordem em todo ciclo do timer.
    def abrir_garra(self):
        robo = self.robo
        if not robo.habilitar_garra or self.garra_aberta or self.garra_fechada:
            return

        robo.publicar_garra(robo.comando_garra_aberta)
        self.garra_aberta = True
        robo.get_logger().info(
            f'Garra: aberta para captura futura {robo.comando_garra_aberta}.'
        )

    def fechar_garra(self):
        robo = self.robo
        if not robo.habilitar_garra or self.garra_fechada:
            return

        robo.publicar_garra(robo.comando_garra_captura)
        self.garra_fechada = True
        robo.get_logger().info(
            f'Garra: comando de captura enviado {robo.comando_garra_captura}.'
        )

    def trocar_estado(self, novo_estado: EstadoMissao, motivo: str):
        if novo_estado == self.estado_atual:
            return

        estado_anterior = self.estado_atual
        self.estado_atual = novo_estado
        self.instante_inicio_estado = time.monotonic()
        self.robo.get_logger().info(
            f'Estado: {estado_anterior.value} -> {novo_estado.value} | {motivo}'
        )

    def log_estado_periodico(self, mensagem: str, periodo: float = 1.0):
        chave = f'estado_{self.estado_atual.value}'
        self.robo.log_periodico(
            chave,
            f'[{self.estado_atual.value}] {mensagem}',
            periodo=periodo,
        )
