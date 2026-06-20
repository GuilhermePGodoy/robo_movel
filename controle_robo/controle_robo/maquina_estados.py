"""Maquina de estados da missao da bandeira azul.

O arquivo ficou separado do no ROS para deixar claro o que e decisao de
controle e o que e infraestrutura de ROS. Assim fica mais facil testar ideias
de navegacao sem mexer em publisher, subscriber ou parametros.
"""

import math
import time

from controle_robo.criterios_visuais import (
    ConfirmadorBandeiraInteira,
    bandeira_parcial_visivel as deteccao_parcial_visivel,
    bandeira_util_para_posicionamento as deteccao_util_para_posicionamento,
)
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
        self.bandeira_capturada = False
        self.garra_entregue = False
        self.estado_retorno_desvio = EstadoMissao.EXPLORANDO
        self.fase_desvio = 'girando'
        self.instante_inicio_avanco_desvio = None
        self.direcao_desvio_atual = 1.0
        self.confirmador_bandeira_inteira = ConfirmadorBandeiraInteira()

        # Tabela simples de despacho: cada estado aponta para o metodo que
        # executa sua regra. Quando entrar um estado novo, ele aparece aqui.
        self.acoes_por_estado = {
            EstadoMissao.EXPLORANDO: self.estado_explorando,
            EstadoMissao.BANDEIRA_DETECTADA: self.estado_bandeira_detectada,
            EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA: (
                self.estado_estimando_posicao_bandeira
            ),
            EstadoMissao.PLANEJANDO_PARA_BANDEIRA: (
                self.estado_planejando_para_bandeira
            ),
            EstadoMissao.SEGUINDO_CAMINHO_PARA_BANDEIRA: (
                self.estado_seguindo_caminho_para_bandeira
            ),
            EstadoMissao.DESVIANDO_OBSTACULO: self.estado_desviando_obstaculo,
            EstadoMissao.REPLANEJANDO_CAMINHO: self.estado_replanejando_caminho,
            EstadoMissao.FALHA_PLANEJAMENTO: self.estado_falha_planejamento,
            EstadoMissao.POSICIONANDO_PARA_COLETA: (
                self.estado_posicionando_para_coleta
            ),
            EstadoMissao.CAPTURANDO_BANDEIRA: self.estado_capturando_bandeira,
            EstadoMissao.PLANEJANDO_RETORNO_BASE: (
                self.estado_planejando_retorno_base
            ),
            EstadoMissao.RETORNANDO_BASE: self.estado_retornando_base,
            EstadoMissao.ENTREGANDO_BANDEIRA: self.estado_entregando_bandeira,
            EstadoMissao.MISSAO_CONCLUIDA: self.estado_missao_concluida,
        }

    def executar(self):
        acao = self.acoes_por_estado.get(self.estado_atual)
        if acao is None:
            self.robo.publicar_velocidade(0.0, 0.0)
            self.robo.get_logger().warn(
                f'Estado desconhecido: {self.estado_atual}. Robo parado.'
            )
            return

        self.atualizar_confirmacao_bandeira_inteira()

        if self.deve_desviar_por_obstaculo_lateral():
            self.apontar_desvio_para_lado_livre()
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                self.motivo_obstaculo_lateral(),
            )
            self.robo.publicar_velocidade(0.0, 0.0)
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
                EstadoMissao.EXPLORANDO,
                'deteccao visual ficou antiga; voltando a explorar',
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
            self.robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira ja esta grande o bastante; usando alinhamento visual',
            )
            return

        self.trocar_estado(
            EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA,
            'bandeira detectada, estimando posicao no mapa',
        )

    def estado_estimando_posicao_bandeira(self):
        robo = self.robo

        if not self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'bandeira sumiu antes da estimativa; voltando a explorar',
            )
            return

        if self.bandeira_pronta_para_posicionamento():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira ja esta proxima antes do A*; usando controle visual',
            )
            return

        estimativa = robo.atualizar_estimativa_bandeira()
        if robo.estimativa_bandeira_confiavel():
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_PARA_BANDEIRA,
                (
                    'estimativa confiavel '
                    f'conf={estimativa.confianca:.2f}, '
                    f'alvo=({estimativa.x:.2f}, {estimativa.y:.2f})'
                ),
            )
            return

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= 5.0:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                (
                    'estimativa visual ainda fraca '
                    f'conf={estimativa.confianca:.2f}; voltando a explorar'
                ),
            )
            return

        det = robo.deteccao_bandeira
        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo durante estimativa visual '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        # A estimativa usa o angulo da bandeira na imagem; ela nao precisa
        # estar centralizada. Aqui o robo so faz um arco leve para melhorar a
        # amostra sem ficar parado girando e perdendo a bandeira.
        angular = 0.5 * self.controle_angular_para_bandeira()
        fator_obstaculo = self.fator_velocidade_por_obstaculo(
            permitir_aceleracao=False
        )
        if self.bandeira_util_para_posicionamento():
            linear = robo.velocidade_exploracao * fator_obstaculo
            acao = 'estimando com a bandeira fora do centro'
        else:
            linear = robo.velocidade_exploracao * fator_obstaculo
            acao = 'mantendo busca em movimento com leitura parcial'

        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                f'{acao}; '
                f'erro={det.erro_x:+.2f}, area={det.area_relativa:.3f}, '
                f'conf={estimativa.confianca:.2f}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd_linear={linear:.2f}, '
                f'cmd_angular={angular:+.2f}'
            ),
            periodo=1.0,
        )

    def estado_planejando_para_bandeira(self):
        robo = self.robo

        if not robo.estimativa_bandeira_confiavel():
            self.trocar_estado(
                EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA,
                'estimativa perdeu confianca antes do planejamento',
            )
            return

        sucesso, motivo = robo.planejar_para_bandeira()
        if sucesso:
            self.trocar_estado(
                EstadoMissao.SEGUINDO_CAMINHO_PARA_BANDEIRA,
                motivo,
            )
        else:
            self.trocar_estado(EstadoMissao.FALHA_PLANEJAMENTO, motivo)

    def estado_seguindo_caminho_para_bandeira(self):
        robo = self.robo

        if self.bandeira_pronta_para_posicionamento():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira grande e alinhada durante A*; usando aproximacao visual',
            )
            return
        if self.bandeira_parcial_visivel():
            det = robo.deteccao_bandeira
            self.log_estado_periodico(
                (
                    'bandeira visivel durante A*, mas ainda nao esta pronta '
                    'para posicionamento visual; '
                    f'area={det.area_relativa:.3f}, '
                    f'min_pos={robo.area_posicionamento_bandeira:.3f}, '
                    f'erro={det.erro_x:+.2f}, box={det.largura}x{det.altura}'
                ),
                periodo=1.0,
            )

        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo imediato durante caminho para bandeira '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        if robo.chegou_perto_da_bandeira_planejada():
            if self.bandeira_util_para_posicionamento():
                robo.limpar_caminho()
                self.trocar_estado(
                    EstadoMissao.POSICIONANDO_PARA_COLETA,
                    'robo chegou ao alvo do A* e tem boa visao da bandeira',
                )
            else:
                self.trocar_estado(
                    EstadoMissao.EXPLORANDO,
                    'chegou ao alvo estimado sem ver a bandeira; explorando',
                )
            return

        if robo.waypoint_bloqueado():
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                'waypoint atual ficou bloqueado no mapa',
            )
            return

        comando = robo.comando_para_waypoint()
        if comando is None:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'caminho para a bandeira terminou sem alvo visual; explorando',
            )
            return

        linear, angular, distancia, erro_yaw = comando
        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'seguindo A* para bandeira; '
                f'wp={robo.indice_waypoint + 1}/{len(robo.caminho_planejado)}, '
                f'dist_wp={distancia:.2f}m, erro_yaw={erro_yaw:+.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=0.8,
        )

    def estado_desviando_obstaculo(self):
        robo = self.robo
        agora = time.monotonic()
        tempo_no_estado = agora - self.instante_inicio_estado
        distancia_lateral = min(robo.distancia_esquerda, robo.distancia_direita)
        laterais_livres = distancia_lateral >= robo.distancia_lateral_desvio

        if (
            self.fase_desvio == 'girando'
            and self.frente_livre_para_avancar_no_desvio()
            and tempo_no_estado >= robo.tempo_minimo_desvio
            and laterais_livres
        ):
            self.fase_desvio = 'avancando'
            self.instante_inicio_avanco_desvio = agora
            robo.get_logger().info(
                'Desvio: frente abriu; avancando em arco antes de replanejar.'
            )

        if self.fase_desvio == 'avancando':
            if robo.obstaculo_a_frente:
                self.fase_desvio = 'girando'
                self.instante_inicio_avanco_desvio = None
                self.instante_inicio_estado = agora
            elif self.tempo_avanco_desvio_concluido(agora):
                self.sair_do_desvio(distancia_lateral)
                return
            else:
                angular = (
                    robo.velocidade_angular_desvio
                    * self.direcao_desvio_atual
                    * 0.35
                )
                linear = min(
                    0.12,
                    max(0.06, robo.velocidade_exploracao * 0.6),
                )
                sentido = (
                    'esquerda'
                    if self.direcao_desvio_atual > 0
                    else 'direita'
                )
                robo.publicar_velocidade(linear, angular)
                self.log_estado_periodico(
                    (
                        f'desviando: avancando em arco para {sentido}; '
                        f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                        f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
                        f'dir={robo.formatar_distancia(robo.distancia_direita)}, '
                        f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
                    ),
                    periodo=0.5,
                )
                return

        if (
            self.fase_desvio == 'girando'
            and self.frente_livre_para_avancar_no_desvio()
            and tempo_no_estado >= robo.tempo_minimo_desvio
            and not laterais_livres
        ):
            self.log_estado_periodico(
                (
                    'frente livre, mas lateral ainda apertada; '
                    f'lateral_min={robo.formatar_distancia(distancia_lateral)}, '
                    f'lateral_minima={robo.distancia_lateral_desvio:.2f}m'
                ),
                periodo=0.8,
            )

        angular = robo.velocidade_angular_desvio * self.direcao_desvio_atual
        sentido = 'esquerda' if self.direcao_desvio_atual > 0 else 'direita'
        robo.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                f'desviando: girando para {sentido}; '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
                f'dir={robo.formatar_distancia(robo.distancia_direita)}, '
                f'lateral_min={robo.formatar_distancia(distancia_lateral)}, '
                f'lateral_minima={robo.distancia_lateral_desvio:.2f}m'
            ),
            periodo=0.8,
        )

    def estado_replanejando_caminho(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        if self.bandeira_capturada:
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
                'replanejando caminho de volta para a base',
            )
        else:
            sucesso, motivo = robo.replanejar_para_bandeira_congelada()
            if sucesso:
                self.trocar_estado(
                    EstadoMissao.SEGUINDO_CAMINHO_PARA_BANDEIRA,
                    motivo,
                )
            else:
                self.trocar_estado(EstadoMissao.FALHA_PLANEJAMENTO, motivo)

    def estado_falha_planejamento(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        tempo_no_estado = time.monotonic() - self.instante_inicio_estado

        if tempo_no_estado < 1.0:
            self.log_estado_periodico(
                'falha de planejamento; aguardando nova leitura do mapa',
                periodo=0.5,
            )
            return

        if self.bandeira_capturada:
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
                'tentando planejar retorno novamente apos falha',
            )
        elif self.bandeira_pronta_para_posicionamento():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'A* falhou, mas a bandeira esta grande/alinhada; usando camera',
            )
        elif robo.tem_alvo_bandeira_congelado():
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                'tentando replanejar novamente ate o alvo congelado',
            )
        elif self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA,
                'tentando melhorar estimativa apos falha do A*',
            )
        else:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'sem caminho e sem bandeira visivel; voltando a explorar',
            )

    def estado_posicionando_para_coleta(self):
        robo = self.robo

        if not self.bandeira_util_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                'visao da bandeira ficou pequena demais; voltando a explorar',
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
        self.bandeira_capturada = True

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= 1.0:
            self.trocar_estado(
                EstadoMissao.MISSAO_CONCLUIDA,
                'bandeira capturada; encerrando esta etapa da missao',
            )
            return

        self.log_estado_periodico(
            (
                'capturando bandeira; robo parado enquanto a garra fecha '
                f'em pose=({robo.x:.2f}, {robo.y:.2f}).'
            ),
            periodo=0.5,
        )

    def estado_planejando_retorno_base(self):
        robo = self.robo
        sucesso, motivo = robo.planejar_para_base()
        if sucesso:
            self.trocar_estado(EstadoMissao.RETORNANDO_BASE, motivo)
        else:
            self.trocar_estado(EstadoMissao.FALHA_PLANEJAMENTO, motivo)

    def estado_retornando_base(self):
        robo = self.robo

        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo imediato durante retorno '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        if robo.chegou_na_base():
            self.trocar_estado(
                EstadoMissao.ENTREGANDO_BANDEIRA,
                'robo chegou na base inicial',
            )
            return

        if robo.waypoint_bloqueado():
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                'waypoint de retorno ficou bloqueado no mapa',
            )
            return

        comando = robo.comando_para_waypoint()
        if comando is None:
            self.trocar_estado(
                EstadoMissao.ENTREGANDO_BANDEIRA,
                'caminho de retorno terminou perto da base',
            )
            return

        linear, angular, distancia, erro_yaw = comando
        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'retornando para base por A*; '
                f'wp={robo.indice_waypoint + 1}/{len(robo.caminho_planejado)}, '
                f'dist_wp={distancia:.2f}m, erro_yaw={erro_yaw:+.2f}, '
                f'cmd_linear={linear:.2f}, cmd_angular={angular:+.2f}'
            ),
            periodo=0.8,
        )

    def estado_entregando_bandeira(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        self.soltar_garra()

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= 1.0:
            self.trocar_estado(
                EstadoMissao.MISSAO_CONCLUIDA,
                'bandeira entregue na base',
            )
            return

        self.log_estado_periodico(
            'entregando bandeira na base; aguardando garra abrir',
            periodo=0.5,
        )

    def estado_missao_concluida(self):
        self.robo.publicar_velocidade(0.0, 0.0)
        self.log_estado_periodico(
            'missao concluida ou pausada no objetivo atual; robo parado',
            periodo=2.0,
        )

    # Predicados pequenos deixam as transicoes mais legiveis: os estados leem
    # quase como uma frase, e os detalhes ficam concentrados aqui embaixo.
    def bandeira_recente(self):
        return (
            self.robo.deteccao_bandeira.visivel
            and self.tempo_desde_bandeira() <= self.robo.tempo_perda_bandeira
        )

    def bandeira_parcial_visivel(self):
        return (
            self.bandeira_recente()
            and deteccao_parcial_visivel(self.robo.deteccao_bandeira)
        )

    def bandeira_util_para_posicionamento(self):
        if not self.bandeira_parcial_visivel():
            return False

        return (
            self.bandeira_inteira_visivel()
            or deteccao_util_para_posicionamento(
                self.robo.deteccao_bandeira,
                self.robo.area_posicionamento_bandeira,
            )
        )

    def atualizar_confirmacao_bandeira_inteira(self):
        if not self.bandeira_recente():
            self.confirmador_bandeira_inteira.frames_confirmados = 0
            return

        self.confirmador_bandeira_inteira.atualizar(
            self.robo.deteccao_bandeira,
            self.robo.margem_borda_bandeira_px,
            self.robo.area_minima_bandeira_inteira,
        )

    def bandeira_inteira_visivel(self):
        return self.confirmador_bandeira_inteira.confirmada(
            self.robo.frames_bandeira_inteira
        )

    def tempo_desde_bandeira(self):
        if self.robo.ultimo_instante_bandeira is None:
            return math.inf

        return time.monotonic() - self.robo.ultimo_instante_bandeira

    def deve_desviar_por_obstaculo_lateral(self):
        if self.estado_atual in (
            EstadoMissao.DESVIANDO_OBSTACULO,
            EstadoMissao.CAPTURANDO_BANDEIRA,
            EstadoMissao.ENTREGANDO_BANDEIRA,
            EstadoMissao.MISSAO_CONCLUIDA,
        ):
            return False

        return (
            self.menor_distancia_lateral()
            < self.robo.distancia_lateral_desvio
        )

    def menor_distancia_lateral(self):
        return min(self.robo.distancia_esquerda, self.robo.distancia_direita)

    def apontar_desvio_para_lado_livre(self):
        robo = self.robo
        robo.direcao_desvio = (
            1.0 if robo.distancia_esquerda >= robo.distancia_direita else -1.0
        )

    def motivo_obstaculo_lateral(self):
        robo = self.robo
        lado_apertado = (
            'esquerdo'
            if robo.distancia_esquerda < robo.distancia_direita
            else 'direito'
        )
        lado_desvio = 'esquerda' if robo.direcao_desvio > 0.0 else 'direita'
        return (
            f'obstaculo lateral {lado_apertado} '
            f'({self.menor_distancia_lateral():.2f} m); '
            f'desviando para {lado_desvio}'
        )

    def frente_livre_para_avancar_no_desvio(self):
        distancia_segura = self.robo.distancia_obstaculo + 0.18
        return (
            math.isinf(self.robo.distancia_frontal)
            or self.robo.distancia_frontal >= distancia_segura
        )

    def tempo_avanco_desvio_concluido(self, agora):
        if self.instante_inicio_avanco_desvio is None:
            return False

        tempo_avanco = agora - self.instante_inicio_avanco_desvio
        tempo_minimo = max(1.0, self.robo.tempo_minimo_desvio)
        return tempo_avanco >= tempo_minimo

    def sair_do_desvio(self, distancia_lateral):
        if (
            self.estado_retorno_desvio == EstadoMissao.POSICIONANDO_PARA_COLETA
            and self.bandeira_pronta_para_posicionamento()
        ):
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                (
                    'obstaculo contornado e bandeira ainda visivel; '
                    'retomando aproximacao fina'
                ),
            )
        elif (
            self.estado_retorno_desvio
            == EstadoMissao.SEGUINDO_CAMINHO_PARA_BANDEIRA
            and self.bandeira_pronta_para_posicionamento()
        ):
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                (
                    'obstaculo contornado e bandeira visivel; '
                    'trocando A* por controle visual'
                ),
            )
        elif self.estado_retorno_desvio in (
            EstadoMissao.SEGUINDO_CAMINHO_PARA_BANDEIRA,
            EstadoMissao.RETORNANDO_BASE,
        ):
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); replanejando rota'
                ),
            )
        elif self.bandeira_capturada:
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); retomando retorno'
                ),
            )
        elif self.bandeira_recente():
            self.trocar_estado(
                EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA,
                (
                    'obstaculo contornado e bandeira visivel '
                    f'({distancia_lateral:.2f} m lateral)'
                ),
            )
        else:
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); retomando busca'
                ),
            )

    def bandeira_pronta_para_posicionamento(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        centralizada = abs(det.erro_x) <= robo.erro_alinhamento_bandeira * 1.5
        proxima_por_imagem = (
            det.area_relativa >= robo.area_posicionamento_bandeira
        )
        return self.bandeira_parcial_visivel() and centralizada and proxima_por_imagem

    def bandeira_pronta_para_coleta(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        centralizada = abs(det.erro_x) <= robo.erro_alinhamento_bandeira
        proxima_por_imagem = det.area_relativa >= robo.area_coleta_bandeira
        return self.bandeira_parcial_visivel() and centralizada and proxima_por_imagem

    def obstaculo_deve_ser_desviado_no_posicionamento(self):
        robo = self.robo
        det = robo.deteccao_bandeira

        # O LIDAR nao sabe diferenciar bandeira de obstaculo. So ignoramos uma
        # leitura frontal se a imagem ja diz que a bandeira esta bem grande,
        # isto e, plausivelmente perto da garra.
        area_para_confiar_no_alvo = max(
            robo.area_posicionamento_bandeira * 1.5,
            robo.area_coleta_bandeira * 0.7,
        )
        alvo_de_coleta_provavel = (
            abs(det.erro_x) <= robo.erro_alinhamento_bandeira
            and det.area_relativa >= area_para_confiar_no_alvo
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

    def soltar_garra(self):
        robo = self.robo
        if not robo.habilitar_garra or self.garra_entregue:
            return

        robo.publicar_garra(robo.comando_garra_aberta)
        self.garra_entregue = True
        self.garra_fechada = False
        robo.get_logger().info(
            f'Garra: bandeira entregue, abrindo {robo.comando_garra_aberta}.'
        )

    def trocar_estado(self, novo_estado: EstadoMissao, motivo: str):
        if novo_estado == self.estado_atual:
            return

        estado_anterior = self.estado_atual
        if novo_estado == EstadoMissao.DESVIANDO_OBSTACULO:
            self.estado_retorno_desvio = estado_anterior
            self.fase_desvio = 'girando'
            self.instante_inicio_avanco_desvio = None
            self.direcao_desvio_atual = self.robo.direcao_desvio
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
