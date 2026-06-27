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
from controle_robo.garra import ControleGarra
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
        self.instante_inicio_missao = self.instante_inicio_estado
        self.ultimo_inicio_exploracao_desconhecida = -math.inf
        self.garra = ControleGarra(robo)
        self.bandeira_capturada = False
        self.bandeira_entregue = False
        self.estado_retorno_desvio = EstadoMissao.EXPLORANDO
        self.fase_desvio = 'girando'
        self.instante_inicio_avanco_desvio = None
        self.direcao_desvio_atual = 1.0
        self.direcao_redeteccao_bandeira = 1.0
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
            EstadoMissao.PLANEJANDO_EXPLORACAO_DESCONHECIDA: (
                self.estado_planejando_exploracao_desconhecida
            ),
            EstadoMissao.SEGUINDO_CAMINHO_EXPLORACAO: (
                self.estado_seguindo_caminho_exploracao
            ),
            EstadoMissao.DESVIANDO_OBSTACULO: self.estado_desviando_obstaculo,
            EstadoMissao.REPLANEJANDO_CAMINHO: self.estado_replanejando_caminho,
            EstadoMissao.FALHA_PLANEJAMENTO: self.estado_falha_planejamento,
            EstadoMissao.REENCONTRANDO_BANDEIRA: (
                self.estado_reencontrando_bandeira
            ),
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
        self.atualizar_memoria_direcao_bandeira()

        if self.deve_desviar_por_obstaculo_lateral():
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                self.motivo_obstaculo_lateral(),
            )
            self.robo.publicar_velocidade(0.0, 0.0)
            return

        acao()

    def estado_explorando(self):
        robo = self.robo

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

        if self.deve_explorar_fronteira_desconhecida():
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_EXPLORACAO_DESCONHECIDA,
                (
                    'tempo sem ver bandeira excedeu '
                    f'{robo.timeout_exploracao_desconhecida:.0f}s; '
                    'tentando avancar para regiao desconhecida do mapa'
                ),
            )
            robo.publicar_velocidade(0.0, 0.0)
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
                'acao=explorar_curva | '
                f'pose=({robo.x:.2f}, {robo.y:.2f}, yaw={robo.yaw:.2f}), '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
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
                'visao=bandeira | '
                f'area={det.area_relativa:.3f}, '
                f'erro_blob={det.erro_x:+.2f}, '
                f'erro_haste={det.erro_x_haste:+.2f}'
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
                f'acao={acao}; '
                f'erro_blob={det.erro_x:+.2f}, '
                f'erro_haste={det.erro_x_haste:+.2f}, '
                f'area={det.area_relativa:.3f}, '
                f'conf={estimativa.confianca:.2f}, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
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
                (
                    'bandeira grande o bastante na camera; '
                    'abandonando A* e usando aproximacao visual'
                ),
            )
            return

        if self.bandeira_inteira_visivel():
            robo.atualizar_estimativa_bandeira()
            if robo.alvo_bandeira_mudou_para_replanejar():
                self.trocar_estado(
                    EstadoMissao.REPLANEJANDO_CAMINHO,
                    'bandeira inteira vista durante A*; atualizando alvo',
                )
                return

        if self.bandeira_parcial_visivel():
            det = robo.deteccao_bandeira
            self.log_estado_periodico(
                (
                    'visao=bandeira_parcial | aguardando_posicionamento | '
                    f'area={det.area_relativa:.3f}, '
                    f'min_pos={robo.area_posicionamento_bandeira:.3f}, '
                    f'erro_haste={det.erro_x_haste:+.2f}, '
                    f'box={det.largura}x{det.altura}'
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
            if self.bandeira_pronta_para_posicionamento():
                robo.limpar_caminho()
                self.trocar_estado(
                    EstadoMissao.POSICIONANDO_PARA_COLETA,
                    'robo chegou ao alvo do A* e tem boa visao da bandeira',
                )
            elif self.bandeira_recente():
                robo.limpar_caminho()
                self.trocar_estado(
                    EstadoMissao.ESTIMANDO_POSICAO_BANDEIRA,
                    (
                        'chegou ao alvo do A*, mas ainda nao esta pronto '
                        'para coleta; recalculando estimativa local'
                    ),
                )
            else:
                self.trocar_estado(
                    EstadoMissao.FALHA_PLANEJAMENTO,
                    (
                        'chegou ao alvo do A* sem leitura visual recente; '
                        'evitando voltar direto para exploracao'
                    ),
                )
            return

        if robo.waypoint_bloqueado():
            if robo.waypoint_bloqueado_eh_final():
                if robo.obstaculo_a_frente:
                    self.trocar_estado(
                        EstadoMissao.DESVIANDO_OBSTACULO,
                        (
                            'waypoint final esta ocupado e ha obstaculo '
                            'na frente; tentando contornar localmente'
                        ),
                    )
                else:
                    self.trocar_estado(
                        EstadoMissao.FALHA_PLANEJAMENTO,
                        (
                            'waypoint final ficou ocupado no mapa; '
                            'fazendo busca local antes de desistir do alvo'
                        ),
                    )
                return

            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                'waypoint atual ficou bloqueado no mapa',
            )
            return

        comando = robo.comando_para_waypoint()
        if comando is None:
            self.trocar_estado(
                EstadoMissao.FALHA_PLANEJAMENTO,
                'caminho para a bandeira terminou; recuperando planejamento',
            )
            return

        linear, angular, distancia, erro_yaw = comando
        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'acao=seguir_astar_bandeira | '
                f'wp={robo.indice_waypoint + 1}/{len(robo.caminho_planejado)}, '
                f'dist_wp={distancia:.2f}m, erro_yaw={erro_yaw:+.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
            ),
            periodo=0.8,
        )

    def estado_planejando_exploracao_desconhecida(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        self.ultimo_inicio_exploracao_desconhecida = time.monotonic()

        if self.bandeira_recente():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'bandeira apareceu antes da exploracao por fronteira',
            )
            return

        sucesso, motivo = robo.planejar_para_desconhecido()
        if sucesso:
            self.trocar_estado(
                EstadoMissao.SEGUINDO_CAMINHO_EXPLORACAO,
                motivo,
            )
            return

        self.trocar_estado(
            EstadoMissao.EXPLORANDO,
            f'exploracao por fronteira nao gerou rota; {motivo}',
        )

    def estado_seguindo_caminho_exploracao(self):
        robo = self.robo

        if self.bandeira_recente():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.BANDEIRA_DETECTADA,
                'bandeira encontrada durante ida para regiao desconhecida',
            )
            return

        if robo.obstaculo_a_frente:
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo imediato durante exploracao por fronteira '
                    f'({robo.distancia_frontal:.2f} m)'
                ),
            )
            return

        if robo.waypoint_bloqueado():
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_EXPLORACAO_DESCONHECIDA,
                'waypoint da exploracao desconhecida ficou bloqueado no mapa',
            )
            return

        comando = robo.comando_para_waypoint()
        if comando is None:
            robo.limpar_caminho()
            self.ultimo_inicio_exploracao_desconhecida = time.monotonic()
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                (
                    'alvo desconhecido alcancado; retomando busca visual '
                    'normal pela bandeira'
                ),
            )
            return

        linear, angular, distancia, erro_yaw = comando
        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'acao=seguir_astar_desconhecido | '
                f'wp={robo.indice_waypoint + 1}/{len(robo.caminho_planejado)}, '
                f'dist_wp={distancia:.2f}m, erro_yaw={erro_yaw:+.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
            ),
            periodo=0.8,
        )

    def estado_desviando_obstaculo(self):
        robo = self.robo
        agora = time.monotonic()
        tempo_no_estado = agora - self.instante_inicio_estado
        distancia_frontal = self.distancia_frontal_ativa()
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
                'DESVIO | fase=arco | motivo=frente_e_laterais_livres'
            )

        if self.fase_desvio == 'avancando':
            if self.obstaculo_a_frente_ativo():
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
                        f'fase=arco | lado={sentido} | '
                        f'frente={robo.formatar_distancia(distancia_frontal)}, '
                        f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
                        f'dir={robo.formatar_distancia(robo.distancia_direita)}, '
                        f'esq_fr={robo.formatar_distancia(robo.distancia_esquerda_frente)}, '
                        f'dir_fr={robo.formatar_distancia(robo.distancia_direita_frente)}, '
                        f'cmd=({linear:.2f}, {angular:+.2f})'
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
                    'fase=giro | motivo=lateral_apertada | '
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
                f'fase=giro | lado={sentido} | '
                f'frente={robo.formatar_distancia(distancia_frontal)}, '
                f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
                f'dir={robo.formatar_distancia(robo.distancia_direita)}, '
                f'esq_fr={robo.formatar_distancia(robo.distancia_esquerda_frente)}, '
                f'dir_fr={robo.formatar_distancia(robo.distancia_direita_frente)}, '
                f'lateral_min={robo.formatar_distancia(distancia_lateral)}, '
                f'lateral_minima={robo.distancia_lateral_desvio:.2f}m, '
                f'cmd=(0.00, {angular:+.2f})'
            ),
            periodo=0.8,
        )

    def estado_replanejando_caminho(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        if self.bandeira_capturada:
            sucesso, motivo = robo.planejar_para_base()
            if sucesso:
                self.trocar_estado(
                    EstadoMissao.RETORNANDO_BASE,
                    motivo,
                )
            else:
                self.trocar_estado(EstadoMissao.FALHA_PLANEJAMENTO, motivo)
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

        if self.bandeira_capturada:
            if tempo_no_estado < 1.0:
                self.log_estado_periodico(
                    'falha no retorno; aguardando nova leitura do mapa',
                    periodo=0.5,
                )
                return

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
        elif (
            robo.tem_alvo_bandeira_congelado()
            and robo.chegou_perto_da_bandeira_planejada()
        ):
            if robo.obstaculo_a_frente:
                self.trocar_estado(
                    EstadoMissao.DESVIANDO_OBSTACULO,
                    (
                        'perto do alvo congelado sem bandeira clara, mas ha '
                        'obstaculo a frente; tentando abrir visada'
                    ),
                )
                return

            if tempo_no_estado < 4.0:
                angular = robo.velocidade_giro_busca * 0.55
                robo.publicar_velocidade(0.0, angular)
                self.log_estado_periodico(
                    (
                        'acao=busca_local | motivo=alvo_astar_sem_bandeira | '
                        f'cmd=(0.00, {angular:+.2f})'
                    ),
                    periodo=0.7,
                )
                return

            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.EXPLORANDO,
                (
                    'busca local no alvo nao reencontrou a bandeira; '
                    'retomando exploracao ampla'
                ),
            )
        elif tempo_no_estado < 1.0:
            self.log_estado_periodico(
                'falha de planejamento; aguardando nova leitura do mapa',
                periodo=0.5,
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

    def estado_reencontrando_bandeira(self):
        robo = self.robo
        tempo_no_estado = time.monotonic() - self.instante_inicio_estado

        if (
            self.bandeira_pronta_para_posicionamento()
            or self.bandeira_centralizada_para_posicionamento()
        ):
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.POSICIONANDO_PARA_COLETA,
                'bandeira reenquadrada; retomando aproximacao fina',
            )
            return

        if tempo_no_estado >= robo.tempo_redeteccao_bandeira:
            if robo.tem_alvo_bandeira_congelado():
                self.trocar_estado(
                    EstadoMissao.REPLANEJANDO_CAMINHO,
                    (
                        'redeteccao visual esgotou tempo; '
                        'voltando ao alvo estimado'
                    ),
                )
            else:
                self.trocar_estado(
                    EstadoMissao.EXPLORANDO,
                    (
                        'redeteccao visual nao reencontrou a bandeira; '
                        'retomando exploracao ampla'
                    ),
                )
            return

        if self.bandeira_parcial_visivel():
            angular = self.controle_angular_para_bandeira()
            if abs(angular) < 0.08:
                angular = 0.4 * robo.velocidade_giro_busca * (
                    self.direcao_redeteccao_bandeira
                )
            acao = 'reenquadrar pela haste'
        else:
            angular = (
                robo.velocidade_giro_busca
                * self.direcao_redeteccao_bandeira
            )
            acao = 'varrer ultimo lado visto'

        robo.publicar_velocidade(0.0, angular)
        self.log_estado_periodico(
            (
                f'acao={acao}; '
                f'tempo={tempo_no_estado:.1f}/{robo.tempo_redeteccao_bandeira:.1f}s, '
                f'direcao={"esquerda" if angular > 0.0 else "direita"}, '
                f'{self.resumo_visao()}, cmd=(0.00, {angular:+.2f})'
            ),
            periodo=0.5,
        )

    def estado_posicionando_para_coleta(self):
        robo = self.robo

        if not self.bandeira_util_para_posicionamento():
            self.trocar_estado(
                EstadoMissao.REENCONTRANDO_BANDEIRA,
                (
                    'visao da bandeira ficou pequena demais; '
                    'tentando reenquadrar antes de explorar'
                ),
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

        if abs(self.erro_x_haste()) > robo.erro_alinhamento_bandeira:
            linear = 0.0
            fator_obstaculo = 1.0
            acao = 'ajustando orientacao pela haste'
        else:
            fator_obstaculo = self.fator_velocidade_por_obstaculo(
                permitir_aceleracao=False
            )
            linear = robo.velocidade_posicionamento * fator_obstaculo
            acao = 'aproximando devagar'

        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                f'acao={acao}; erro_haste={det.erro_x_haste:+.2f}, '
                f'erro_blob={det.erro_x:+.2f}, '
                f'area={det.area_relativa:.3f}, '
                f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                f'fr_coleta={robo.formatar_distancia(robo.distancia_frontal_coleta)}, '
                f'area_coleta={robo.area_coleta_bandeira:.3f}, '
                f'dist_coleta={robo.distancia_coleta_bandeira:.2f}m, '
                f'fator_vel={fator_obstaculo:.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
            ),
            periodo=0.7,
        )

    def estado_capturando_bandeira(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        self.fechar_garra(forcar=True)
        self.bandeira_capturada = True

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= 1.0:
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
                'bandeira capturada; planejando retorno para a base',
            )
            return

        self.log_estado_periodico(
            (
                'acao=fechar_garra | cmd=(0.00, 0.00) | '
                f'pose=({robo.x:.2f}, {robo.y:.2f})'
            ),
            periodo=0.5,
        )

    def estado_planejando_retorno_base(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)

        if robo.chegou_na_base():
            self.trocar_estado(
                EstadoMissao.ENTREGANDO_BANDEIRA,
                'robo ja esta na base inicial; entregando bandeira',
            )
            return

        sucesso, motivo = robo.planejar_para_base()
        if sucesso:
            self.trocar_estado(
                EstadoMissao.RETORNANDO_BASE,
                motivo,
            )
        else:
            self.trocar_estado(
                EstadoMissao.FALHA_PLANEJAMENTO,
                motivo,
            )

    def estado_retornando_base(self):
        robo = self.robo
        self.fechar_garra(forcar=True)

        if robo.chegou_na_base():
            robo.limpar_caminho()
            self.trocar_estado(
                EstadoMissao.ENTREGANDO_BANDEIRA,
                'robo chegou na base inicial',
            )
            return

        if self.obstaculo_a_frente_ativo():
            self.trocar_estado(
                EstadoMissao.DESVIANDO_OBSTACULO,
                (
                    'obstaculo imediato durante retorno para base '
                    f'({self.distancia_frontal_ativa():.2f} m)'
                ),
            )
            return

        if robo.waypoint_bloqueado():
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                'waypoint do retorno ficou bloqueado no mapa',
            )
            return

        waypoint_pulado = robo.pular_waypoint_ruim_no_retorno()
        if waypoint_pulado is not None:
            indice_pulado, distancia, erro_yaw = waypoint_pulado
            self.log_estado_periodico(
                (
                    'acao=pular_waypoint_retorno | '
                    f'wp_pulado={indice_pulado + 1}/'
                    f'{len(robo.caminho_planejado)}, '
                    f'dist_wp={distancia:.2f}m, '
                    f'erro_yaw={erro_yaw:+.2f}, '
                    'motivo=perto_demais_para_giro_puro_com_bandeira'
                ),
                periodo=0.2,
            )
            robo.publicar_velocidade(0.0, 0.0)
            return

        comando = robo.comando_para_waypoint(self.distancia_frontal_ativa())
        if comando is None:
            self.trocar_estado(
                EstadoMissao.FALHA_PLANEJAMENTO,
                'caminho de retorno terminou antes da base',
            )
            return

        linear, angular, distancia, erro_yaw = comando
        robo.publicar_velocidade(linear, angular)
        self.log_estado_periodico(
            (
                'acao=seguir_astar_base | '
                f'wp={robo.indice_waypoint + 1}/{len(robo.caminho_planejado)}, '
                f'dist_wp={distancia:.2f}m, erro_yaw={erro_yaw:+.2f}, '
                f'cmd=({linear:.2f}, {angular:+.2f})'
            ),
            periodo=0.8,
        )

    def estado_entregando_bandeira(self):
        robo = self.robo
        robo.publicar_velocidade(0.0, 0.0)
        self.soltar_garra(forcar=True)

        tempo_no_estado = time.monotonic() - self.instante_inicio_estado
        if tempo_no_estado >= 1.0:
            self.bandeira_entregue = True
            self.trocar_estado(
                EstadoMissao.MISSAO_CONCLUIDA,
                'bandeira depositada na base inicial',
            )
            return

        self.log_estado_periodico(
            'acao=abrir_garra | motivo=depositar_bandeira | cmd=(0.00, 0.00)',
            periodo=0.5,
        )

    def estado_missao_concluida(self):
        self.robo.publicar_velocidade(0.0, 0.0)
        if self.bandeira_entregue:
            detalhe = 'bandeira entregue na base; garra aberta'
        elif self.bandeira_capturada:
            self.fechar_garra(forcar=True)
            detalhe = 'bandeira capturada; mantendo garra fechada'
        else:
            detalhe = 'missao concluida ou pausada no objetivo atual'

        self.log_estado_periodico(
            f'acao=parado | {detalhe}',
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

    def bandeira_centralizada_para_posicionamento(self):
        det = self.robo.deteccao_bandeira
        return (
            self.bandeira_parcial_visivel()
            and det.area_relativa >= self.robo.area_minima_bandeira_inteira
            and abs(self.erro_x_haste())
            <= self.robo.erro_alinhamento_bandeira * 1.5
        )

    def atualizar_memoria_direcao_bandeira(self):
        if not self.bandeira_parcial_visivel():
            return

        erro = self.erro_x_haste()
        if abs(erro) <= 0.05:
            return

        # Erro negativo significa haste a esquerda da imagem. Para reencontrar,
        # o robo deve girar para a esquerda; em ROS isso e angular.z positivo.
        self.direcao_redeteccao_bandeira = -1.0 if erro > 0.0 else 1.0

    def tempo_desde_bandeira(self):
        if self.robo.ultimo_instante_bandeira is None:
            return math.inf

        return time.monotonic() - self.robo.ultimo_instante_bandeira

    def tempo_sem_ver_bandeira(self):
        if self.robo.ultimo_instante_bandeira is None:
            return time.monotonic() - self.instante_inicio_missao

        return self.tempo_desde_bandeira()

    def deve_explorar_fronteira_desconhecida(self):
        robo = self.robo
        if not robo.habilitar_exploracao_desconhecida:
            return False
        if not robo.usar_planejamento_grade or robo.mapa_grade is None:
            return False
        if self.bandeira_recente():
            return False
        if self.tempo_sem_ver_bandeira() < robo.timeout_exploracao_desconhecida:
            return False

        agora = time.monotonic()
        return (
            agora - self.ultimo_inicio_exploracao_desconhecida
            >= robo.intervalo_exploracao_desconhecida
        )

    def deve_desviar_por_obstaculo_lateral(self):
        if self.estado_atual in (
            EstadoMissao.DESVIANDO_OBSTACULO,
            EstadoMissao.REENCONTRANDO_BANDEIRA,
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

    def motivo_obstaculo_lateral(self):
        robo = self.robo
        lado_apertado = (
            'esquerdo'
            if robo.distancia_esquerda < robo.distancia_direita
            else 'direito'
        )
        return (
            f'obstaculo lateral {lado_apertado} '
            f'({self.menor_distancia_lateral():.2f} m); '
            f'esq_frente={robo.formatar_distancia(robo.distancia_esquerda_frente)}, '
            f'dir_frente={robo.formatar_distancia(robo.distancia_direita_frente)}, '
            f'desviando para {robo.nome_lado_desvio()}'
        )

    def frente_livre_para_avancar_no_desvio(self):
        distancia_segura = self.robo.distancia_obstaculo + 0.18
        distancia_frontal = self.distancia_frontal_ativa()
        return (
            math.isinf(distancia_frontal)
            or distancia_frontal >= distancia_segura
        )

    def obstaculo_a_frente_ativo(self):
        if self.ignorar_centro_lidar_por_bandeira():
            return self.robo.obstaculo_a_frente_sem_centro

        return self.robo.obstaculo_a_frente

    def distancia_frontal_ativa(self):
        if self.ignorar_centro_lidar_por_bandeira():
            return self.robo.distancia_frontal_sem_centro

        return self.robo.distancia_frontal

    def ignorar_centro_lidar_por_bandeira(self):
        return (
            self.bandeira_capturada
            and self.estado_atual in (
                EstadoMissao.DESVIANDO_OBSTACULO,
                EstadoMissao.RETORNANDO_BASE,
                EstadoMissao.REPLANEJANDO_CAMINHO,
                EstadoMissao.FALHA_PLANEJAMENTO,
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
            )
        )

    def tempo_avanco_desvio_concluido(self, agora):
        if self.instante_inicio_avanco_desvio is None:
            return False

        tempo_avanco = agora - self.instante_inicio_avanco_desvio
        tempo_minimo = max(1.0, self.robo.tempo_minimo_desvio)
        return tempo_avanco >= tempo_minimo

    def sair_do_desvio(self, distancia_lateral):
        robo = self.robo

        if self.estado_retorno_desvio == EstadoMissao.POSICIONANDO_PARA_COLETA:
            if (
                self.bandeira_pronta_para_posicionamento()
                or self.bandeira_centralizada_para_posicionamento()
            ):
                self.trocar_estado(
                    EstadoMissao.POSICIONANDO_PARA_COLETA,
                    (
                        'obstaculo contornado e bandeira reenquadrada; '
                        'retomando aproximacao fina'
                    ),
                )
            else:
                self.trocar_estado(
                    EstadoMissao.REENCONTRANDO_BANDEIRA,
                    (
                        'obstaculo contornado perto da coleta; '
                        'reenquadrando bandeira antes de decidir rota'
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
            EstadoMissao.REPLANEJANDO_CAMINHO,
            EstadoMissao.FALHA_PLANEJAMENTO,
        ) and robo.tem_alvo_bandeira_congelado():
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); replanejando rota'
                ),
            )
        elif self.estado_retorno_desvio in (
            EstadoMissao.SEGUINDO_CAMINHO_EXPLORACAO,
            EstadoMissao.PLANEJANDO_EXPLORACAO_DESCONHECIDA,
        ):
            self.trocar_estado(
                EstadoMissao.PLANEJANDO_EXPLORACAO_DESCONHECIDA,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); '
                    'buscando nova fronteira desconhecida'
                ),
            )
        elif (
            self.bandeira_capturada
            and self.estado_retorno_desvio in (
                EstadoMissao.RETORNANDO_BASE,
                EstadoMissao.PLANEJANDO_RETORNO_BASE,
                EstadoMissao.REPLANEJANDO_CAMINHO,
                EstadoMissao.FALHA_PLANEJAMENTO,
            )
        ):
            self.trocar_estado(
                EstadoMissao.REPLANEJANDO_CAMINHO,
                (
                    'obstaculo contornado com avanco em arco '
                    f'({distancia_lateral:.2f} m lateral); '
                    'replanejando retorno para base'
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
        """Diz quando a camera ja deve assumir o ajuste final.

        Perto da bandeira, pequenas mudancas na estimativa podem fazer o A*
        escolher um caminho ruim. Por isso usamos um criterio bem direto:
        quando a bandeira ocupa area suficiente na imagem, o planejamento
        global para e o controle visual passa a mirar a haste.
        """

        return self.bandeira_util_para_posicionamento()

    def bandeira_pronta_para_coleta(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        centralizada = abs(self.erro_x_haste()) <= robo.erro_alinhamento_bandeira
        visual_minimo = det.area_relativa >= robo.area_coleta_bandeira
        proxima_por_lidar = (
            math.isfinite(robo.distancia_frontal_coleta)
            and robo.distancia_frontal_coleta <= robo.distancia_coleta_bandeira
        )
        return (
            self.bandeira_parcial_visivel()
            and centralizada
            and visual_minimo
            and proxima_por_lidar
        )

    def obstaculo_deve_ser_desviado_no_posicionamento(self):
        robo = self.robo
        det = robo.deteccao_bandeira

        if self.bandeira_pronta_para_coleta():
            robo.log_periodico(
                'posicionamento_lidar_alvo',
                (
                    'LIDAR | frente_ocupada_pelo_alvo=confirmado | '
                    f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                    f'fr_coleta={robo.formatar_distancia(robo.distancia_frontal_coleta)}, '
                    f'area={det.area_relativa:.3f}, '
                    f'area_coleta={robo.area_coleta_bandeira:.3f}, '
                    f'erro_haste={det.erro_x_haste:+.2f}, '
                    f'dist_coleta={robo.distancia_coleta_bandeira:.2f}m | '
                    'acao=permitir_captura'
                ),
                periodo=1.0,
            )
            return False

        # Se a haste esta centralizada e a bandeira ja tem tamanho suficiente
        # para aproximacao visual, o alvo da garra e justamente o objeto que
        # aparece no LIDAR. A area de coleta continua sendo exigida para fechar
        # a garra, mas nao para permitir a aproximacao final.
        area_minima_aproximacao = 0.8 * min(
            robo.area_posicionamento_bandeira,
            robo.area_coleta_bandeira,
        )
        alvo_visual_alinhado = (
            self.bandeira_parcial_visivel()
            and abs(self.erro_x_haste()) <= robo.erro_alinhamento_bandeira
            and det.area_relativa >= area_minima_aproximacao
            and math.isfinite(robo.distancia_frontal_coleta)
        )
        # Quando a bandeira e vista de lado, alguns raios fora do centro tambem
        # batem nela. Entao so desviamos se a leitura fora do centro estiver
        # claramente mais perto que a leitura central de coleta.
        margem_mesmo_alvo = max(0.12, 0.25 * robo.distancia_obstaculo)
        fora_do_centro_compativel_com_alvo = (
            not robo.obstaculo_a_frente_sem_centro
            or math.isinf(robo.distancia_frontal_sem_centro)
            or robo.distancia_frontal_sem_centro
            >= robo.distancia_frontal_coleta - margem_mesmo_alvo
        )
        alvo_central_em_aproximacao = (
            alvo_visual_alinhado
            and fora_do_centro_compativel_com_alvo
        )
        if alvo_central_em_aproximacao:
            robo.log_periodico(
                'posicionamento_lidar_alvo',
                (
                    'LIDAR | frente_ocupada_pela_haste=provavel | '
                    f'frente={robo.formatar_distancia(robo.distancia_frontal)}, '
                    f'fr_coleta={robo.formatar_distancia(robo.distancia_frontal_coleta)}, '
                    f'fr_sem_centro={robo.formatar_distancia(robo.distancia_frontal_sem_centro)}, '
                    f'margem={margem_mesmo_alvo:.2f}m, '
                    f'area={det.area_relativa:.3f}, '
                    f'area_aprox={area_minima_aproximacao:.3f}, '
                    f'area_coleta={robo.area_coleta_bandeira:.3f}, '
                    f'erro_haste={det.erro_x_haste:+.2f} | '
                    'acao=continuar_aproximando'
                ),
                periodo=0.8,
            )
            return False

        return True

    def controle_angular_para_bandeira(self):
        robo = self.robo
        erro = self.erro_x_haste()

        # Erro positivo significa bandeira a direita da imagem. Em ROS,
        # angular.z negativo gira o robo para a direita.
        angular = -robo.ganho_angular_bandeira * erro
        return robo.limitar(
            angular,
            -robo.velocidade_giro_busca,
            robo.velocidade_giro_busca,
        )

    def erro_x_haste(self):
        return self.robo.deteccao_bandeira.erro_x_haste

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

    # Comandos auxiliares. A garra so abre quando o robo ja entrou na fase
    # final de posicionamento; ate la ela nao precisa ficar aberta.
    def abrir_garra(self, forcar: bool = False):
        self.garra.abrir_para_coleta(forcar)

    def fechar_garra(self, forcar: bool = False):
        self.garra.fechar_para_transporte(forcar)

    def soltar_garra(self, forcar: bool = False):
        self.garra.soltar_na_base(forcar)

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
        if novo_estado == EstadoMissao.POSICIONANDO_PARA_COLETA:
            self.abrir_garra(forcar=True)
        self.robo.get_logger().info(
            (
                f'TRANSICAO | {estado_anterior.value} -> {novo_estado.value} | '
                f'motivo={motivo} | '
                f'pose=({self.robo.x:.2f}, {self.robo.y:.2f}, '
                f'yaw={self.robo.yaw:.2f}) | '
                f'{self.resumo_lidar()} | {self.resumo_visao()}'
            )
        )

    def log_estado_periodico(self, mensagem: str, periodo: float = 1.0):
        chave = f'estado_{self.estado_atual.value}'
        self.robo.log_periodico(
            chave,
            f'ESTADO {self.estado_atual.value} | {mensagem}',
            periodo=periodo,
        )

    def resumo_lidar(self):
        robo = self.robo
        frente = self.distancia_frontal_ativa()
        return (
            'lidar('
            f'fr={robo.formatar_distancia(frente)}, '
            f'esq={robo.formatar_distancia(robo.distancia_esquerda)}, '
            f'dir={robo.formatar_distancia(robo.distancia_direita)}, '
            f'desvio={robo.nome_lado_desvio()}'
            ')'
        )

    def resumo_visao(self):
        robo = self.robo
        det = robo.deteccao_bandeira
        if self.bandeira_recente():
            return (
                'visao('
                'bandeira=sim, '
                f'area={det.area_relativa:.3f}, '
                f'erro_haste={det.erro_x_haste:+.2f}'
                ')'
            )

        return 'visao(bandeira=nao)'
