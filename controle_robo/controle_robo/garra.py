"""Comandos da garra da missao.

O JointGroupPositionController recebe sempre tres posicoes, na ordem definida
em ``robo_movel/config/controller_config.yaml``:

``[gripper_extension, right_gripper_joint, left_gripper_joint]``.

A maquina de estados nao precisa conhecer esses detalhes; ela so pede para
abrir, fechar para transporte ou soltar na base.
"""


class ControleGarra:
    def __init__(self, robo):
        self.robo = robo
        self.aberta = False
        self.fechada = False

    def abrir_para_coleta(self, forcar: bool = False):
        if not self.robo.habilitar_garra or self.fechada:
            return
        if self.aberta and not forcar:
            return

        self._publicar(self.robo.comando_garra_aberta)
        ja_estava_aberta = self.aberta
        self.aberta = True
        if not ja_estava_aberta:
            self.robo.get_logger().info(
                'GARRA | acao=abrir_para_coleta | '
                f'cmd={self.robo.comando_garra_aberta}'
            )

    def fechar_para_transporte(self, forcar: bool = False):
        if not self.robo.habilitar_garra:
            return
        if self.fechada and not forcar:
            return

        self._publicar(self.robo.comando_garra_captura)
        ja_estava_fechada = self.fechada
        self.aberta = False
        self.fechada = True
        if not ja_estava_fechada:
            self.robo.get_logger().info(
                'GARRA | acao=fechar_e_levantar | '
                f'cmd={self.robo.comando_garra_captura}'
            )

    def recolher_sem_captura(self, forcar: bool = False):
        """Fecha a garra, mas permite abrir novamente depois.

        Usamos isso quando o robo estava se posicionando, perdeu a visao da
        bandeira e vai voltar a procurar/replanejar. A garra nao deve seguir
        aberta pela arena, mas tambem nao podemos marcar a bandeira como
        capturada.
        """

        if not self.robo.habilitar_garra:
            return
        if not self.aberta and not forcar:
            return

        self._publicar(self.robo.comando_garra_recolhida)
        self.aberta = False
        self.fechada = False
        self.robo.get_logger().info(
            'GARRA | acao=recolher_sem_captura | '
            f'cmd={self.robo.comando_garra_recolhida}'
        )

    def soltar_na_base(self, forcar: bool = False):
        if not self.robo.habilitar_garra:
            return
        if self.aberta and not forcar:
            return

        ja_estava_aberta = self.aberta and not self.fechada
        self._publicar(self.robo.comando_garra_aberta)
        self.aberta = True
        self.fechada = False
        if not ja_estava_aberta:
            self.robo.get_logger().info(
                'GARRA | acao=soltar_na_base | '
                f'cmd={self.robo.comando_garra_aberta}'
            )

    def _publicar(self, comando):
        self.robo.publicar_garra(comando)
