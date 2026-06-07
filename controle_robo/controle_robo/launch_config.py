from pathlib import Path

import yaml

from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration


def aplicar_config_file(context, nomes_configuraveis):
    """Carrega um YAML de missao e preenche LaunchConfigurations ausentes."""
    config_path = context.perform_substitution(LaunchConfiguration('config_file'))
    if not config_path:
        return []

    caminho = Path(config_path).expanduser()
    if not caminho.is_file():
        raise FileNotFoundError(
            f'Arquivo de configuracao nao encontrado: {caminho}'
        )

    with caminho.open('r', encoding='utf-8') as arquivo:
        config = yaml.safe_load(arquivo) or {}

    valores = {}
    valores.update(config.get('launch', {}) or {})
    valores.update(
        (config.get('controle_robo', {}) or {}).get('ros__parameters', {}) or {}
    )

    acoes = []
    for nome in nomes_configuraveis:
        if nome in context.launch_configurations:
            continue
        if nome not in valores:
            continue

        acoes.append(
            SetLaunchConfiguration(
                nome,
                valor_para_launch_config(valores[nome]),
            )
        )

    return acoes


def valor_para_launch_config(valor):
    if isinstance(valor, bool):
        return 'true' if valor else 'false'
    return str(valor)
