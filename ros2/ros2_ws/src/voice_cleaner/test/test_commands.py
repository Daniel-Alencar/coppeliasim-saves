"""Testes do vocabulário: o que o robô deve e não deve entender.

Rodam sem ROS, sem microfone e sem modelo baixado:

    cd ros2/ros2_ws/src/voice_cleaner && /usr/bin/python3 -m pytest test/test_commands.py
"""

import pytest

from voice_cleaner import commands


@pytest.mark.parametrize('frase, intencao', [
    # Como a pessoa fala de verdade, com enfeite em volta do comando.
    ('robô, limpa a sala por favor', 'limpar_local'),
    ('pode limpar tudo agora', 'limpar_tudo'),
    ('aspirador, mapear o ambiente', 'mapear'),
    ('volta para a base', 'voltar_base'),
    ('vai para o quarto', 'ir_para'),
    ('limpar aqui', 'limpar_aqui'),
    ('mais devagar', 'ajustar_velocidade'),
    ('qual o nível da bateria', 'status'),
    ('pare', 'parar'),
    ('cancela', 'parar'),
    ('pausa', 'pausar'),
    ('continua', 'continuar'),
    ('ajuda', 'ajuda'),
    # Sem acento e em maiúsculas, como o reconhecedor às vezes devolve.
    ('LIMPA O ESCRITORIO', 'limpar_local'),
])
def test_intencao_reconhecida(frase, intencao):
    intent, _, _ = commands.parse(frase)
    assert intent is not None, f'nada casou com {frase!r}'
    assert intent.name == intencao


@pytest.mark.parametrize('frase, local', [
    ('limpar a cozinha', 'cozinha'),
    ('aspirar o banheiro', 'banheiro'),
    ('limpa a casa toda', 'casa toda'),
    ('vai para a garagem', 'garagem'),
])
def test_local_extraido(frase, local):
    intent, args, _ = commands.parse(frase)
    assert intent.takes_place
    assert args.get('local') == local


@pytest.mark.parametrize('frase, velocidade', [
    ('mais rápido', 'rapido'),
    ('acelera', 'rapido'),
    ('mais devagar', 'devagar'),
    ('diminuir a velocidade', 'devagar'),
])
def test_velocidade_extraida(frase, velocidade):
    intent, args, _ = commands.parse(frase)
    assert args.get('velocidade') == velocidade


@pytest.mark.parametrize('frase', [
    '',
    '   ',
    'bom dia tudo bem',
    'o jantar está pronto',
    # Com o vocabulário aberto o robô transcreve a conversa da sala, então
    # estas chegam ao interpretador de verdade. Falar do lugar não é mandar
    # limpar o lugar: é a distinção que o casamento por conjunto mantém.
    'a sala está muito bonita hoje',
    'a cozinha é o lugar mais quente da casa',
    'o quarto do meu filho é ali',
    'tenho que ir para o trabalho amanhã',
    'você viu o jogo ontem à noite',
])
def test_conversa_nao_vira_comando(frase):
    intent, _, score = commands.parse(frase)
    assert intent is None
    assert score == 0


@pytest.mark.parametrize('frase, intencao', [
    ('pode parar de falar nisso', 'parar'),
    ('espera um pouco que eu já vou', 'pausar'),
    ('vamos continuar a conversa depois', 'continuar'),
    ('qual o status do projeto', 'status'),
])
def test_conversa_com_palavra_de_comando_casa_fraco(frase, intencao):
    """Conversa que contém a palavra do comando casa — com score 1.

    As intenções de uma palavra (parar, pausar, continuar, status) não têm como
    exigir mais do que a própria palavra, então uma frase de conversa que a
    contenha casa inevitavelmente. Não é bug do casamento: é o motivo de o
    command_parser ter ``min_score`` e ``wake_word``. O que o teste garante é
    que o casamento continue *fraco*, de uma palavra só — se algum dia uma
    destas subir para 2, ``min_score:=2`` deixa de ser defesa e o robô passa a
    obedecer à conversa.
    """
    intent, _, score = commands.parse(frase)
    assert intent.name == intencao
    assert score == 1


def test_frase_especifica_ganha_da_generica():
    """Dizer o lugar não pode cair no comando de limpar tudo."""
    intent, args, score = commands.parse('limpar a sala')
    assert intent.name == 'limpar_local'
    assert args['local'] == 'sala'
    # A genérica também casaria, com uma palavra só.
    assert score == 2


def test_parar_ganha_no_empate():
    """Dito junto com outra coisa de uma palavra, parar tem prioridade."""
    intent, _, _ = commands.parse('pare limpa')
    assert intent.name == 'parar'


def test_vocabulario_cobre_as_frases():
    """Toda palavra das frases está na gramática do reconhecedor.

    Se faltar, o Vosk com gramática fechada nunca devolve aquela palavra e o
    comando correspondente fica impossível de dizer. A gramática é acentuada e
    o casamento não, por isso a comparação é feita na forma normalizada.
    """
    vocabulario = {commands.normalize(p) for p in commands.vocabulary()}
    for intent in commands.INTENTS:
        faltando = intent.words() - vocabulario
        assert not faltando, f'{intent.name}: fora da gramática {faltando}'


def test_gramatica_e_acentuada():
    """As palavras que têm acento em português saem acentuadas na gramática.

    É o que o léxico do modelo Vosk espera; sem acento ele descarta a palavra.
    """
    vocabulario = set(commands.vocabulary())
    for palavra in ('robô', 'rápido', 'relatório', 'escritório', 'nível',
                    'começar', 'está'):
        assert palavra in vocabulario, f'{palavra} saiu sem acento'


def test_toda_intencao_tem_frase():
    """Frase feita só de stopwords desapareceria no tokenize."""
    for intent in commands.INTENTS:
        assert intent.phrases, f'{intent.name} ficou sem frase nenhuma'


def test_normalize_tira_acento_e_pontuacao():
    assert commands.normalize('Robô, limpa o ESCRITÓRIO!') == 'robo limpa o escritorio'
