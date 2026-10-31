"""O que o robô aspirador entende: intenções, frases e lugares.

Este módulo é a fonte única do vocabulário, usada pelos dois lados da camada
de voz:

- o voice_listener monta a gramática fechada do Vosk com as palavras daqui, o
  que restringe o reconhecedor a este vocabulário e sobe muito o acerto;
- o command_parser usa as mesmas frases para decidir qual intenção foi dita.

Manter os dois lados na mesma lista evita o erro clássico de ensinar uma frase
ao reconhecedor e esquecer de ensinar ao interpretador.

As frases são escritas com acento
---------------------------------
Cada frase existe aqui em duas formas, e as duas importam:

- **com acento**, como está escrita abaixo, é o que vai para a gramática. O
  léxico do modelo português do Vosk é acentuado: pedir "robo", "rápido" sem
  acento ou "relatorio" faz o Vosk responder *Ignoring word missing in
  vocabulary* e a palavra simplesmente não pode ser reconhecida.
- **sem acento e sem artigo**, pelo :func:`tokenize`, é o que se compara com a
  transcrição. O modelo pode devolver "limpa" ou "limpá" conforme o áudio, e
  quem digita no console erra acento; comparar sem acento tira isso do
  caminho.

Casamento por conjunto de palavras
----------------------------------
Uma frase casa quando **todas as palavras dela aparecem** na transcrição, em
qualquer ordem e sem precisar estar seguidas. Dizer "robô, pode limpar a sala
agora" casa com a frase "limpar a sala", porque "limpar" e "sala" estão lá.

A alternativa seria exigir a frase exata, o que quebra com qualquer palavra a
mais ou a menos; ou uma só palavra-chave, que confundiria "limpar a sala" com
"limpar a cozinha". O conjunto de palavras fica no meio: tolera o enfeite da
fala e ainda distingue os comandos.

Quando mais de uma intenção casa, ganha a que casou com **mais palavras**
("limpar a sala" ganha de "limpar"); no empate vale a declarada primeiro, por
isso 'parar' está no topo da lista.
"""

import unicodedata


def spell(text):
    """Palavras de um texto, em minúsculas e sem pontuação, **com** acento.

    É a forma que vai para a gramática do reconhecedor, que precisa das
    palavras escritas como estão no léxico do modelo.

    >>> spell('Robô, limpa o ESCRITÓRIO!')
    ['robô', 'limpa', 'o', 'escritório']
    """
    limpo = ''.join(c if c.isalnum() else ' ' for c in text.lower())
    return limpo.split()


def normalize(text):
    """Deixa o texto comparável: minúsculas, sem acento e sem pontuação.

    >>> normalize('Robô, limpa o ESCRITÓRIO!')
    'robo limpa o escritorio'
    """
    decomposto = unicodedata.normalize('NFKD', text.lower())
    sem_acento = ''.join(c for c in decomposto if not unicodedata.combining(c))
    limpo = ''.join(c if c.isalnum() else ' ' for c in sem_acento)
    return ' '.join(limpo.split())


#: Palavras que não distinguem comando nenhum. Continuam na gramática — a fala
#: tem artigos e preposições, e o reconhecedor precisa de onde encaixá-los —
#: mas saem antes do casamento, para não contarem pontos.
STOPWORDS = frozenset(
    'o a os as um uma de do da no na em pro pra ao e que por favor me nos'.split()
)


def tokenize(text):
    """Palavras úteis de uma transcrição, normalizadas e sem stopword."""
    return [p for p in normalize(text).split() if p not in STOPWORDS]


#: Palavra que abre o comando, quando o wake word está ligado no parser. Vai
#: para a gramática mesmo desligado: o reconhecedor precisa conhecê-la, senão
#: a troca por outra parecida que esteja no vocabulário.
WAKE_WORDS = ('robô', 'robozinho', 'aspirador')

#: Lugares que podem virar argumento de um comando ("limpar a cozinha").
#: Chave: forma normalizada que o parser procura nas palavras da transcrição.
#: Valor: nome acentuado, que vai no comando publicado e na gramática.
PLACES = {
    'sala': 'sala',
    'cozinha': 'cozinha',
    'quarto': 'quarto',
    'banheiro': 'banheiro',
    'corredor': 'corredor',
    'escritorio': 'escritório',
    'garagem': 'garagem',
    'casa': 'casa toda',
}

#: Modificadores do comando de velocidade. Também entram os verbos que já
#: dizem o sentido por si ("acelera", "diminuir a velocidade"), senão a
#: intenção casaria sem argumento nenhum e o comando sairia incompleto.
#: Chaves normalizadas, como as de PLACES.
SPEEDS = {
    'rapido': 'rapido',
    'acelerar': 'rapido',
    'acelera': 'rapido',
    'aumentar': 'rapido',
    'devagar': 'devagar',
    'lento': 'devagar',
    'diminuir': 'devagar',
}

#: Palavras acentuadas que valem a pena estar na gramática mas não aparecem em
#: nenhuma frase: variações que a pessoa pode dizer e que o reconhecedor
#: precisa ter onde encaixar, em vez de forçar numa palavra de comando.
EXTRA_WORDS = (
    'lento', 'rápida', 'agora', 'tudo', 'bem', 'por', 'favor', 'obrigado',
    'sim', 'não', 'aqui', 'ali', 'lá',
)


class Intent:
    """Uma intenção reconhecível: o nome, o que ela quer dizer e como se diz.

    Atributos:
        name: identificador publicado no tópico (ex. ``limpar_local``).
        description: uma linha em português, para o log e o README.
        raw_phrases: as frases como foram escritas, com acento, para a
            gramática do reconhecedor.
        phrases: as mesmas frases tokenizadas (sem acento, sem artigo), que é
            a forma comparada com a transcrição.
        takes_place: a intenção aceita um lugar como argumento.
        takes_speed: a intenção aceita 'rapido' ou 'devagar' como argumento.
    """

    def __init__(self, name, description, phrases,
                 takes_place=False, takes_speed=False):
        self.name = name
        self.description = description
        self.raw_phrases = list(phrases)
        self.phrases = [tuple(tokenize(p)) for p in phrases if tokenize(p)]
        self.takes_place = takes_place
        self.takes_speed = takes_speed

    def match(self, tokens):
        """Palavras da melhor frase que casa, ou 0 se nenhuma casa.

        O número serve de pontuação: a frase mais específica (mais palavras)
        ganha da genérica que também casaria.
        """
        presentes = set(tokens)
        casadas = [len(frase) for frase in self.phrases
                   if all(palavra in presentes for palavra in frase)]
        return max(casadas, default=0)

    def words(self):
        """Palavras normalizadas que esta intenção usa, para o casamento."""
        return {palavra for frase in self.phrases for palavra in frase}

    def grammar_words(self):
        """Palavras acentuadas desta intenção, para a gramática."""
        return {p for frase in self.raw_phrases for p in spell(frase)}


#: As intenções, em ordem de prioridade: no empate de pontuação vale a
#: primeira. 'parar' vem antes de tudo porque é o comando de segurança — dito
#: no meio de outra coisa, é ele que tem de valer.
INTENTS = (
    Intent(
        'parar',
        'parar o robô agora e cancelar o que estava fazendo',
        ['parar', 'pare', 'parar tudo', 'para tudo', 'cancelar', 'cancela',
         'desligar', 'desliga'],
    ),
    Intent(
        'pausar',
        'interromper a tarefa sem cancelar, para continuar depois',
        ['pausar', 'pausa', 'espera', 'esperar'],
    ),
    Intent(
        'continuar',
        'retomar a tarefa que estava pausada',
        ['continuar', 'continua', 'retomar', 'retoma', 'seguir'],
    ),
    Intent(
        'limpar_local',
        'limpar um lugar específico',
        ['limpar a sala', 'limpa a sala', 'aspirar a sala',
         'limpar a cozinha', 'limpa a cozinha', 'aspirar a cozinha',
         'limpar o quarto', 'limpa o quarto', 'aspirar o quarto',
         'limpar o banheiro', 'limpa o banheiro', 'aspirar o banheiro',
         'limpar o corredor', 'limpa o corredor', 'aspirar o corredor',
         'limpar o escritório', 'limpa o escritório', 'aspirar o escritório',
         'limpar a garagem', 'limpa a garagem', 'aspirar a garagem',
         'limpar a casa', 'limpa a casa', 'limpar a casa toda'],
        takes_place=True,
    ),
    Intent(
        'limpar_aqui',
        'limpar só o ponto onde o robô está',
        ['limpar aqui', 'limpa aqui', 'aspirar aqui', 'limpar este ponto'],
    ),
    Intent(
        'limpar_tudo',
        'limpar todo o ambiente conhecido',
        ['limpar tudo', 'limpa tudo', 'aspirar tudo', 'limpar', 'limpa',
         'aspirar', 'começar a limpeza', 'começa a limpeza',
         'iniciar a limpeza', 'faxina'],
    ),
    Intent(
        'mapear',
        'percorrer o ambiente construindo o mapa, sem limpar',
        ['mapear o ambiente', 'mapear a casa', 'mapear', 'mapeia',
         'fazer o mapa', 'faz o mapa', 'explorar', 'explora'],
    ),
    Intent(
        'ir_para',
        'ir até um lugar, sem limpar no caminho',
        ['ir para a sala', 'vai para a sala',
         'ir para a cozinha', 'vai para a cozinha',
         'ir para o quarto', 'vai para o quarto',
         'ir para o banheiro', 'vai para o banheiro',
         'ir para o corredor', 'vai para o corredor',
         'ir para o escritório', 'vai para o escritório',
         'ir para a garagem', 'vai para a garagem'],
        takes_place=True,
    ),
    Intent(
        'voltar_base',
        'voltar para a base de carga',
        ['voltar para a base', 'volta para a base', 'ir para a base',
         'vai para a base', 'voltar para o carregador', 'carregar a bateria',
         'voltar', 'volta', 'base'],
    ),
    Intent(
        'ajustar_velocidade',
        'andar mais rápido ou mais devagar',
        ['mais rápido', 'mais devagar', 'acelerar', 'acelera',
         'diminuir a velocidade', 'aumentar a velocidade',
         'devagar', 'rápido'],
        takes_speed=True,
    ),
    Intent(
        'status',
        'dizer o que está fazendo e quanto já limpou',
        ['status', 'relatório', 'como está', 'o que está fazendo',
         'quanto falta', 'nível da bateria', 'bateria'],
    ),
    Intent(
        'ajuda',
        'listar os comandos que o robô entende',
        ['ajuda', 'socorro', 'quais os comandos', 'o que sabe fazer',
         'lista de comandos'],
    ),
)


def parse(text):
    """Interpreta uma transcrição.

    Devolve ``(intent, args, score)``, onde intent é um :class:`Intent` ou
    ``None`` quando nada casou, args é um dicionário com o lugar e/ou a
    velocidade encontrados, e score é o número de palavras que casaram.

    >>> intent, args, score = parse('robô, limpa a cozinha por favor')
    >>> intent.name, args, score
    ('limpar_local', {'local': 'cozinha'}, 2)
    >>> parse('bom dia')[0] is None
    True
    """
    tokens = tokenize(text)
    if not tokens:
        return None, {}, 0

    melhor, melhor_score = None, 0
    for intent in INTENTS:
        score = intent.match(tokens)
        # Estritamente maior: no empate fica a intenção declarada antes.
        if score > melhor_score:
            melhor, melhor_score = intent, score
    if melhor is None:
        return None, {}, 0

    args = {}
    if melhor.takes_place:
        for palavra in tokens:
            if palavra in PLACES:
                args['local'] = PLACES[palavra]
                break
    if melhor.takes_speed:
        for palavra in tokens:
            if palavra in SPEEDS:
                args['velocidade'] = SPEEDS[palavra]
                break
    return melhor, args, melhor_score


def vocabulary(wake_words=True):
    """Palavras acentuadas do vocabulário, para a gramática do reconhecedor.

    A gramática fechada do Vosk só devolve palavras desta lista, então ela
    precisa conter tudo que vale a pena ouvir: as palavras das frases, os
    lugares, as palavras de ativação, as stopwords e os enfeites de EXTRA_WORDS
    que aparecem na fala natural.
    """
    palavras = set(STOPWORDS)
    palavras |= set(EXTRA_WORDS)
    for intent in INTENTS:
        palavras |= intent.grammar_words()
    for nome in PLACES.values():
        palavras |= set(spell(nome))
    if wake_words:
        palavras |= set(WAKE_WORDS)
    return sorted(palavras)


def help_lines():
    """Uma linha por intenção, com a descrição e um exemplo de como dizer."""
    linhas = []
    for intent in INTENTS:
        exemplo = max(intent.raw_phrases, key=lambda p: len(tokenize(p)))
        linhas.append(f'  {intent.name:<20} {intent.description} '
                      f'(ex.: "{exemplo}")')
    return linhas
