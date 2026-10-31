# voice_cleaner

Robô aspirador que **recebe comandos por voz**. O reconhecimento é offline, em
português, e o comando reconhecido sai num tópico ROS 2. É o trabalho descrito
em [requirements.md](requirements.md).

O robô **transcreve tudo o que for falado** e **age só no que sabe fazer**: o
reconhecedor trabalha com o vocabulário inteiro do português, e é o
interpretador que separa comando de conversa.

```
microfone → Vosk (offline) → texto → intenção + argumentos → voice/command
```

Nesta etapa o robô **ouve e entende**; executar é a etapa seguinte. Ninguém
consome `voice/command` ainda: o navegador herdado do `potential_field`
continua fazendo o que já fazia, lado a lado com a camada de voz.

```
                                                      (nova camada)
   microfone ──▶ voice_listener ──transcript──▶ command_parser ──▶ voice/command
   teclado   ──▶ command_console ─────┘                                  ╎
                                                                        ╎ ainda
                                                      (base herdada)    ╎ ninguém
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ──▶ yolo_vision ──▶ perception_map
     ▲                       ▲                                   │
     │ motores               │ cmd_vel                           ▼
     └───────────────────────┴──────────────────────────────── navigator
```

| Executável | Papel |
|---|---|
| `voice_listener` | **Novo.** Captura o microfone e transcreve com o Vosk; publica o texto |
| `command_parser` | **Novo.** Decide qual intenção foi dita, extrai o argumento e publica o comando |
| `command_console` | **Novo.** Teclado no lugar do microfone, para testar sem falar |
| `coppelia_bridge` | Só comunicação com a cena: imagem, profundidade, pose, TF; aplica o `cmd_vel` |
| `yolo_vision` | Roda o YOLO e projeta cada detecção em 3D no referencial da câmera |
| `perception_map` | Leva as detecções para o mundo e mantém o mapa persistente |
| `navigator` | Campo potencial: escolhe o alvo, soma as forças, publica `cmd_vel` |
| `fake_world` | Substitui a ponte para testar o navegador sem o simulador |

Os cinco últimos vêm do `potential_field` sem mudança de comportamento.

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- `libportaudio2`, a biblioteca de áudio que o `sounddevice` usa. Nem toda
  instalação do Ubuntu 24.04 já a tem; sem ela o `sounddevice` nem importa
  (`OSError: PortAudio library not found`):

```bash
sudo apt-get install -y libportaudio2
```

- No Python do sistema (o `ros2 run` não usa o `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages vosk sounddevice
```

Para a base herdada (só se for usar o simulador e o YOLO):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages \
  coppeliasim-zmqremoteapi-client ultralytics "numpy<2"
```

### O modelo de fala

Na primeira execução o `voice_listener` pede `Model(lang='pt')` e o Vosk baixa
o `vosk-model-small-pt-0.3` (31 MB compactado, 52 MB em disco) para
`~/.cache/vosk/`. Depois disso ele carrega em menos de um segundo e nada mais é
buscado na rede.

Para usar um modelo baixado à mão (de <https://alphacephei.com/vosk/models>),
aponte a pasta:

```bash
ros2 launch voice_cleaner voice.launch.py model_path:=$HOME/vosk-model-small-pt-0.3
```

**O download automático tem um bug no vosk 0.3.45.** Vale conhecê-lo porque o
sintoma não diz o que fazer. O `Model(lang='pt')` procura o modelo varrendo
`~/.cache/vosk/` com a regex `vosk-model(-small)?-pt` e aceita a primeira
entrada que casar (`get_model_by_lang`, em `vosk/__init__.py`). O zip que ele
mesmo acabou de baixar casa tanto quanto a pasta extraída, e quando o
`os.listdir` o entrega primeiro o reconhecedor morre assim:

```
ERROR (VoskAPI:Model():model.cc:122) Folder '~/.cache/vosk/vosk-model-small-pt-0.3.zip'
does not contain model files.
[voice_listener]: não consegui carregar o modelo: Failed to create a model
```

Um download interrompido também deixa a pasta pela metade — faltando
`HCLr.fst`, `mfcc.conf` ou arquivos do `ivector/` —, com o mesmo erro no fim.
Para consertar as duas coisas de uma vez, extraia de novo e apague o zip:

```bash
cd ~/.cache/vosk
unzip -t vosk-model-small-pt-0.3.zip        # confirma que o zip está íntegro
rm -rf vosk-model-small-pt-0.3
unzip -q vosk-model-small-pt-0.3.zip
rm vosk-model-small-pt-0.3.zip              # o zip no cache é que confunde o vosk
```

A pasta correta tem 9 entradas, entre elas `final.mdl`, `Gr.fst`, `HCLr.fst`,
`mfcc.conf` e `ivector/` com 6 arquivos. Passar `model_path` à mão evita a
busca por regex inteira, e é o caminho à prova desse bug.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select voice_cleaner --symlink-install
source install/setup.bash    # no zsh, install/setup.zsh
ros2 pkg executables voice_cleaner
```

O `--symlink-install` faz os `.py` do nó valerem direto da árvore de fontes, sem
recompilar a cada edição — **mas não os launch files**, que são copiados para
`install/`. Mexeu em `launch/`, compile de novo, senão o launch que roda é o
antigo e o sintoma é mudança que "não pegou".

---

## 3. Executar

### Só a voz, sem simulador

É o caminho mais curto para ver a coisa funcionando:

```bash
ros2 launch voice_cleaner voice.launch.py
```

Fale. O robô transcreve tudo o que ouve, e o terminal mostra o que ele fez com
cada frase — comando quando é tarefa dele, nada quando não é:

```
[voice_listener]: vocabulário completo do português: transcrevo tudo o que for dito, ...
[voice_listener]: ouvindo o microfone (dispositivo padrão); fale um comando
[voice_listener]: ouvi: "robô limpa a cozinha" (confiança 0.94)
[command_parser]: comando: limpar_local local=cozinha  <- "robô limpa a cozinha"
[voice_listener]: ouvi: "você viu o jogo ontem" (confiança 0.88)
[command_parser]: ouvido, mas não é tarefa minha: "você viu o jogo ontem"
```

A segunda frase foi transcrita e descartada: isso é o funcionamento normal, não
falha de reconhecimento. `voice/transcript` tem tudo; `voice/command`, só o que
ele sabe executar.

Se o nó sobe e ouve mas o texto não tem nada a ver com o que você disse, o
problema costuma ser a entrada de áudio, não o reconhecedor. Para ver as
entradas:

```bash
ros2 run voice_cleaner voice_listener --ros-args -p list_devices:=true
```

```
   7 USB PnP Sound Device: Audio (hw:1,0), ALSA (1 in, 2 out)
*  11 default, ALSA (64 in, 64 out)
```

A padrão (`*`) é a do servidor de som, que normalmente serve. Com microfone USB
ou headset, apontar o dispositivo direto costuma captar melhor:

```bash
ros2 launch voice_cleaner voice.launch.py device:=7
```

E, falando, o nível tem de passar de −40 dBFS:

```bash
ros2 topic echo /myRobot/voice/level
```

Noutro terminal, o comando como ele sai no tópico:

```bash
ros2 topic echo /myRobot/voice/command
```

```yaml
data: '{"intent": "limpar_local", "args": {"local": "cozinha"},
        "texto": "robo limpa a cozinha", "score": 2, "stamp": 1793285571.711}'
```

### Conferindo que ele ouve tudo e age só no que sabe

Os dois lados do comportamento ficam visíveis lado a lado em três terminais.
No primeiro, o launch; nos outros dois, os tópicos que separam a fala do
comando:

```bash
ros2 topic echo /myRobot/voice/transcript   # tudo o que foi dito
ros2 topic echo /myRobot/voice/intent       # só o que ele vai fazer
```

Agora fale duas frases, uma de cada tipo — por exemplo "robô, limpa a cozinha"
e depois "você viu o jogo ontem". As duas aparecem em `voice/transcript`;
só a primeira aparece em `voice/intent`. É esse o teste: **transcript enche,
intent é seletivo.**

#### Publicando a transcrição à mão

Para conferir a separação sem reconhecimento de fala no meio — útil para isolar
o interpretador de problemas de áudio, e o jeito mais direto de repetir sempre
a mesma frase. Num terminal, só o interpretador:

```bash
ros2 run voice_cleaner command_parser --ros-args -r __ns:=/myRobot
```

Noutro, as frases:

```bash
ros2 topic pub -1 /myRobot/voice/transcript std_msgs/String \
  "{data: 'robô limpa a cozinha'}"
ros2 topic pub -1 /myRobot/voice/transcript std_msgs/String \
  "{data: 'você viu o jogo ontem à noite'}"
ros2 topic pub -1 /myRobot/voice/transcript std_msgs/String \
  "{data: 'preciso lembrar de comprar pão'}"
```

O interpretador responde a cada uma:

```
[command_parser]: comando: limpar_local local=cozinha  <- "robô limpa a cozinha"
[command_parser]: ouvido, mas não é tarefa minha: "você viu o jogo ontem à noite"
[command_parser]: ouvido, mas não é tarefa minha: "preciso lembrar de comprar pão"
```

#### Vendo o falso positivo, e a defesa

Vale conhecer o ponto fraco antes de topar com ele em uso. Como `parar`,
`pausar`, `continuar` e `status` são comandos de uma palavra, conversa que
contenha a palavra casa:

```bash
ros2 topic pub -1 /myRobot/voice/transcript std_msgs/String \
  "{data: 'pode parar de falar nisso'}"
```

```
[command_parser]: comando: parar  <- "pode parar de falar nisso"
```

Suba o interpretador com `min_score:=2` e a mesma frase passa a ser recusada,
porque duas palavras casadas obrigam a dizer "para tudo":

```bash
ros2 run voice_cleaner command_parser --ros-args -r __ns:=/myRobot -p min_score:=2
```

```
[command_parser]: "pode parar de falar nisso" parece parar, mas casou só 1 palavra(s) de 2: ignorado
```

**`min_score:=2` tem um custo que vale medir antes de adotar:** `pausar` e
`continuar` ficam impossíveis de dizer, porque nenhuma forma delas tem duas
palavras ("pausar", "pausa", "espera" — todas valem 1). Os outros dez comandos
sobrevivem, alguns trocando de forma: `parar` exige "para tudo" em vez de
"parar", e `status` exige "como está" ou "o que está fazendo" em vez de
"status". Para conferir você mesmo:

```bash
PYTHONPATH=. /usr/bin/python3 -c "
from voice_cleaner import commands
for it in commands.INTENTS:
    n = max(len(commands.tokenize(p)) for p in it.raw_phrases)
    print(f'{it.name:20s} maior frase = {n} palavra(s)')"
```

Por isso o padrão é `min_score:=1`: ele aceita o vocabulário inteiro, ao preço
dos falsos positivos acima. `wake_word:=robo` é a defesa que não custa comando
nenhum — exige só que você chame o robô pelo nome antes de mandar. A seção 6
discute a escolha.

### Sem microfone: digitando

Troca o microfone pelo teclado, publicando no mesmo tópico. Não precisa do
modelo baixado nem de áudio nenhum:

```bash
ros2 run voice_cleaner command_console --ros-args -r __ns:=/myRobot
```

```
comando> limpar a sala
  -> limpar_local (local=sala), 2 palavra(s)
comando> bom dia
  -> nenhuma intenção casou (o parser vai ignorar)
comando> sair
```

A cadeia inteira fica de pé com o `command_parser` noutro terminal:

```bash
ros2 run voice_cleaner command_parser --ros-args -r __ns:=/myRobot
```

Pelo launch, `console:=1` faz a mesma troca e sobe os dois de uma vez. Nesse
caso o prompt divide o terminal com os logs do launch, que passam por cima do
que você digita: dá para usar, mas digitar fica mais confortável com os dois
`ros2 run` em terminais separados.

### Com o robô

```bash
# Ponte + navegador por campos potenciais + voz
ros2 launch voice_cleaner voice_cleaner.launch.py

# Idem, com o mapa construído pela câmera (YOLO na Kinect)
ros2 launch voice_cleaner perception_voice.launch.py

# Só a voz, robô parado
ros2 launch voice_cleaner voice_cleaner.launch.py navigator:=0

# Sem o simulador: mundo de mentira para o navegador, voz de verdade
ros2 launch voice_cleaner voice_cleaner.launch.py fake:=1
```

Os launches com simulador pedem o CoppeliaSim aberto com a cena
[coppeliasim/robot/pega_banana_potential_field.ttt](coppeliasim/robot/pega_banana_potential_field.ttt),
**com interface gráfica** (a Kinect precisa de OpenGL). O resto — scripts da
cena, critério de parada, argumentos da ponte — é igual ao `potential_field`,
documentado no README daquele pacote.

### Argumentos do launch de voz

Valem para `voice.launch.py` e, repassados, para os outros dois.

| Argumento | Padrão | O que faz |
|---|---|---|
| `console` | `0` | `1` troca o microfone pelo teclado |
| `model_path` | `''` | Pasta do modelo Vosk; vazio baixa o modelo do idioma |
| `lang` | `pt` | Idioma do modelo baixado |
| `device` | `''` | Entrada de áudio (índice ou parte do nome); vazio usa a padrão |
| `grammar` | `false` | Padrão transcreve toda a fala; `true` restringe o reconhecedor aos comandos |
| `min_confidence` | `0.0` | Descarta transcrição com confiança média abaixo disto |
| `wake_word` | `''` | Palavra de ativação; vazio aceita qualquer frase |
| `min_score` | `1` | Palavras que a frase precisa casar para virar comando |
| `namespace` | `myRobot` | Namespace dos tópicos |

---

## 4. Os comandos

| Intenção | Argumento | Como dizer |
|---|---|---|
| `parar` | — | "parar", "pare", "para tudo", "cancela", "desliga" |
| `pausar` | — | "pausar", "pausa", "espera" |
| `continuar` | — | "continuar", "continua", "retoma", "seguir" |
| `limpar_local` | `local` | "limpar a sala", "limpa a cozinha", "aspirar o quarto" |
| `limpar_aqui` | — | "limpar aqui", "limpar este ponto" |
| `limpar_tudo` | — | "limpar tudo", "começar a limpeza", "faxina" |
| `mapear` | — | "mapear o ambiente", "fazer o mapa", "explorar" |
| `ir_para` | `local` | "ir para a cozinha", "vai para o quarto" |
| `voltar_base` | — | "voltar para a base", "carregar a bateria", "volta" |
| `ajustar_velocidade` | `velocidade` | "mais rápido", "mais devagar", "acelera" |
| `status` | — | "status", "relatório", "quanto falta", "nível da bateria" |
| `ajuda` | — | "ajuda", "quais os comandos", "o que sabe fazer" |

Lugares aceitos em `local`: sala, cozinha, quarto, banheiro, corredor,
escritório, garagem, casa (vira "casa toda"). Em `velocidade`: `rapido` ou
`devagar`.

A lista completa, com todas as formas de dizer cada comando, está em
[voice_cleaner/commands.py](voice_cleaner/commands.py) — é o único lugar a
mexer para acrescentar comando, e tanto o reconhecedor como o interpretador
saem de lá. O `command_parser` imprime a lista quando sobe.

---

## 5. Tópicos

Todos no namespace `myRobot`, aplicado pelo launch. Só os três primeiros são
novos; os outros vêm do `potential_field`.

| Tópico | Tipo | Quem publica | Conteúdo |
|---|---|---|---|
| `voice/transcript` | `std_msgs/String` | `voice_listener` ou `command_console` | Toda frase reconhecida (ou digitada), comando ou não |
| `voice/partial` | `std_msgs/String` | `voice_listener` | Hipótese do meio da frase; não vira comando |
| `voice/level` | `std_msgs/Float32` | `voice_listener` | Nível do áudio em dBFS, 2 Hz; serve para ver se o microfone capta |
| `voice/command` | `std_msgs/String` | `command_parser` | O comando, em JSON (ver abaixo) |
| `voice/intent` | `std_msgs/String` | `command_parser` | Só o nome da intenção, para `ros2 topic echo` rápido |
| `rgb/image`, `depth/image`, `rgb/camera_info` | `sensor_msgs/*` | ponte | Kinect da cena |
| `pose`, `/tf` | `PoseStamped`, `TFMessage` | ponte | Pose do robô e referenciais |
| `detections/bananas`, `detections/poops` | `PoseArray` | `yolo_vision` | Detecções do quadro |
| `bananas`, `poops` | `PoseArray` | `perception_map` | Mapa persistente (transient local) |
| `cmd_vel` | `Twist` | `navigator` | `linear.x` (m/s), `angular.z` (rad/s) |

### O formato do comando

```json
{"intent": "limpar_local", "args": {"local": "cozinha"},
 "texto": "robo limpa a cozinha", "score": 2, "stamp": 1793285571.711}
```

É JSON dentro de `std_msgs/String`, e não uma mensagem própria, porque o pacote
é `ament_python`: um `.msg` exigiria passá-lo para `ament_cmake`. Para quem for
executar o comando depois, um `json.loads` resolve.

---

## 6. Como funciona

### Reconhecimento (`voice_listener`)

O Vosk roda na própria máquina: não depende de internet, não manda áudio para
fora e tem latência previsível, que é o que importa num robô. O modelo pequeno
de português carrega em menos de um segundo e divide a CPU com o simulador sem
briga.

**Ele transcreve tudo, e só obedece ao que sabe fazer.** O reconhecedor usa o
vocabulário inteiro do português, então `voice/transcript` traz toda fala que
chega ao microfone, como foi dita — comando ou conversa. A seleção é do
`command_parser`, que publica em `voice/command` só as intenções de
`commands.py` e deixa o resto no log. São duas responsabilidades separadas de
propósito: o reconhecedor é um ouvido, não um filtro, e ter o registro fiel da
fala distingue "ouviu errado" de "não é tarefa dele", que é a dúvida mais comum
ao depurar.

**A gramática fechada** (`grammar:=true`) é a alternativa. O Vosk aceita
receber, em vez do vocabulário do idioma, a lista de palavras que podem
aparecer; para esses comandos ela tem 108 palavras, e o reconhecedor passa a
escolher sempre dentro dela: "limpa a cozinha" deixa de virar "limpa a buzina".
O preço é que tudo fora da lista sai como `[unk]` ou como a palavra mais
parecida, e o robô deixa de registrar o que foi dito de fato. Vale em ambiente
barulhento, onde só os comandos importam, ou em CPU apertada — medido aqui com
o modelo pequeno de português, a gramática decodifica a 0,07 do tempo real
contra 0,17 do vocabulário completo (7% contra 17% de um núcleo).

O lado ruim do vocabulário aberto é que a conversa da sala também é transcrita,
e uma frase qualquer pode casar com um comando por acidente — "pode parar de
falar nisso" contém "parar". Duas defesas existem, com custos diferentes:
`min_score:=2` exige frase mais completa, mas deixa `pausar` e `continuar`
impossíveis de dizer (nenhuma forma delas tem duas palavras); `wake_word:=robo`
exige o nome do robô antes e não custa comando nenhum. Os dois vêm desligados,
e a seção 3 tem como experimentar cada um.

**As palavras da gramática vão com acento** — vale quando ela está ligada, e a
invariante é testada de qualquer jeito. O léxico do modelo português é
acentuado: pedir `robo`, `rapido` ou `relatorio` faz o Vosk responder *Ignoring
word missing in vocabulary* e a palavra fica impossível de reconhecer. Por
isso as frases em `commands.py` são escritas com acento, e é a comparação com a
transcrição que acontece sem acento.

**As duas threads:** o PortAudio entrega o áudio numa thread dele, que não pode
bloquear nem chamar o ROS; ela só empilha os blocos numa fila. Um timer do nó
esvazia a fila e alimenta o reconhecedor, na thread do ROS, sem lock nenhum
escrito à mão.

### Interpretação (`command_parser`)

A transcrição é normalizada (minúsculas, sem acento, sem pontuação) e dividida
em palavras, descartando artigos e preposições, que não distinguem comando
nenhum.

Uma frase do vocabulário casa quando **todas as palavras dela aparecem** na
transcrição, em qualquer ordem e sem precisar estar seguidas. "Robô, pode
limpar a sala agora" casa com "limpar a sala". Exigir a frase exata quebraria
com qualquer palavra a mais; uma só palavra-chave confundiria "limpar a sala"
com "limpar a cozinha". O conjunto de palavras tolera o enfeite da fala e ainda
distingue os comandos.

Quando mais de uma intenção casa, ganha a que casou com **mais palavras**:
"limpar a sala" (2) ganha de "limpar" (1), então o comando sai como
`limpar_local`, e não `limpar_tudo`. No empate vale a declarada primeiro, e
`parar` é a primeira de todas — dito no meio de outra coisa, é ele que tem de
valer.

Depois, se a intenção aceita argumento, a primeira palavra da transcrição que
for um lugar (ou um modificador de velocidade) vira o argumento.

**Repetição:** o mesmo comando outra vez dentro de `repeat_window` (1,5 s) é
descartado. O Vosk às vezes fecha a mesma frase duas vezes, e um comando
duplicado faria o robô reiniciar a tarefa que acabou de começar.

**Palavra de ativação:** com `wake_word` vazio (o padrão), qualquer frase
reconhecida vale. Com `wake_word:=robo`, só vale o que vier depois da palavra —
na mesma frase ("robô, limpa a sala") ou numa frase anterior, dentro de
`wake_timeout` segundos. Serve para o robô não obedecer à conversa da sala, ao
custo de exigir uma palavra que o reconhecedor também pode errar.

---

## 7. Parâmetros

### `voice_listener`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `model_path` | `''` | Pasta do modelo; vazio pede o modelo do idioma ao Vosk |
| `lang` | `pt` | Idioma, quando `model_path` está vazio |
| `sample_rate` | `16000` | O Vosk trabalha a 16 kHz; o servidor de som reamostra |
| `grammar` | `false` | `true` restringe o reconhecedor ao vocabulário dos comandos |
| `min_confidence` | `0.0` | Confiança média mínima para publicar a transcrição |
| `publish_partial` | `true` | Publicar as hipóteses do meio da frase |
| `list_devices` | `false` | Lista as entradas de áudio e sai |
| `device` | `''` | Entrada de áudio: índice (`3`) ou parte do nome (`ALC236`) |
| `block_size` | `4000` | Amostras por bloco (0,25 s a 16 kHz) |
| `level_rate` | `2.0` | Publicação do nível de áudio (Hz) |
| `silent_warn` | `10.0` | Sem áudio por este tempo, avisa (s) |

### `command_parser`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `wake_word` | `''` | Palavra de ativação; vazio aceita qualquer frase |
| `wake_timeout` | `8.0` | Quanto tempo a ativação continua valendo (s) |
| `min_score` | `1` | Palavras que a frase precisa casar. `2` erra menos com conversa, mas torna `pausar` e `continuar` impossíveis de dizer |
| `repeat_window` | `1.5` | Mesmo comando dentro disto é ignorado (s) |
| `print_help` | `true` | Imprimir a lista de comandos ao subir |

### `command_console`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `prompt` | `comando> ` | Texto do prompt |
| `preview` | `true` | Mostrar no console que intenção a frase casou |

Os parâmetros da ponte, do navegador, do `yolo_vision` e do `perception_map`
são os mesmos do `potential_field`.

---

## 8. Testes

O vocabulário tem teste, e ele roda sem ROS, sem microfone e sem modelo:

```bash
cd ros2/ros2_ws/src/voice_cleaner
PYTHONPATH=. /usr/bin/python3 -m pytest test/test_commands.py -q
```

Cobre o que o robô deve entender (frase com enfeite em volta, sem acento, em
maiúsculas), o que ele **não** deve entender (conversa, inclusive falando do
lugar sem mandar limpá-lo: "a cozinha é o lugar mais quente da casa"), a
extração do argumento, a prioridade entre frase específica e genérica, e duas
invariantes que já quebraram: toda palavra das frases estar na gramática, e as
palavras da gramática saírem acentuadas.

Isto é o vocabulário sozinho; para conferir a cadeia de pé — transcrição de
tudo, comando só do que ele sabe fazer — o roteiro está na seção 3.

Um teste cuida do ponto fraco do vocabulário aberto. Conversa que contém a
palavra do comando casa, e não há como evitar: `parar`, `pausar`, `continuar` e
`status` são intenções de uma palavra, então "pode parar de falar nisso" casa
com `parar`. O teste garante que o casamento continue valendo **1 palavra só** —
é o que mantém `min_score:=2` capaz de filtrá-las. Se algum dia uma dessas
frases subir para 2, a defesa cai calada, e o teste quebra antes.

---

## 9. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `ModuleNotFoundError: vosk` ou `sounddevice` | Instale como na seção 1, no Python do sistema. O `ros2 run` não vê o `.venv/`, então instalar lá não resolve |
| `OSError: PortAudio library not found` | Falta a biblioteca do sistema: `sudo apt-get install -y libportaudio2` |
| `Folder '...vosk-model-small-pt-0.3.zip' does not contain model files` | O zip sobrou no cache e o vosk o escolheu em vez da pasta. Refaça a extração como na seção 1 |
| `não consegui carregar o modelo` | Sem rede na primeira execução: baixe o modelo à mão e passe `model_path`. Com o modelo já em `~/.cache/vosk`, é a extração incompleta ou o zip sobrando — seção 1 |
| `nenhum bloco de áudio chegou ainda` | Entrada errada ou muda: veja `-p list_devices:=true` e passe `device:=<índice>`. Num notebook com microfone USB, a entrada padrão não é sempre a que capta |
| O nó ouve, mas o texto não tem nada a ver | Confira o nível em `voice/level`: acima de −40 dBFS enquanto você fala. Microfone longe ou ganho baixo é a causa mais comum |
| Transcreve certo, mas o comando não sai | Veja o log do `command_parser`: `ouvido, mas não é tarefa minha` quer dizer que a frase não casou com intenção nenhuma. Confira a forma de dizer na seção 4, ou acrescente a sua em `commands.py` |
| Erra justamente a palavra do comando ("cozinha" sai "buzina") | É o custo do vocabulário aberto. `grammar:=true` prende o reconhecedor às 108 palavras dos comandos, ao preço de não transcrever mais o resto da fala |
| `Ignoring word missing in vocabulary: 'xxx'` | Palavra nova em `commands.py` sem acento, ou que não existe no léxico do modelo. Escreva com acento; se continuar, troque por um sinônimo |
| Reconhece comando quando ninguém falou com ele | Esperado com o vocabulário aberto, que transcreve a conversa toda. `wake_word:=robo` resolve sem custo; `min_score:=2` também filtra, mas perde `pausar` e `continuar` (seção 3) |
| O mesmo comando chega duas vezes | Aumente `repeat_window` |
| Quer testar a interpretação sem falar | `command_console`, ou `console:=1` no launch |
| `Package 'voice_cleaner' not found` | Faltou sourcear o workspace: `source install/setup.bash` (ou `.zsh`). Sourcear o arquivo da outra shell dá `bad substitution` e não registra o pacote |
| Problema da ponte, do YOLO ou do navegador | É a base herdada: veja a seção 8 do README do `potential_field` |
