# voice_cleaner

Robô aspirador que **obedece a comandos de voz**. Você diz "vá para a cozinha"
e ele vai: o reconhecimento é offline, em português, e o robô anda no
CoppeliaSim enquanto o rviz mostra a planta, o caminho e onde ele está.

```
microfone → Vosk → "vá para a cozinha" → A* na planta → waypoints → CoppeliaSim
```

O robô sabe o tempo todo **onde está** (a posição vem da própria simulação, a
20 Hz) e **como a casa é** (a planta e as paredes são conhecidas de antemão,
não descobertas por sensor). Navegar, então, é só achar o caminho: o A* roda
sobre a planta com as paredes infladas pelo raio do robô.

A casa tem sete cômodos e um corredor no meio. Nenhum cômodo dá direto no
outro, então todo trajeto passa pelo corredor:

```
      y
    +2.5 ┌──────────┬────────┬───────────┐
         │  quarto  │ banh.  │  cozinha  │
    +0.55├───┐  ┌───┴─┐  ┌───┴──┐  ┌─────┤   <- portas de 0,90 m
         │            corredor           │
    −0.55├─────┐  ┌────┐  ┌───────┐  ┌───┤
         │escritório│  sala  │ garagem   │
    −2.5 └───────────┴────────┴──────────┘
        −2.5                            +2.5  x
```

---

## 1. Instalar

ROS 2 Jazzy (`/opt/ros/jazzy`) e CoppeliaSim 4.x já instalados. Falta o áudio e
o reconhecedor, **no Python do sistema** — o `ros2 run` não enxerga o `.venv/`:

```bash
sudo apt-get install -y libportaudio2
/usr/bin/python3 -m pip install --user --break-system-packages \
  vosk sounddevice coppeliasim-zmqremoteapi-client
```

### O modelo de fala

Na primeira execução o Vosk baixa o `vosk-model-small-pt-0.3` (31 MB) para
`~/.cache/vosk/`. Depois disso ele carrega em menos de um segundo e nada mais é
buscado na rede.

**O download automático tem um bug no vosk 0.3.45**, e o sintoma não diz o que
fazer:

```
ERROR (VoskAPI:Model():model.cc:122) Folder '~/.cache/vosk/vosk-model-small-pt-0.3.zip'
does not contain model files.
```

O vosk procura o modelo varrendo `~/.cache/vosk/` com uma regex, e o zip que
ele mesmo baixou casa tanto quanto a pasta extraída. Um download interrompido
também deixa a pasta pela metade. Para consertar os dois casos:

```bash
cd ~/.cache/vosk
unzip -t vosk-model-small-pt-0.3.zip        # confirma que o zip está íntegro
rm -rf vosk-model-small-pt-0.3
unzip -q vosk-model-small-pt-0.3.zip
rm vosk-model-small-pt-0.3.zip              # o zip no cache é que confunde o vosk
```

A pasta correta tem 9 entradas, entre elas `final.mdl`, `Gr.fst`, `HCLr.fst`,
`mfcc.conf` e `ivector/` com 6 arquivos.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select voice_cleaner --symlink-install
source install/setup.bash    # no zsh, install/setup.zsh
```

O `--symlink-install` faz os `.py` valerem direto da árvore de fontes, **mas
não os launch files**, que são copiados. Mexeu em `launch/`, compile de novo.

---

## 3. Construir a casa na cena

Uma vez por cena. Abra o CoppeliaSim — uma **cena nova e vazia serve**, basta
ter o chão — e, com a simulação **parada**:

```bash
ros2 run voice_cleaner house_builder
```

```
[house_builder]: 16 trechos de parede criados sob /House
[house_builder]: robô carregado de .../coppeliasim/robot/myRobot.ttm
[house_builder]: /myRobot posto na base (+0.05, -1.52), no meio da sala
[house_builder]: scripts desabilitados (escreviam nos motores e anulariam o
                 cmd_vel): /myRobot/controler, /myRobot/python_controler
```

Ele faz três coisas: cria as paredes, **põe o robô** (carrega o modelo que vem
no pacote, se a cena não tiver um) e desliga os scripts do robô que escreveriam
nos motores — esses, ligados, sobrescrevem o `cmd_vel` e o robô não obedece.

Rodar de novo apaga o `/House` anterior e reconstrói, reaproveitando o robô que
já está lá.

---

## 4. Executar

**Dê play no CoppeliaSim**, e então:

```bash
ros2 launch voice_cleaner house.launch.py
```

O rviz abre com a planta, os nomes dos cômodos e o robô. O microfone já está
ouvindo:

```
[voice_listener]:   ouvindo o microfone (dispositivo padrão); fale um comando
[house_map]:        o robô está no(a) sala
```

Fale. Por exemplo, **"vá para a cozinha"**:

```
[voice_listener]:   ouvi: "vá para a cozinha" (confiança 0.94)
[command_parser]:   comando: ir_para local=cozinha  <- "vá para a cozinha"
[house_navigator]:  indo para o(a) cozinha
[house_navigator]:  caminho até o(a) cozinha: 69 células, 11 waypoints, 3.88 m
[house_map]:        sala -> corredor
[house_map]:        corredor -> cozinha
[house_navigator]:  cheguei no(a) cozinha
```

O robô atravessa o corredor e para no meio da cozinha. No rviz o caminho
aparece em verde, e a seta vermelha é o robô.

### Se o microfone não pegar

A causa quase sempre é a entrada de áudio, não o reconhecedor. Para ver as
entradas:

```bash
ros2 run voice_cleaner voice_listener --ros-args -p list_devices:=true
```

```
   7 USB PnP Sound Device: Audio (hw:1,0), ALSA (1 in, 2 out)
*  11 default, ALSA (64 in, 64 out)
```

A padrão (`*`) normalmente serve. Com microfone USB ou headset, apontar o
dispositivo direto costuma captar melhor:

```bash
ros2 launch voice_cleaner house.launch.py device:=7
```

E, falando, o nível tem de passar de −40 dBFS:

```bash
ros2 topic echo /myRobot/voice/level
```

---

## 5. O que dizer

| Comando | O que o robô faz |
|---|---|
| "vá para a cozinha" | vai até o cômodo |
| "limpar a cozinha" | vai até lá e anuncia (limpar de fato é a etapa seguinte) |
| "voltar para a base" | vai até a base, no meio da sala |
| "limpar tudo", "mapear o ambiente" | percorre os sete cômodos, um a um |
| "pausar" / "continuar" | para sem perder o caminho, e retoma de onde estava |
| "parar" | cancela e para onde está |
| "como está" | diz onde está, para onde vai e quanto falta |

Os cômodos: **sala, cozinha, quarto, banheiro, corredor, escritório,
garagem**.

Cada comando aceita várias formas de dizer — "vá para", "vai para", "ir para";
"limpa", "limpar", "aspirar". A lista completa está em
[voice_cleaner/commands.py](voice_cleaner/commands.py), que é o único lugar a
mexer para acrescentar um jeito de falar.

O robô **transcreve tudo** o que ouve, mas só age no que é tarefa dele. Falar
de outra coisa perto dele aparece no log e não vira comando:

```
[command_parser]: ouvido, mas não é tarefa minha: "você viu o jogo ontem"
```

### Onde ele acha que está

```bash
ros2 topic echo /myRobot/house/room     # o cômodo onde ele está
ros2 topic echo /myRobot/house/task     # "indo para o(a) cozinha" / "parado"
```

Os dois mostram o valor atual na hora, sem esperar a próxima mudança.

---

## 6. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `ModuleNotFoundError: vosk` ou `sounddevice` | Instale no Python do sistema, como na seção 1. O `ros2 run` não vê o `.venv/` |
| `OSError: PortAudio library not found` | `sudo apt-get install -y libportaudio2` |
| `Folder '...-pt-0.3.zip' does not contain model files` | O zip sobrou no cache. Refaça a extração como na seção 1 |
| `Package 'voice_cleaner' not found` | Faltou `source install/setup.bash`. Sourcear o arquivo da outra shell dá `bad substitution` |
| `nenhum bloco de áudio chegou ainda` | Entrada de áudio errada ou muda: veja `list_devices` e passe `device:=<índice>` |
| Ouve, mas o texto não tem nada a ver | Confira o nível em `voice/level`: acima de −40 dBFS enquanto você fala |
| Transcreve certo, mas nada acontece | O log do `command_parser` diz `ouvido, mas não é tarefa minha`: a frase não casou. Veja as formas de dizer na seção 5 |
| O robô não anda, mas o caminho aparece no rviz | Algum script está escrevendo nos motores. O `house_builder` desliga os do robô e diz quais; se o seu tem outro nome, desabilite na árvore da cena |
| `sem pose há 2.1 s` | A simulação não está em play, ou a ponte não conectou |
| `a casa ficou sem robô` | A cena não tinha robô e o modelo não foi achado. Recompile para instalá-lo |
| `sem caminho até o(a) cozinha` | A inflação fechou a porta. Baixe `safety_margin`, ou suba com `show_map:=true` para ver a grade |
| O launch mexeu na cena errada | Há mais de um CoppeliaSim aberto. Passe `port:=` ou feche o que não é da demonstração |
| Reconhece comando quando ninguém falou com ele | Suba com `wake_word:=robo`: aí só vale o que vier depois do nome |

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `device` | `''` | Entrada de áudio (índice ou parte do nome) |
| `wake_word` | `''` | Exige o nome do robô antes do comando |
| `port` | `23000` | Porta do CoppeliaSim; mude se houver mais de um aberto |
| `rviz` | `1` | `0` não abre o rviz |
| `safety_margin` | `0.06` | Folga somada ao raio do robô na inflação das paredes |
| `show_map` | `0` | `1` desenha a grade e o caminho no terminal a cada plano |

---

## 7. Como é feito

Seis nós, cada um com um papel:

| Nó | Papel |
|---|---|
| `voice_listener` | Microfone → texto, com o Vosk offline |
| `command_parser` | Texto → comando (intenção + lugar) |
| `house_map` | A planta: publica o mapa e em que cômodo o robô está |
| `house_navigator` | Comando → A* no mapa → waypoints |
| `position_control` | Waypoint → `cmd_vel` |
| `coppelia_bridge` | `cmd_vel` → motores da cena, e a pose de volta |

A planta mora em [voice_cleaner/house.py](voice_cleaner/house.py), e é **fonte
única**: dela saem o mapa do rviz, as paredes criadas no CoppeliaSim, a
resposta de "em que cômodo estou" e os destinos. Se viessem de lugares
diferentes, o robô planejaria num mapa e bateria noutro.

O caminho é A*, e não campo potencial, porque campo potencial empaca
justamente em porta: as duas paredes do vão empurram o robô para o lado oposto
ao alvo, e o mínimo local fica na frente da porta. O A* não tem mínimo local —
se existe caminho, ele acha.

A inflação é o que torna o plano executável: o A* move o *centro* do robô de
célula em célula, e sem inflar a parede pelo raio dele o caminho passaria a
1 cm do batente. Com raio 0,16 m e folga 0,06 m, a porta de 0,90 m deixa
0,46 m de passagem livre.

### Testes

Rodam sem ROS, sem simulador e sem microfone:

```bash
cd ros2/ros2_ws/src/voice_cleaner
PYTHONPATH=. /usr/bin/python3 -m pytest test/test_commands.py test/test_house.py -q
```

O que mais vale é `test_todo_comodo_e_alcancavel`: monta a grade com a inflação
de verdade e roda o A* da base até cada um dos sete cômodos. Estreitar uma
porta ou subir a folga fecha a passagem sem mudar nada que se note olhando o
mapa — e o sintoma, em uso, é o robô recusando o comando.

### O resto do pacote

O `voice_cleaner` também carrega os nós herdados do `potential_field`
(`navigator`, `yolo_vision`, `perception_map`, `fake_world`, `command_console`)
e os launches `voice.launch.py`, `voice_cleaner.launch.py` e
`perception_voice.launch.py`, que não fazem parte desta demonstração. Estão
documentados no README do `potential_field`.
