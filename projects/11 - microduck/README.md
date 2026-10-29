# 11 — Microduck no CoppeliaSim

O [Microduck](https://pollen-robotics.com/microduck) é um bípede de 25 cm e
800 g da Pollen Robotics, governado por políticas de aprendizado por reforço. O
repositório dele — `microduck/`, ao lado deste, fora do controle de versão — é
o **cérebro**: os daemons em Rust, o laço de controle a 50 Hz, as políticas
ONNX. O **corpo** vive noutro repositório (`microduck_rl`), em MuJoCo.

Este projeto põe o CoppeliaSim no lugar do MuJoCo.

```
   microduck/ (Rust)                      este projeto (Python)
   ┌────────────────────┐                 ┌──────────────────────┐
   │ robotd --sim :7801 │ ──── TCP ─────▶ │ duck_body.py         │ ──ZeroMQ──▶ CoppeliaSim
   │  laço 50 Hz        │  JSON por linha │  protocolo do fio    │
   │  política ONNX     │ ◀──────────────  │  passo da simulação  │
   │  segurança, queda  │                 └──────────────────────┘
   └────────────────────┘
```

A costura é uma só, e é escolhida pelo próprio Microduck: `robotd --sim` troca
o barramento dos servos por `duck_control::sim::RemoteIo` e passa a falar TCP.
Tudo acima dela — o laço, a política, a segurança, a detecção de queda, a
cinemática, a odometria, o IPC, o `robotctl` — é o código que um robô roda, e
não tem como saber que o corpo mudou de simulador.

## Os arquivos

| | |
|---|---|
| [build_duck.py](build_duck.py) | Monta o pato na cena a partir do MJCF do próprio robô |
| [duck_body.py](duck_body.py) | Serve o corpo na porta 7801, falando o protocolo do `duck-body` |
| [poke_body.py](poke_body.py) | Exercita o corpo pelo protocolo, no lugar do `robotd` |
| [walk.py](walk.py) | Roda a política de marcha do robô contra esse corpo |

`policies/` guarda a política baixada do Hugging Face. Ela **não é autoria
deste repositório** e não precisa ser versionada — o próprio Microduck a busca
em vez de embarcar, porque um retreino de marcha não deve exigir uma release do
daemon.

## Como rodar

```bash
# 1. o pato na cena (CoppeliaSim aberto)
python3 build_duck.py

# 2. o corpo no fio
python3 duck_body.py

# 3a. sem o cérebro: varrer uma junta, ver o IMU, derrubar o pato
python3 poke_body.py --sweep left_knee
python3 poke_body.py --limp

# 3b. com a política de marcha do robô (ver §Andar)
mkdir -p policies && curl -L -o policies/velstand.onnx \
  https://huggingface.co/pollen-robotics/microduck-policies/resolve/main/velstand.onnx
python3 walk.py --vx 0.15

# 3c. com o cérebro inteiro, do outro repositório
robotd --sim 127.0.0.1:7801
```

## O que foi preciso descobrir

### O MJCF não tem geometria

`microduck/kinematics/assets/alpha/robot_walk.xml` traz a árvore cinemática, as
juntas com os seus limites e a **inércia** de cada corpo — e nenhum `<geom>`.
As malhas vivem no outro repositório.

Montar o pato aqui exige, então, inventar a forma de cada peça, e a escolha
feita é a única que não inventa física junto: para cada corpo, a **caixa de
densidade uniforme cuja inércia é a que o MJCF declara**. Dadas a massa `m` e
a diagonal de inércia,

```
Ixx = m(b² + c²)/12        a² = (B + C − A)/2,  A = 12·Ixx/m
Iyy = m(a² + c²)/12   ⟹    b² = (A + C − B)/2,  B = 12·Iyy/m
Izz = m(a² + b²)/12        c² = (A + B − C)/2,  C = 12·Izz/m
```

A caixa que sai tem a massa certa, a inércia certa e o centro de massa no lugar
certo. O que ela não tem é a aparência de um pato.

A exceção são os **pés**: o MJCF marca a sola com um `site`, e a caixa de
inércia do tornozelo não chega lá. Sem uma sola explícita o pato não tem em que
se apoiar, então cada pé ganha uma placa fina na posição do site.

### O frame do corpo não é o centro de massa

No MJCF cada corpo tem um **frame** — onde a junta mora e onde a pose do filho
é medida — e um `inertial pos`, que é onde está o **centro de massa**, quase
nunca na origem: o tronco tem o dele 18 mm atrás, a casca da cabeça 33 mm
abaixo.

A primeira versão deste projeto pôs cada caixa na origem do frame, que é o que
um shape primitivo do CoppeliaSim faz por padrão. O pato tombava **73° em um
segundo**. Num bípede, mover o centro de massa de 18 mm é mover a decisão entre
ficar de pé e cair.

A correção parecia ser `sim.relocateShapeFrame`, que separa o frame da
geometria. Ele **devolve `False`** para um shape primitivo e não faz nada — e
não checar esse retorno custou uma rodada inteira de depuração. A solução que
ficou é aritmética: o frame do shape fica no centro de massa, e `body_to_link`
desconta esse deslocamento ao posicionar cada filho e cada site. A conta é a
mesma; o que muda é onde ela vive.

A verificação é direta e vale a pena: a cinemática direta do MJCF, calculada em
Python, contra as posições que o CoppeliaSim reporta. **Erro máximo de 0,00 mm
nos quinze corpos.**

### A boca não existe

O protocolo indexa **quinze** juntas (`duck_ipc_proto::JOINT_NAMES`); o MJCF
tem **catorze**. Falta `mouth`, que nenhuma política alfa aciona — o vetor de
ação delas é de 14 e salta esse índice.

O lugar dela no vetor tem de continuar existindo. Reportá-la parada em zero
custa uma linha; omiti-la faria os catorze índices restantes andarem um para a
esquerda, e a observação inteira sairia errada sem nada acusar.

### O cliente ZeroMQ não é thread-safe

O servidor começou como um `ThreadingTCPServer`, que é o reflexo normal para um
serviço de rede. O daemon conectava, o `hello` respondia, e a primeira operação
que tocava o CoppeliaSim **travava** — sem erro, sem log, até o cliente desistir
por timeout.

A razão é que o cliente ZeroMQ guarda um socket, e socket ZeroMQ não é
thread-safe: o `Body` é criado na thread principal e o handler roda noutra. O
servidor é monothread de propósito, e o robô abre uma conexão só — servir uma
por vez é o que o caso de uso pede e remove a classe inteira de problema.

O sintoma merece registro porque ele não se parece com a causa: o que se vê é
um daemon esperando a resposta de um `torque`.

### O primeiro passo custa um segundo

Medido: o primeiro `sim.step()` depois de `startSimulation()` leva ~1 s — é
onde o CoppeliaSim monta o mundo dinâmico. Os seguintes levam ~15 ms.

O servidor paga esse primeiro passo ao subir, e não no primeiro tique do
daemon. Um tique mil vezes mais longo que os outros é exatamente o tipo de
coisa que um detector de queda lê como um salto de estado.

### Quem dá o passo da simulação

O `write` é o fim de um tique do robô: ele leu os sensores, rodou a política e
mandou os alvos. É aí que `duck_body.py` adianta o mundo em um período de
controle — 20 ms do robô, quatro passos de 5 ms da cena.

Deixar a cena correr livre e só amostrar foi descartado pela mesma razão que as
pontes dos outros trabalhos deste repositório rodam em *stepping*: em curso
livre o laço fica à mercê da carga da máquina. Aqui seria pior, porque uma
política de RL foi treinada contra um passo fixo.

## O que foi medido

### A cinemática confere

A cinemática direta do MJCF, calculada em Python, contra as posições que o
CoppeliaSim reporta, na pose de repouso: **erro máximo de 0,00 mm nos quinze
corpos**. As solas ficam em z = 2,9 mm com o tronco a 122 mm — encostadas no
chão, como o MJCF pretende.

### O centro de massa decide se ele fica de pé

| | pitch após 1 s |
|---|---|
| caixa na origem do frame | **+73°** — caiu |
| caixa no centro de massa | **+10°** — de pé, estável |

Com a massa no lugar certo o tronco assenta em 110 mm e segura, inclinado 10°
para a frente.

### O corpo roda a 21–31 Hz, e isso não é um defeito

| | |
|---|---|
| chamadas da Remote API por tique | ~8,5 ms |
| `sim.step()` x4, com desenho | 60 ms |
| `sim.step()` x4, sem desenho | 20 ms |
| tiques servidos, medido | **21 Hz** com desenho, **31 Hz** sem |

O laço do robô é 50 Hz, então o mundo roda a cerca de 0,4–0,6 do tempo real. Um
tique continua adiantando exatamente 20 ms de simulação; o que muda é quanto
tempo de parede ele leva. O próprio Microduck mede isso (`duck-sim realtime`) e
trata rodar devagar como um modo normal. O que seria erro é o passo **variar**,
e ele não varia.

O gargalo é a física, não o protocolo: 14 juntas lidas e escritas custam
~8,5 ms, e quatro passos de 5 ms custam 20 a 60 ms. `--no-display` é o botão
que compra o fator de três.

### A costura responde

Com `poke_body.py` no lugar do `robotd`, varrendo `head_yaw` em ±1,2 rad: o
alvo vai, a junta segue com o atraso de um servo de posição, e a leitura volta
pelo mesmo índice. O IMU reporta gravidade `(0, 0, −1)` com o pato em pé e
`(−1, 0, 0)` com ele deitado.

Segurando só a pose de repouso, o pato fica de pé cerca de quatro segundos e
tomba. **É o comportamento certo:** não há política de equilíbrio aqui, e quem
segura um bípede é o `robotd` com a política carregada.

## O que falta para o cérebro de verdade

Rodar o `robotd` contra este corpo precisa de duas coisas que esta máquina não
tem:

| | |
|---|---|
| **Toolchain Rust** | Não há `cargo` instalado. O workspace do Microduck tem quinze caixas e puxa ONNX Runtime e GStreamer junto. |
| **As políticas** | Não estão no repositório: `scripts/seed-policies.sh` as busca no Hugging Face Hub, porque uma política não acompanha uma release do daemon. |

Nenhuma das duas é impedimento de projeto — são instalações. A costura, que era
a parte que podia não fechar, fechou.

## Andar: até onde chegou

Construir o `robotd` precisaria de uma toolchain Rust que a máquina não tem.
Mas a parte do tique que faz o robô andar é pequena e está toda escrita no
repositório do Microduck, então [walk.py](walk.py) a refaz em Python e roda a
**política de verdade** — `velstand.onnx`, do conjunto oficial no Hugging Face:

```
duck-control/src/obs.rs     o vetor de observação, 61 floats
robotd/src/control.rs       alvos ← pose de repouso + 0,9 × ação, mais dois filtros
duck-control/src/policy.rs  quando a rede de postura substituiria a de marcha
```

### O que funcionou

**O vetor de observação está certo.** É a parte que o `obs.rs` chama de código
de maior risco do pacote — 61 floats em que um índice errado não falha, só
produz um robô plausível que cai. Verificado rodando a política sozinha, sem
simulador: mudar `vx` muda a ação de forma coerente e crescente, e `vyaw` e
`vy` dão respostas distintas.

| comando | ação \|max\| |
|---|---|
| parado, `vx` = 0,00 | 0,095 |
| andar, `vx` = 0,15 | 0,175 |
| andar forte, `vx` = 0,40 | 0,365 |
| girar, `vyaw` = 1,00 | 0,312 |

**A política é reativa, sem relógio interno.** Com o estado das juntas travado
e a ação realimentada, ela converge para um ponto fixo: a oscilação pico-a-pico
é exatamente zero em todas as catorze juntas. A marcha não está dentro da rede
— ela nasce do **laço fechado com o corpo**. Isso tem uma consequência direta,
e é a próxima seção.

### O ganho do servo decide se existe marcha

O padrão do CoppeliaSim é P = 0,1, e com ele um degrau de 0,3 rad no joelho
leva **205 ms** para assentar — dez tiques de controle. A junta nunca chega ao
alvo dentro do tique, a realimentação não se desenvolve, e o resultado medido
foi um pato **de pé e parado**: ação de amplitude 0,15, tronco imóvel em
x = −1,1 cm por 300 tiques.

| P | em 20 ms | assenta | overshoot |
|---|---|---|---|
| 0,1 | 32 % | 205 ms | 0 |
| **1,0** | **78 %** | **25 ms** | **0** |
| 3,0 | 78 % | 25 ms | 0,032 rad |
| 10 | 78 % | 25 ms | 0,053 rad |

Com P = 1,0 a política passou a atuar de verdade — amplitude de ação de 1,26
contra 0,15 — e o pato **deu um passo**, avançando 5,4 cm em dois segundos.
A simulação também ficou mais rápida que o tempo real (1,17×), porque a física
deixou de brigar com um servo mole.

Uma armadilha de API no caminho: `sim.setObjectFloatParam(h,
sim.jointfloatparam_pid_p, ...)` — o caminho antigo e óbvio — **aceita a
escrita e não faz nada**; o valor lido de volta continua 0,1. O PID é por motor
de física (`bullet.posPid`, `ode.posPid`, ...). Custou uma varredura inteira de
ganho em que todas as linhas davam o mesmo número.

### O que não funcionou

O pato dá um ou dois passos e **cai** — e cai também com comando **zero**, que
é a condição em que a `velstand` deveria simplesmente ficar parada em pé. Com o
servo mole ele ficava de pé indefinidamente; com o servo que de fato segue o
alvo, a atuação da política desestabiliza este corpo.

A leitura é que a planta não é a que a política viu no treino, e há três
candidatos, nenhum descartado:

| | |
|---|---|
| **O atuador** | A política foi treinada contra um modelo de Dynamixel com folga — a tarefa de treino se chama `Mjlab-VelStand-Rough-Backlash-MicroDuck`. Aqui é um servo de posição ideal, e o torque está em 1,5 N·m contra os ~0,5 N·m de um XL330. |
| **O contato** | Os pés são placas retangulares num chão com o atrito padrão do CoppeliaSim, não as solas do CAD. |
| **A geometria de colisão** | Caixas de inércia, não as malhas. A massa e a inércia estão certas; a forma que toca o mundo não. |

A varredura que cruzaria ganho × torque medindo tempo de pé ficou **por rodar**:
o CoppeliaSim travou no meio (96 % de CPU, sem responder à Remote API) e o
trabalho parou aí.

### Por onde continuar

1. Baixar o torque de cada junta para o do XL330 real (~0,5 N·m) e repetir.
2. Subir o atrito dos pés e dar-lhes uma forma mais próxima da sola.
3. Cruzar ganho × torque medindo **quanto tempo o pato fica de pé com comando
   zero** — é o critério mais limpo, porque separa "a planta aguenta a
   política" de "a marcha transfere".
4. Se nada disso bastar, o caminho honesto é o `robotd` de verdade contra o
   corpo MuJoCo do `microduck_rl`, para ter um ponto de comparação em que a
   política comprovadamente anda.

## O que este corpo não é

| | |
|---|---|
| **Atuador** | Os servos do pato são Dynamixel, e as políticas foram treinadas contra um modelo deles (BAM, no `microduck_rl`). Aqui cada junta é uma junta em controle de posição do CoppeliaSim com limite de torque. Uma política de marcha não tem por que andar igual contra este atuador. |
| **Aparência** | Caixas de inércia, não as malhas do pato. |
| **Drivers** | O barramento Dynamixel, o rádio BLE, a câmera e a NPU não existem aqui — como não existem no simulador MuJoCo do projeto, pela mesma razão. |
| **Equilíbrio** | Com a política carregada o pato dá um ou dois passos e cai (ver §Andar). Sem ela, cai em quatro segundos. |

O que ele serve para exercitar é tudo o que está **acima** do atuador, que é
exatamente o que o `sim.rs` do Microduck diz do simulador MuJoCo: o laço de
controle, a política, a segurança, a detecção de queda, a cinemática, a
odometria e o IPC inteiro.

E a ressalva ficou maior do que parecia no começo: **o atuador é justamente
onde este corpo difere**, e uma política de locomoção é a coisa que mais
depende dele. A costura funciona; a transferência da marcha, não ainda.
