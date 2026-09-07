# Desvio de obstáculos com APF

O algoritmo escolhido para lidar com os obstáculos foi o **APF** (*Artificial
Potential Fields* — Campos Potenciais Artificiais). O projeto foi implementado
de três maneiras diferentes, em ordem crescente de dificuldade para o robô:
`1- APF.py`, `2 - APF.py` e `3 - APF.py`.

Todos os arquivos são scripts de um robô diferencial no CoppeliaSim (cena
`myRobotSolution.ttt`) e devem ser colados no script do próprio robô.

## A ideia do APF

O robô se move como se estivesse dentro de um campo de forças:

- o **objetivo atrai** o robô, com força proporcional à distância que falta;
- cada **obstáculo repele** o robô, com força que cresce quando ele se aproxima
  e é zero além de um raio de influência `d_safe`.

A soma dessas forças dá uma direção desejada. Um controlador proporcional gira o
robô até a frente apontar para essa direção, e a velocidade linear diminui
quando o erro de ângulo é grande (`cos(erro)`), para o robô girar antes de
avançar. Chegando a 0,15 m do objetivo, ele para.

Antes de começar, o script se calibra sozinho: descobre para onde é a "frente"
(direção do sensor de proximidade), qual roda é a esquerda e a direita, e qual
sinal de velocidade faz cada roda empurrar o robô para frente. Assim o código
funciona mesmo se as rodas estiverem montadas invertidas na cena.

## As três implementações

### 1 - APF.py — mapa conhecido

O robô já sabe onde estão os obstáculos: as posições estão fixas na lista
`q_obstacles` e são usadas direto no cálculo das forças. O sensor de
proximidade só acrescenta uma repulsão extra do ponto que estiver vendo naquele
instante.

É a versão mais simples e serve para mostrar o APF puro funcionando. A limitação
é óbvia: não há percepção de verdade, o mundo foi informado ao robô.

### 2 - APF.py — mapa descoberto pelo sensor

Aqui o robô **começa sem saber de nada**. A lista `q_obstacles` só é usada para
desenhar os cilindros na cena; o cálculo das forças usa apenas
`discovered_obstacles`, preenchida em tempo de execução com o que o sensor
enxerga.

Duas mudanças fazem isso funcionar:

- **Memória.** Um obstáculo visto continua repelindo mesmo depois de sair do
  campo de visão. Sem isso o robô voltaria a entrar no obstáculo assim que
  desviasse o sensor dele.
- **Fusão de detecções** (`OBSTACLE_MERGE_DIST`). Duas leituras a menos de
  0,25 m são o mesmo obstáculo. Sem esse filtro o mapa cresceria a cada passo de
  simulação e a repulsão de um único cilindro seria somada centenas de vezes.

O cilindro do objetivo é criado como **não detectável**, senão o robô o
enxergaria como obstáculo e fugiria do próprio destino.

### 3 - APF.py — cenário aleatório

Mesma navegação da versão 2, mas o cenário é sorteado a cada execução: o
objetivo e 15 obstáculos caem em posições novas dentro da arena, por
**amostragem com rejeição** (sorteia um ponto, descarta se violar alguma folga,
tenta de novo).

As folgas exigidas são:

| Restrição | O que garante |
|---|---|
| `MIN_GOAL_DIST` | o objetivo fica longe do robô, há percurso a fazer |
| `MIN_ROBOT_CLEARANCE` | o robô não nasce dentro de um obstáculo |
| `MIN_OBSTACLE_GAP` | os cilindros não se sobrepõem |
| `MIN_GOAL_GAP` | nenhum obstáculo cai perto demais do objetivo |

A última é a mais importante. Chegando ao destino, a atração tende a zero, mas a
repulsão continua finita: um obstáculo dentro do raio `D_SAFE` do objetivo faria
o mínimo do potencial deixar de coincidir com o destino, e o robô nunca
conseguiria pousar nele.

A *seed* do sorteio é impressa no console. Para repetir um cenário problemático,
basta copiá-la para `RANDOM_SEED`.

## Parâmetros principais

| Parâmetro | Papel |
|---|---|
| `k_attractive` | intensidade da atração do objetivo |
| `k_repulsive` | intensidade da repulsão dos obstáculos |
| `d_safe` | raio de influência: além dele o obstáculo é ignorado |
| `Kp` | ganho do controlador de giro |
| `max_linVel`, `max_rotVel` | limites de velocidade do robô |

Note que `k_repulsive` cresce entre as versões (1,5 → 8,5 → 3,5): quanto menos o
robô sabe do mundo, mais cedo ele precisa reagir ao que vê.

## Limitação conhecida

O APF sofre de **mínimos locais**: se a atração e a repulsão se cancelam (por
exemplo, com um obstáculo exatamente entre o robô e o objetivo, ou uma parede em
forma de U), o robô empaca. Nenhuma das três versões trata esse caso — sair dele
exigiria uma estratégia adicional, como seguir a borda do obstáculo ou aplicar
uma perturbação aleatória.

## Como executar

1. Abra `myRobotSolution.ttt` no CoppeliaSim.
2. Cole o conteúdo de uma das versões no script Python do robô.
3. Inicie a simulação. Os cilindros verde (objetivo) e vermelhos (obstáculos)
   são criados sozinhos e removidos ao encerrar.

Com `DEBUG = True`, o console mostra a cada passo o ângulo atual, o desejado, o
erro, as velocidades e quantos obstáculos são conhecidos.
