# 1 - robot

O ponto de partida da disciplina: a montagem do robô diferencial que todos os
projetos seguintes reutilizam. A pasta tem só a cena, sem controlador.

```
1 - robot/
└── myRobot.ttt
```

---

## Como abrir

Abra o CoppeliaSim e carregue [`myRobot.ttt`](myRobot.ttt). Não há script para
colar, terminal para rodar nem dependência de ROS 2 — é uma cena de montagem.

Dê *play* para conferir que o robô se mantém em pé e que as juntas respondem.
Sem controlador ele fica parado; para vê-lo andar, use os projetos seguintes.

---

## A convenção de nomes que vale para todo o repositório

Esta é a cena que estabelece os nomes que os controladores dos outros projetos
esperam encontrar:

```
/myRobot
├── leftMotor          (revolute joint, modo de velocidade)
├── rightMotor         (revolute joint, modo de velocidade)
└── proximitySensor
```

Se você renomear qualquer um deles, os scripts de
[`2 - obstacles`](../2%20-%20obstacles/), [`3 - remoteAPI`](../3%20-%20remoteAPI/)
e [`4 - roomba`](../4%20-%20roomba/) param de achar os objetos.

Duas características do robô que aparecem como constante em quase todo
controlador do repositório:

| Grandeza | Valor |
|---|---|
| Raio da roda | 0,05 m |
| Distância entre rodas | 0,2 m |
| Sinal do motor | **negativo** anda para a frente |

Esse sinal invertido é consequência de como as juntas foram montadas aqui, e é
a origem do `MOTOR_SIGN = -1` que reaparece nos projetos 4 e no pacote
`robot_docking`.

---

## Para onde ir depois

| Projeto | O que acrescenta |
|---|---|
| [`2 - obstacles`](../2%20-%20obstacles/) | navegação com campos potenciais, em child script |
| [`3 - remoteAPI`](../3%20-%20remoteAPI/) | controle de fora do simulador, pela ZeroMQ Remote API |
| [`4 - roomba`](../4%20-%20roomba/) | cobertura de área |
| [`5 - tf_demo`](../5%20-%20tf_demo/) | percepção com YOLO e árvore de TFs |

---

> **Nota.** Não consegui inspecionar o conteúdo da cena sem abri-la no
> CoppeliaSim — os arquivos `.ttt` são comprimidos. A hierarquia acima é a que
> os controladores dos outros projetos assumem; se a sua cena divergir, o nome
> real é o que vale.
