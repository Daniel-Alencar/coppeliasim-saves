# 6 - PID_roomba

Controle PID aplicado ao robô aspirador. A pasta tem só a cena, sem controlador
versionado — o script fica dentro do próprio arquivo do CoppeliaSim.

```
6 - PID_roomba/
└── PID with roomba.ttt
```

---

## Como rodar

1. Abra o CoppeliaSim e carregue [`PID with roomba.ttt`](PID%20with%20roomba.ttt).
2. Dê *play*.

Não há terminal, ROS 2 nem Remote API envolvidos: como nos projetos 1, 2 e 4, o
controle roda como child script dentro da cena. Para ler ou editar o código,
abra o script na *Scene Hierarchy*.

---

## O que é um PID, no contexto deste robô

Um controlador PID corrige um **erro** — a diferença entre o valor desejado e o
medido — somando três termos:

| Termo | O que faz | Efeito de aumentar o ganho |
|---|---|---|
| **P** (proporcional) | reage ao erro atual | responde mais rápido, mas oscila |
| **I** (integral) | acumula o erro passado | elimina erro permanente, mas pode dar *overshoot* |
| **D** (derivativo) | reage à taxa de variação do erro | amortece a oscilação, mas amplifica ruído |

Nos projetos anteriores deste repositório o controle é **só proporcional**: em
[`2 - obstacles`](../2%20-%20obstacles/) o robô gira com `rotVel = K * erro_de_ângulo`,
e no controlador 2 de [`4 - roomba`](../4%20-%20roomba/) o `K_HEADING` faz o
mesmo papel. Esse último já usa um termo derivativo no seguir-parede
(`K_WALL_D`), então é o antecedente mais próximo deste projeto.

Comparar os dois é o exercício natural: onde o proporcional puro deixa erro
permanente ou oscila, e o que o I e o D mudam.

---

## Ajustando os ganhos

O caminho usual é o mesmo, seja qual for a variável controlada:

1. Zere I e D. Aumente P até a resposta ficar rápida e começar a oscilar.
2. Acrescente D para amortecer a oscilação.
3. Acrescente I só se sobrar erro permanente, e com valor pequeno.

Vale registrar os valores que funcionaram, porque eles dependem da massa do
robô, do atrito das rodas e do passo de simulação (50 ms nas cenas deste
repositório).

---

> **Nota.** Não consegui inspecionar a cena sem abri-la no CoppeliaSim — os
> `.ttt` são comprimidos, e o simulador está aberto com outra cena em execução.
> Por isso este README não descreve o script de controle nem os ganhos usados:
> qual variável o PID controla (rumo? distância? velocidade das rodas?), quais
> são os ganhos e onde ficam. Abra a cena e me diga, que eu completo.
