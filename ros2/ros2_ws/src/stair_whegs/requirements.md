# Trabalho: rover de whegs que sobe escada

## Contexto

Todos os robôs da disciplina até aqui têm **rodas**, e todos andam em piso
plano. Uma roda não sobe degrau: ela precisaria de atrito na face vertical do
espelho, que não existe. É uma limitação da morfologia, não do controle — nenhum
planejador conserta isso.

Este trabalho troca a morfologia. O robô é um rover de **whegs** (*wheel-legs*):
rodas de três raios curvos, que rolam como roda no plano e **engancham** a quina
do degrau como perna na escada. É a geometria do Mini-Whegs e do RHex.

A troca traz duas famílias de conceitos novas:

1. **Locomoção com pernas e coordenação de fases.** Não basta girar os
   atuadores: o que define uma marcha é a FASE relativa entre as pernas. Há
   marchas que andam melhor e marchas que sobem melhor, e elas não são a mesma.
2. **Percepção de terreno guiando a locomoção.** De olhos fechados o rover sobe
   e, no topo, atravessa o patamar e cai do outro lado. A camada que olha o
   terreno e decide — achou degrau, está torto, chegou ao fim — é o que
   transforma um mecanismo numa máquina autônoma.

## 1. Preparação da cena

- O CoppeliaSim **não tem** um modelo de rover de whegs: ele tem de ser
  construído pela Remote API, peça por peça.
- A geometria do robô tem de ser **variável de experimento** (raio do wheg,
  número de raios, curvatura, entre-eixos, cauda), e não constante no código.
- A escada também: altura do espelho, profundidade do piso, número de degraus.
- A cena tem de ter um **patamar** no topo, ou não há como distinguir "chegou"
  de "empinou na beirada".

## 2. Tarefas

### Tarefa 1 — Construção do rover

- Construir o wheg a partir de peças convexas (o motor de física trata mal
  malhas côncavas).
- Montar o chassi, as quatro juntas e os sensores.
- Justificar a presença ou ausência da cauda com medida, não com opinião.

### Tarefa 2 — Gerador de marcha

- Implementar `θ_i(t) = φ_i + ω·t` com as fases de cada marcha.
- Implementar pelo menos três padrões de fase e o giro por derrapagem.
- Permitir trocar de marcha **em movimento**, sem salto nas juntas.
- Decidir, com medida, entre comandar as juntas em posição ou em velocidade.

### Tarefa 3 — Percepção de terreno

- Detectar que há um degrau à frente.
- Medir o desalinhamento do rover com a face do degrau e corrigi-lo.
- Detectar o fim da escada e a beirada do patamar.
- Os sinais vêm de sensores montados num corpo que balança 25 mm e inclina
  ±15° a cada passo: o tratamento desse ruído faz parte da tarefa.

### Tarefa 4 — Máquina de estados

- Procurar, alinhar, subir, detectar o topo, parar.
- Detectar que empacou e tentar de novo.
- Parar antes de cair.

## 3. Experimentos e discussão

- **Altura máxima de degrau** vencida, em função do raio do wheg. Comparar com
  a estimativa geométrica.
- **Número de raios:** o compromisso entre subir degrau e andar liso.
- **Comparação de marchas** num degrau fácil e num degrau difícil. A diferença
  só aparece perto do limite.
- **Taxa de sucesso**, variando a fase em que o rover encontra o degrau — um
  ensaio determinístico repetido não mede robustez nenhuma.
- **Efeito da cauda.**
- **Marcha aberta vs. guiada por percepção:** o que a camada de decisão
  acrescenta.

Discutir por que a marcha melhor para andar não é necessariamente a melhor para
subir, e o que isso diz sobre escolher marcha em função do terreno.
