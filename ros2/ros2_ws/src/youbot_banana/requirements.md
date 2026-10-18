# Trabalho final: manipulador móvel — pegar e depositar as bananas

## Contexto

Todos os exercícios da disciplina pararam no mesmo ponto: o robô **chega** até a
banana. O robô diferencial do CoppeliaSim não tem como fazer mais do que isso —
ele não tem braço. Este trabalho troca a plataforma por um **manipulador
móvel**, o KUKA youBot, e fecha a tarefa: o robô vai até a banana, **pega**, e
**deposita** numa bandeja.

A troca de plataforma traz duas famílias de conceitos que o semestre não tinha
visto, e é por elas que o trabalho existe:

1. **Base holonômica.** As quatro rodas suecas do youBot dão as três liberdades
   do plano. A restrição não-holonômica que justificava a lei ρ-α-β desaparece,
   e o controle de posição fica mais simples — vale comparar os dois.
2. **Cinemática de manipulador.** Um braço de 5 juntas, com cinemática direta e
   inversa, espaço de configuração, limites de junta e envelope de alcance. É a
   primeira vez que "onde o robô está" deixa de ser três números.

O planejamento global **não** é refeito: o A\* e o mapa de ocupação são
importados do exercício anterior.

## 1. Preparação da cena

- Montar a cena pela Remote API, a partir dos modelos do CoppeliaSim e das
  bananas e poops dos exercícios anteriores. A cena tem de ser **reproduzível**
  a partir de uma semente, para os experimentos poderem ser repetidos.
- Nenhuma coordenada de objeto escrita no código: posições, orientações e
  tamanhos saem da hierarquia da cena pela ZeroMQ Remote API.
- A cena contém: o youBot, paredes fechando a arena, obstáculos (poops), as
  bananas e a bandeja de depósito.

## 2. Tarefas

### Tarefa 1 — Cinemática da base holonômica

- Escrever a cinemática inversa da base de quatro rodas suecas: do twist do
  corpo `(vx, vy, ω)` para a velocidade de cada roda.
- **Medir** no simulador se o modelo corresponde à base: comandar cada
  liberdade isoladamente e comparar o movimento observado com o pedido.
- Discutir o escorregamento dos rolos e a saturação do motor da roda, e como
  eles limitam a velocidade utilizável.

### Tarefa 2 — Cinemática do braço

- Escrever a cinemática **direta**: das cinco juntas para a posição da ponta da
  garra.
- Escrever a cinemática **inversa** de forma analítica, aproveitando a estrutura
  do braço (uma junta vertical, três juntas coplanares, um giro de ferramenta).
- Determinar o **envelope de alcance** para preensão por cima, e os limites de
  junta que o restringem.
- **Validar** contra o simulador: comandar as juntas e comparar a posição que o
  modelo prevê para a garra com a posição medida.

### Tarefa 3 — Planejamento com A\*

- Reaproveitar o A\* e o mapa de ocupação do exercício anterior, ajustando
  apenas o que descreve este robô (footprint maior) e os obstáculos novos.
- O alvo da base **não é a banana**: é uma pose de preensão, de onde o braço
  alcança. Decidir de que direção aproximar faz parte do planejamento.

### Tarefa 4 — Preensão

- Levar a garra até a banana, fechar em volta dela e levantá-la, usando a
  física do simulador (não vale prender o objeto na garra por software).
- Orientar a garra conforme a orientação da banana.
- Detectar se a preensão deu certo.

### Tarefa 5 — Missão completa

- Máquina de estados: planejar, navegar, aproximar, descer, fechar, levantar,
  transportar, depositar, repetir.
- Tratar as falhas: alvo fora do alcance, garra fechando no vazio, banana
  caindo no caminho, destino inalcançável.
- O sucesso é a banana **dentro da bandeja** — pegar e derrubar não conta.

## 3. Experimentos e discussão

- **Missão completa:** demonstrar o robô coletando e depositando as bananas sem
  colidir.
- **Resolução da grade:** repetir com outra resolução e comparar caminho e
  comportamento.
- **Raio de preensão:** repetir com o braço mirando a borda do envelope em vez
  do meio, e medir o efeito na taxa de sucesso.
- **Destino inalcançável:** uma banana cercada de obstáculos, ou larga demais
  para a garra.
- **Repetibilidade:** várias sementes de cena, medindo quantas bananas são
  depositadas.

Discutir como a inflação do mapa, o envelope do braço e o escorregamento das
rodas interagem — em particular, o que acontece quando o raio de inflação fica
maior que o alcance do braço.
