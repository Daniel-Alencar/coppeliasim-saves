# 📚 Projeto – Robô aspirador que recebe comandos por voz

Ponto de partida: o pacote `potential_field`, com a ponte para o CoppeliaSim,
a percepção pela câmera e a navegação por campos potenciais já funcionando.

---

## Objetivo desta etapa

O robô deve **receber** comandos falados em português. Executá-los é a etapa
seguinte: aqui o comando precisa chegar, ser entendido e ficar disponível no
ROS 2 para quem for executá-lo.

- Capturar o microfone e transcrever a fala, **offline**, sem mandar áudio
  para serviço nenhum.
- Reconhecer um conjunto fechado de comandos de robô aspirador: limpar a sala,
  mapear o ambiente, voltar para a base, parar, e assim por diante.
- Extrair o argumento quando o comando tem um ("limpar **a cozinha**").
- Publicar o comando num tópico, em formato estruturado, para que a execução
  possa ser ligada depois sem mexer na camada de voz.
- Distinguir comando de conversa: frase que não é comando não vira comando.
- Permitir testar toda a interpretação **sem microfone**, digitando as frases.

A cadeia esperada:

```
microfone → reconhecimento de fala (offline) → intenção + argumentos → tópico ROS 2
```

---

## Fora do escopo desta etapa

> O robô **não** precisa executar os comandos ainda.

O navegador continua fazendo o que já fazia no `potential_field`; nada consome
o tópico de comando. Ligar um ao outro — dar ao navegador um modo por comando,
com as tarefas de limpar, mapear e voltar à base — é o passo seguinte.
