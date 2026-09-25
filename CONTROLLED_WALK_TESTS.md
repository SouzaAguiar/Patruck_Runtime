# Próximo ensaio: comando limitado e registro de apoio

## Instalação

Encerre o runtime e extraia `patruck-controlled-tests.zip` na raiz
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas. O pacote atualiza
o script de caminhada, o controlador web Python e os três arquivos da interface.
Não altera ONNX, ganhos, offsets, calibração ou configuração de boot.

Reinicie com o comando habitual e reabra/recarregue completamente a página.
Confirme que aparecem **Limite frente/trás** e **Apoio da mão**. O limite inicial
é **0,03 m/s**. Uma aba antiga não recebe esses controles até ser recarregada.

Preserve as opções do último ensaio, trocando apenas o rótulo:

```text
--imu-i2c-bus 8 --imu-max-age-ms 50 --runtime-gc bounded --active-window-s 10 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-controlled-tests-v3
```

Este bloco complementa o comando de caminhada em `scripts`; não é comando
independente. Modelo e demais caminhos continuam os já usados no Raspberry.

## Alteração da seleção da IMU

A reserva de idade continua 10 ms quando o limite final é 50 ms: a seleção
exige amostra com até 40 ms. A espera deliberada continua limitada a 5 ms e
ao restante do período de controle (20 ms a 50 Hz). Ela deixa de terminar
automaticamente na metade do período por subtrair novamente a reserva.

Ao voltar da última consulta, uma amostra recente pode ser aceita mesmo se o
prazo de espera tiver passado, desde que ainda esteja dentro do período do
ciclo e cumpra a reserva de idade. Se não houver amostra com margem, não há
nova espera após o prazo. Ultrapassar o período do ciclo também causa pausa.
Linux pode atrasar o retorno da chamada: esses limites não são garantia de
tempo real rígido.

Continuam as checagens do orçamento de execução, após inferência e antes de
enviar aos motores. O limite final de **50 ms** é aplicado à amostra efetivamente
usada para calcular a ação. Não há reaproveitamento de ação antiga, repetição
de inferência após falha ou avanço do histórico sem envio. Esta revisão busca
reduzir as pausas preventivas observadas; o efeito precisa ser medido no Pi.

## Controles novos

- **Limite frente/trás:** teto simétrico aplicado pelo servidor à velocidade
  solicitada (não à velocidade física medida). Opções 0,03; 0,05; 0,08; 0,15 m/s.
  Trocar o limite zera os comandos e exige novo gesto. O giro e o movimento
  lateral livre não são limitados por esse seletor.
- **Só frente/trás:** ligue antes do teste. O bloqueio lateral continua aplicado
  no servidor. Deixe o joystick de giro centralizado.
- **Apoio da mão:** marca o período de apoio informado pelo operador; não altera
  comandos nem pausa. Marque “sim” quando houver contato/apoio e “não” quando
  terminar. Se precisar usar a mão imediatamente, priorize conter o robô;
  uma marcação tardia é aproximada e deve ser informada na análise. Pode ser
  operada por uma segunda pessoa. Não é sensor de contato ou detector de queda.

Ao recarregar a página: limite 0,03, apoio “não” e bloqueio lateral desligado.
Reconfigure o bloqueio e o apoio real antes de INICIAR. Mantenha uma única aba
de controle. Clientes antigos que não enviam o limite preservam a faixa anterior
de ±0,15 e ficam identificados por `longitudinal_limit_reported=false`.
Um limite explícito inválido é substituído por 0,03 pelo servidor.

## Sequência de teste

Mantenha apoio contra queda: início pausado ainda envia a pose inicial; pausa
mantém o último alvo e não garante equilíbrio. Não é necessário provocar quedas.

1. Ligue **Só frente/trás**, confira limite **0,03** e indique se há apoio da mão.
2. Faça uma janela de 10 s com comandos zerados, observando pose e equilíbrio.
3. Após reconhecer a pausa, faça pulsos manuais curtos (aproximadamente 1 s)
   somente para frente, soltando entre pulsos. Use PARAR se a instabilidade exigir.
4. Faça uma janela separada para trás em 0,03. Não misture direções na mesma
   janela para facilitar a comparação. Não use SPRINT nem altere PASSO.
5. Se 0,03 permitir marcha controlada sem crescente necessidade de apoio,
   repita frente e trás em **0,05**. Se já houver tendência a cair, preserve
   o ensaio em 0,03 e encerre: não é necessário subir até 0,08 ou 0,15.
6. Registre em qual janela começou a deriva/tendência para frente e quando
   precisou de apoio. Vídeo ajuda, se disponível. Encerre com Ctrl+C e aguarde
   concluir a gravação.

Os pulsos são manuais: não existe sequência automática de movimentação.
Não mude modelo, ganhos, offsets ou posição da cabeça entre essas janelas.

## Telemetria esperada

Em metadata: `imu_selection_version=2`. Em `command_source`:

- `client_version=web-controlled-tests-v3`;
- `longitudinal_limit_reported=true` e `longitudinal_limit_m_s` com o teto aplicado;
- `hand_support_reported=true` e `hand_support` com a anotação atual;
- bloqueio lateral solicitado/aplicado true nos trechos de ensaio;
- `effective_commands` igual aos comandos entregues à política; lateral e
  giro zerados, longitudinal dentro do limite escolhido.

Durante timeout/desconexão, os campos da última mensagem servem como contexto:
consulte `command_fresh`, `connected` e `effective_commands` para a saída atual.

Os ciclos também incluem `imu_selection_wait_deadline_overrun_ms` (retorno
além do prazo de espera) e `imu_selection_cycle_remaining_ms`. Em uma falha,
`selection_cycle_deadline_ns` complementa o prazo de espera e a idade. Isso
permite distinguir atraso de agendamento de falta de amostra recente.

Envie a pasta completa da sessão, incluindo metadata, status, todos os chunks
e writer_timing. Contexto de apoio precisa acompanhar qualquer uso futuro dos
dados para adaptação da política.

## Verificação local

103 testes Python e 2 testes JavaScript aprovados. A integração de limite/apoio
com telemetria e observação da política foi revalidada após sua ampliação.
Oito cenários de layout (320×568, 390×844, 844×390, 1280×800; câmera visível/oculta)
verificados em Chromium: sem transbordamento horizontal nem sobreposição dos
novos controles, com rolagem quando necessária. Nenhum hardware acionado.
