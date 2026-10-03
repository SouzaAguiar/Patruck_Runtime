# Próximos testes: tempos da IMU no runtime real

Preparado em 02/10/2026, após três diagnósticos sem rejeições. Agora vamos
medir a aquisição/publicação da IMU junto das leituras e envios dos servos,
da política e da telemetria do runtime completo.

As duas sessões usam o mesmo teacher e as mesmas opções; só o rótulo muda.
Cada sessão começa pausada, permite janelas ativas de até 10 segundos e
encerra após 90 segundos de coleta, contados depois da preparação dos motores.
O limite de idade continua 50 ms e a reserva antes de inferir continua 10 ms.

**Este teste aciona os servos e envia a postura inicial ao iniciar o runtime.**
Comandos de deslocamento ficam zero, mas a política pode ajustar as juntas
para manter a postura. Apoie/proteja o robô durante a preparação, como no
último teste parado. Pausa e encerramento mantêm o último alvo; não cortam torque.

## 1. Atualizar no Pi

Encerre o runtime e outros leitores da IMU. Extraia
`patruck-runtime-trace-tests.zip` em `/home/jonathan/Open_Duck_Mini_Runtime`,
preservando as pastas. O pacote contém fontes e instruções; mantenha o ONNX,
`duck_config.json`, calibração e referência de movimento já usados.

Ative seu ambiente `patruck-runtime` e entre na pasta scripts:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
read -r -p "Cole o caminho do ONNX teacher usado no ultimo runtime: " TEACHER_ONNX
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --dry-run
```

O dry-run verifica arquivos e o hash do teacher, sem importar hardware ou
acionar servos. Se houver erro, corrija o caminho/arquivo antes do teste real.
O teacher esperado tem SHA-256
`3c606f9381a1710cc8fecdb7442787dcbfce3ee9bc02a6f1224774ab2b3a1067`.

O lançador usa `~/duck_config.json` e `/dev/ttyACM0`, como o runtime padrão.
Se seu último comando usava outros valores, passe `--config CAMINHO` e
`--serial-port PORTA` também no dry-run e nas duas execuções abaixo.

## 2. Primeira sessão — trace-a

```bash
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --label imu-runtime-trace-a
```

1. Aguarde a preparação da postura terminar e aparecer `Starting`.
2. Abra uma única aba do controle em `http://IP_DO_PI:8080`, token `duck-test`.
   Mantenha câmera desligada, ambos os joysticks centralizados, cabeça neutra
   e “Só frente/trás” ligado. Não pressione controles de frequência/expressões.
3. Clique **INICIAR**. Observe o robô parado por até 10 segundos.
4. Se a mensagem for **“Limite de 10 s atingido”**, a janela terminou como
   previsto. Apoie se necessário, clique **PARAR** para reconhecer a pausa,
   mantenha os controles centrados, aguarde a manutenção terminar e clique
   **INICIAR** novamente. Faça até três janelas.
5. Se aparecer **“PAUSA IMU”**, não retome nesta sessão. Encerre com **Ctrl+C**
   no terminal e aguarde o encerramento e a exportação. Se houver desequilíbrio,
   interrompa antes; anote o ocorrido e marque “Apoio da mão” se intervier.
6. Após três janelas, encerre com Ctrl+C. Se não encerrar manualmente, o
   processo sai ao completar 90 segundos de coleta. Não envie comando para frente.

O modo `--stationary-test` pausa se receber qualquer comando externo diferente
de zero, inclusive lateral, giro ou cabeça. A telemetria registra
`stationary_command_rejected`. Essa é uma proteção do ensaio; não é falha da IMU.

## 3. Segunda sessão — trace-b

Depois de a primeira sessão encerrar completamente, repita na mesma superfície,
com a mesma configuração, postura e aba web:

```bash
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --label imu-runtime-trace-b
```

Siga o mesmo procedimento: até três janelas de 10 segundos, ou encerramento
na primeira PAUSA IMU. A repetição ajuda a distinguir um resultado ocasional.
Não mude PID, escala, ONNX, limite de idade, barramento ou orientação entre sessões.

## 4. Dados para análise

Cada execução cria uma pasta única em `../telemetry`; o terminal mostra o
caminho. Copie as duas pastas completas para `E:\www\Patruck_Runtime\telemetry`.
Inclua:

- `metadata.json`, `status.json` e todos os `chunk-*.jsonl`;
- `writer_timing.json`;
- **`runtime_timing_trace.json`**, novo arquivo salvo ao encerrar.

Se quiser conferir o resumo depois do encerramento, substitua PASTA_DA_SESSAO:

```bash
python summarize_runtime_trace.py ../telemetry/PASTA_DA_SESSAO --output ../telemetry/PASTA_DA_SESSAO/runtime_trace_summary.json
```

Informe se o robô ficou equilibrado, se houve apoio e qual mensagem encerrou
cada janela. Vídeo completo ajuda, mas não é necessário para esta análise temporal.
Não desligue a alimentação para encerrar normalmente: o trace reside em memória
até a saída. Uma queda de energia/SIGKILL pode impedir a exportação desse arquivo.

## Instrumentação e limites

O novo trace usa arrays numéricos com memória fixa; a opção preparada é
`--runtime-trace-seconds 120`, com cerca de 8,2 MiB de arrays pré-alocados.
Retém aproximadamente os últimos 120 segundos
à frequência de 50 Hz, com capacidade adicional para etapas do controle.
Se algum buffer girar, o arquivo informa `overwritten`; os dados recentes ficam
preservados. O tempo configurado é capacidade estimada por taxa, não recorte
exato por timestamp. Não há serialização JSON desse trace durante o controle.

Registra início da aquisição, fim da leitura e publicação concluída, prazo
nominal do próximo início, início/fim das chamadas I²C existentes, etapas do
controle (incluindo ciclos abortados), tempo do registro principal e callbacks
de GC. O gravador já exporta as fases de publicação/fsync em writer_timing.json.
Todos usam o relógio monotônico para cruzar eventos.

Não há leituras extras do barramento. Medições em Python incluem atrasos de
agendamento; uma chamada I²C longa não prova sozinha um bloqueio elétrico.
A instrumentação acrescenta algum custo, que precisa ser avaliado no Pi.
O resumo mostra correlações temporais, sem declarar causalidade automática.

O trace é opcional e fica desligado nas execuções normais. Para retirar a
instrumentação, use seu comando habitual sem `--runtime-trace-seconds`;
`--stationary-test` e `--test-duration-s` também são específicos deste ensaio.
Mantenha `--imu-frame yaw-minus-90` para preservar a correção validada.
