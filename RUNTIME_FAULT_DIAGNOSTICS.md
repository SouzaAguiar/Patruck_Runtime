# Próximo ensaio: localizar o atraso residual da IMU

Atualização de instrumentação para o runtime com GC já integrado. Não muda o
modelo, ganhos, limite de 50 ms, duração das janelas, pausa ou condições de retomada.
Nenhum teste local aciona hardware.

## Instalação e execução

Com o runtime encerrado, extraia o pacote na raiz do checkout do Raspberry,
preservando os caminhos. Os arquivos alterados nesta etapa são:

```text
scripts/v2_rl_walk_mujoco.py
mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
mini_bdx_runtime/mini_bdx_runtime/telemetry.py
```

O pacote também inclui os componentes e o guia da integração anterior de GC.
Não substitui ONNX, configuração, calibração ou boot. Não requer dependência nova.
Confirme a versão instalada sem inicializar a IMU nem motores:

```bash
python -c "from mini_bdx_runtime.raw_imu import Imu; from mini_bdx_runtime.telemetry import TelemetryRecorder; print('snapshot IMU:', hasattr(Imu, 'diagnostic_snapshot')); print('snapshot gravador:', hasattr(TelemetryRecorder, 'writer_snapshot'))"
```

Ambos devem mostrar `True`. Use o mesmo ambiente e comando do último ensaio,
mantendo estas opções e um rótulo novo:

```text
--runtime-gc bounded --active-window-s 10 --imu-max-age-ms 50 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-gc-fault-details
```

Exemplo, substituindo apenas o caminho de modelo e preservando suas demais opções:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
python v2_rl_walk_mujoco.py --onnx_model_path /caminho/real/student.onnx --control-source web --imu-i2c-bus 8 --runtime-gc bounded --active-window-s 10 --imu-max-age-ms 50 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-gc-fault-details
```

Mantenha o robô apoiado: mesmo com início pausado, há envio da pose inicial aos
motores. A pausa mantém o último alvo e não garante equilíbrio.

Faça algumas janelas curtas, primeiro com comandos zerados e depois comandos
breves para frente. Inclua uma pausa pelo limite de 10 segundos e uma retomada
explícita (PARAR, centralizar controles, INICIAR). Se ocorrer falha da IMU,
preserve os dados e siga o mesmo procedimento de reconhecimento. Não aumente
o limite para evitar a pausa. Encerre com Ctrl+C e aguarde o gravador concluir.

## O que foi acrescentado

Cada evento `imu_fault` agora inclui:

- `detected_monotonic_ns`: instante no início do tratamento da falha, antes de
  restaurar GC, imprimir e registrar a pausa;
- `diagnostic`: número da tentativa, próximo índice de ação, etapa e tempos
  parciais disponíveis (uma tentativa interrompida não avança o índice de ação);
- `used_sample`: índice e timestamps da amostra usada, ou null se não foi obtida;
- `latest_reader`: snapshot dos metadados da última amostra em cache, erro do
  produtor e contador de leituras válidas, **sem nova leitura I²C**;
- `writer`: lote/fase de publicação em andamento e tamanho da fila no snapshot;
- comandos vigentes no momento da falha.

As etapas distinguem `get_imu`, `validate_imu`, `after_joint_reads`,
`after_inference_used_sample`, `after_inference_latest_sample`,
`before_motor_write` e `resume_imu_ready`. Os tempos de posições, velocidades,
contatos, inferência, preparação de alvos e verificações de orçamento permitem
localizar onde o tempo foi consumido. Campos não alcançados ficam ausentes;
não são preenchidos com tempos de ciclos anteriores.

`latest_reader` pode ser mais novo que `used_sample`, pois o leitor continua
executando. São snapshots em instantes próximos, não uma captura atômica de
todas as threads. Seus valores não são usados para alterar ações ou aceitar
uma amostra rejeitada. O timestamp da rejeição identifica seu tratamento;
não é uma medição elétrica da aquisição.

## Tempos da gravação

Ao terminar, a pasta contém `writer_timing.json` com `batches`. Cada lote inclui
início, fim da serialização/escrita das linhas, fim do flush/fsync e fim da
publicação/status. A primeira fase inclui a espera de 1 ms entre linhas. Não
interpretar duração do lote como bloqueio integral da thread de controle.

O buffer é limitado aos últimos 4.096 lotes. `total_batches` e `omitted_batches`
informam a cobertura. A exportação acontece no encerramento, fora da execução
ativa; não se adiciona escrita desse arquivo em cada ciclo. Um lote com falha
tem `failed=true` e pode não conter todos os marcos. Se a thread não terminar
no prazo de encerramento, o arquivo pode faltar e haverá aviso no terminal.

Confirme no `metadata.json`: `settings.imu_fault_diagnostics_version=1` e
`writer_timing_schema=1`. Esses indicadores complementam os hashes dos fontes.
Envie **a pasta completa**, incluindo todos os chunks, metadata, status e
writer_timing. Os eventos de memória/manutenção já existentes continuam nela.

Esta instrumentação acrescenta algum custo de software. Os testes locais
verificam preservação das ações e bloqueio dos envios nas mesmas condições;
a latência efetiva precisa ser medida no Raspberry. Não há alteração do critério
de validade ou tentativa automática de reenviar uma ação após falha.
