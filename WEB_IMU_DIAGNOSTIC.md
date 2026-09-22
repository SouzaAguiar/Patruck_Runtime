# Diagnóstico web + IMU sem motores

O `scripts/web_control_test.py` agora oferece quatro modos independentes. Nenhum
importa HWI, o script de caminhada ou drivers de motores. As saídas do ONNX são
descartadas. A câmera só é inicializada com `--camera`.

Copie o `scripts/web_control_test.py` atualizado e o novo
`scripts/data/runtime_telemetry_payload.json` para os mesmos caminhos no Raspberry.
O JSON é obrigatório nos modos ONNX/telemetry. O script usa os
mesmos módulos `raw_imu.py`, `imu_safety.py`, `web_controller.py` e `telemetry.py`
já instalados na integração I²C8. Não há nova dependência além das usadas pelo
runtime; os modos ONNX precisam do `onnxruntime` já utilizado na caminhada.

## Preparação

Encerre o runtime de caminhada e outros leitores da IMU. Mantenha a alimentação
dos motores desligada e o Raspberry/IMU alimentados, na mesma posição durante
as comparações. Execute apenas um teste por vez, no ambiente Python habitual:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
```

Os modos com IMU exigem `imu_calib_data.pkl` nessa pasta e a configuração em
`~/duck_config.json` (ou `--config /caminho/duck_config.json`). O barramento é 8,
a idade máxima é 50 ms e o ciclo do teste é 50 Hz. Não há fallback de barramento.

## Sequência de testes

Execute cada comando e abra o endereço exibido no terminal:
`http://IP_DO_ROBO:8080/?token=duck-test`. Recarregue a página a cada novo teste.
O token `duck-test` é o padrão preexistente de teste, ajustável com `--token`.

1. Controle web sozinho:

```bash
python web_control_test.py --mode web --duration 120
```

2. Controle web e leitor real da IMU:

```bash
python web_control_test.py --mode imu --i2c-bus 8 --duration 120
```

3. Acrescentar inferência do student:

```bash
python web_control_test.py --mode onnx --i2c-bus 8 --duration 120 --onnx-model /caminho/real/student.onnx
```

4. Acrescentar gravação contínua em lotes:

```bash
python web_control_test.py --mode telemetry --i2c-bus 8 --duration 120 --onnx-model /caminho/real/student.onnx
```

Substitua o caminho do ONNX pelo mesmo arquivo usado na caminhada. Não altere
ganhos, calibração ou o limite de idade entre os testes. Comece sem `--camera`;
se precisar comparar a câmera, repita a mesma etapa com essa opção depois.

Em cada teste, repita aproximadamente a mesma sequência de 120 segundos:

- 0–30 s: navegador conectado, controles centralizados;
- 30–60 s: comandos para frente, soltando e repetindo;
- 60–90 s: comandos de giro, soltando e repetindo;
- 90–120 s: controles centralizados; pressione PARAR e INICIAR para registrar os pedidos.

**INICIAR/PARAR não interrompem a coleta deste diagnóstico.** São registrados
como marcadores e não acionam motores. O teste começa ao terminar a inicialização,
dura 120 segundos e continua após falhas da IMU. Ctrl+C encerra antes do prazo.
O servidor web encerra quando o processo Python termina. O runtime de caminhada
continua com sua proteção e comportamento de pausa inalterados.

## Dados e interpretação

Cada execução cria uma pasta em `web_imu_diagnostics` com:

- `metadata.json`: modo, hashes, barramento, limite, câmera e configuração ONNX;
- `chunk-*.jsonl`: comandos, intervalos dos ciclos, amostras aceitas, etapas das
  rejeições (`get_data` ou `after_inference`), timestamps e resumo;
- `writer_timing.json`: início/fim de cada publicação de lote, incluindo
  serialização JSON, escrita, fsync e atualização de status;
- `status.json`: integridade da gravação e perdas, se ocorrerem.

Nos modos `web`, `imu` e `onnx`, as linhas ficam em memória durante a medição e
são gravadas ao fim. Por isso use `cycle_start_monotonic_ns` e os timestamps
internos para a cronologia; `recorded_monotonic_ns` nessas etapas indica a
gravação posterior. Aguarde a saída do processo para copiar os dados. Interrupção
por Ctrl+C salva as linhas disponíveis; desligamento abrupto pode perder o buffer.
A duração máxima permitida é 300 segundos para limitar o uso de memória.

No modo `telemetry`, as linhas são gravadas continuamente. A carga numérica vem de um ciclo real de caminhada, armazenado no JSON de
referência. Ela é copiada no campo `serialization_load_only` nos modos ONNX e
telemetry, inclusive nos ciclos com falha, e nunca é usada para inferência ou
enviada a motores. Os campos extras de diagnóstico podem aumentar o volume total
em relação à linha de referência. Os dados de juntas,
contatos e histórico da entrada ONNX continuam sintéticos; só IMU e comandos web são reais.
**Este teste mede carga e atrasos, não qualidade da política ou equilíbrio.** Não
reproduz comunicação serial dos servos, ruído elétrico de motores nem carga física.

Uma amostra rejeitada na obtenção não é usada na inferência daquele ciclo. A
coleta continua no seguinte. Falhas do produtor são registradas separadamente,
mesmo quando recuperadas antes de serem vistas pelo consumidor. O registro normal
`IMU: leitura encerrada` é excluído da contagem de falhas. O buffer de eventos do
produtor comporta 20.000 entradas; se encher, o resumo sinaliza cobertura limitada.

Saída 0: sem falhas detectadas; 2: falhas de IMU, execução ou gravação; 130: Ctrl+C.
`status=closed` confirma encerramento da gravação, não aprovação do teste.

Comparação: falha já em `imu` sugere investigar leitor/web/agendamento; aumento
em `onnx` indica associação com a carga da inferência; aumento em `telemetry`
indica associação com a gravação. São pistas, não prova causal em um único ensaio.
Se necessário, repita ONNX e telemetria com `--onnx-threads 1`, mantendo as demais
opções. O padrão 0 conserva a escolha automática de threads do ONNX Runtime.

Envie as quatro pastas completas para análise. Nenhum arquivo original de
telemetria ou configuração é substituído pelo diagnóstico.


## Novo ensaio: carga real de dados e rastreamento dos atrasos

Copie estes **dois arquivos**, preservando a subpasta `data`:

- `scripts/web_control_test.py`
- `scripts/data/runtime_telemetry_payload.json`

Repita primeiro `--mode onnx` e depois `--mode telemetry`, com duração de 120 s,
o mesmo caminho de student, câmera desligada e a mesma sequência de comandos
acima. Faça duas rodadas de cada, se possível, alternando a ordem. Os comandos
são os mesmos da seção anterior. O arquivo de referência é encontrado
automaticamente; `--payload-profile` permite selecionar outro explicitamente.
Não é necessário copiar toda a sessão antiga para o Raspberry.

Esses dois modos agora copiam a mesma carga numérica histórica por ciclo.
No modo ONNX, ela fica no buffer até o fim; no modo telemetry, segue para o
mesmo gravador assíncrono. A alocação do buffer e o custo do gravador ainda
são diferentes; o objetivo é medir essa diferença, não declarar equivalência
perfeita de todas as threads do runtime. Os resultados antigos foram preservados.

Além dos arquivos anteriores, cada sessão gera:

- `gc_timing.json`: linhas com timestamp monotônico, fase (`stop=0` início,
  `stop=1` fim), geração, objetos coletados e não coletáveis. No modo padrão, o coletor não é
  desativado nem tem seus limiares alterados; a opção experimental `defer` é
  descrita abaixo. O rastreamento cobre inicialização
  e medição até o encerramento do leitor; exclui a exportação posterior.
- `i2c_timing.json`: início/fim de cada `write_then_readinto` do leitor, endereço
  do registrador e indicador de exceção. Não faz leituras extras, não muda bytes
  nem contorna a validação. Cobre a aquisição contínua, não a configuração inicial.
  A duração inclui a chamada existente e pode incluir atraso de agendamento;
  não equivale a uma medição elétrica do sinal no fio.

Os dois formatos incluem `columns`, `count`, `dropped` e `rows`. O armazenamento
numérico é pré-alocado para evitar uma lista crescente de objetos de diagnóstico
em cada transação. Ainda há custo de instrumentação; os dois ensaios devem usar
a mesma versão. `dropped > 0` indica cobertura incompleta e deve ser considerado
na análise. Os arquivos são exportados ao encerrar; aguarde a saída do processo.

Envie as pastas completas para cruzarmos falhas, transações I²C, coleta de lixo e
publicações dos lotes. As ações continuam descartadas, os motores não são
inicializados e a proteção do runtime não foi alterada.


## Correções experimentais: comparação A/B/C

Esta etapa testa mudanças no diagnóstico **sem motores**. Elas ainda não foram
aplicadas ao runtime de caminhada. O limite de idade continua em 50 ms.

A nova versão aparece em `--help` com `--gc-policy` e `--writer-yield-ms`.
Copie novamente script e arquivo de perfil antes do ensaio. Use o mesmo student,
a mesma posição do robô, câmera desligada e a mesma sequência de comandos.
Substitua `/caminho/real/student.onnx` pelo caminho que você já utiliza.

**A — Referência com GC e gravador habituais:**

```bash
python web_control_test.py --mode telemetry --duration 120 --onnx-model /caminho/real/student.onnx --gc-policy default --writer-yield-ms 0
```

**B — Adiar GC durante a medição:**

```bash
python web_control_test.py --mode telemetry --duration 120 --onnx-model /caminho/real/student.onnx --gc-policy defer --writer-yield-ms 0
```

**C — Adiar GC e ceder tempo no gravador entre linhas:**

```bash
python web_control_test.py --mode telemetry --duration 120 --onnx-model /caminho/real/student.onnx --gc-policy defer --writer-yield-ms 1
```

Faça A, B e C; depois, se possível, repita na ordem C, B, A para verificar
repetibilidade. Não execute testes simultaneamente. O programa imprime as opções
ativas no início e as registra nos metadados. Aguarde o encerramento completo
antes de começar o próximo. Comandos continuam sendo recebidos e a coleta não
pausa por amostra antiga, pois nenhuma ação é enviada aos motores.

Como funciona `defer`: após inicializar os componentes, faz uma coleta explícita
antes da medição, verifica memória e suspende a coleta automática somente no
intervalo do teste. Confirma novamente a prontidão da IMU antes de iniciar a
medição. Ao encerrar a aquisição, restaura o estado anterior do GC, inclusive em
Ctrl+C ou erro. Essa opção recusa duração acima de 120 segundos e exige as medidas
Linux de `/proc`. Não é uma opção para deixar um runtime de produção indefinidamente
sem coleta de lixo.

O script mede RSS do próprio processo e `MemAvailable` do sistema aproximadamente
uma vez por segundo em **todos** os modos. Os limites padrão encerram a coleta
se o RSS crescer mais de 64 MiB em relação ao início ou se houver menos de 64 MiB
disponíveis no sistema. O encerramento é registrado como erro e preserva os dados
já obtidos. A verificação é periódica, não uma reserva ou um limite imposto pelo
kernel; um pico rápido ainda pode ocorrer entre verificações. Não aumente os
limites apenas para passar no teste sem analisar a memória.

`memory_timing.json` contém os timestamps, RSS e memória disponível. O arquivo
permite observar o custo de adiar o GC. O campo `gc_enabled` do metadata descreve
o estado inicial; `gc_policy` informa a política durante o ensaio. Eventos de GC
na inicialização são esperados no modo `defer`; devem ser separados do intervalo
de ciclos medidos.

`--writer-yield-ms 1` insere uma espera de 1 ms após cada linha serializada na
thread do gravador. Mantém JSON, publicação atômica, fsync, fila limitada e
contadores de perda. Testa uma forma de reduzir a concorrência de CPU com a IMU;
não garante eliminar atrasos e pode reduzir a capacidade de escrita. Só vale no
modo `telemetry` e não altera o gravador utilizado na caminhada.

Para interpretar: comparar rejeições e picos de GC de A para B; comparar atrasos
residuais, fila/perdas e escrita de B para C; conferir a trajetória de memória em
todos. Critério para avançar: ensaios completos e repetidos sem rejeições, sem
perdas, sem acionamento do limite de memória e com aquisições recentes. Um único
teste limpo não valida a caminhada. Envie as pastas completas, incluindo
`memory_timing.json`, `gc_timing.json` e `i2c_timing.json`.
