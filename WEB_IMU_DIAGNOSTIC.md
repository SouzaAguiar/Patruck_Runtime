# Diagnóstico web + IMU sem motores

O `scripts/web_control_test.py` agora oferece quatro modos independentes. Nenhum
importa HWI, o script de caminhada ou drivers de motores. As saídas do ONNX são
descartadas. A câmera só é inicializada com `--camera`.

Copie o `scripts/web_control_test.py` atualizado para o Raspberry. Ele usa os
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

No modo `telemetry`, as linhas são gravadas continuamente. O volume é aproximado
ao da caminhada: inclui observações, ações e vetores sintéticos. Os dados de juntas,
contatos e histórico do ONNX são sintéticos; só IMU e comandos web são reais.
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
