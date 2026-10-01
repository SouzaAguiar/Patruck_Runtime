# Investigação das interrupções — 30/09/2026

O operador confirmou equilíbrio parado sem apoio da mão na sessão
20260930T225019.741423Z-571b536a. Restam quatro interrupções temporais:
três amostras com mais de 50 ms e uma sem a reserva de 10 ms antes de inferir.
A rotação yaw-minus-90 está ativa. Não alterar limites de idade nesta rodada.

## Preparação

1. Atualize os arquivos do pacote preservando os caminhos, na pasta
   `/home/jonathan/Open_Duck_Mini_Runtime`. O pacote não contém o modelo nem
   a calibração; mantenha os arquivos já usados no Pi.
2. Encerre o runtime e qualquer outro leitor da IMU. A porta 8080 deve estar livre.
3. Ative o ambiente `patruck-runtime` como nos testes anteriores e entre em
   `/home/jonathan/Open_Duck_Mini_Runtime/scripts`. O arquivo
   `imu_calib_data.pkl` deve estar nessa pasta; `~/duck_config.json` deve existir.
4. Mantenha o robô apoiado numa superfície estável, com os motores desativados.
   Este script não importa controle dos motores nem envia ações aos servos.
5. Use a mesma condição web nos três ensaios: uma aba do controle aberta,
   joysticks centralizados e câmera desligada. Não execute ensaios em paralelo.

## Três ensaios de 60 segundos

Execute no terminal do Pi. Aguarde cada comando terminar antes do seguinte.
Abra `http://IP_DO_PI:8080` quando cada servidor iniciar. O token padrão é
`duck-test`. INICIAR/PARAR não aciona motores nem encerra esta medição.

### A — IMU e controle web

```bash
python web_control_test.py --mode imu --duration 60 --i2c-bus 8 --max-age-ms 50 --imu-frame yaw-minus-90 --gc-policy defer --output ../web_imu_diagnostics
```

### B — Adicionar inferência do teacher

Informe o caminho real do mesmo ONNX teacher usado no último runtime.
O rótulo da sessão não seleciona o modelo. O hash esperado desse teacher é
`3c606f9381a1710cc8fecdb7442787dcbfce3ee9bc02a6f1224774ab2b3a1067`.

```bash
read -r -p "Cole o caminho do ONNX teacher usado no ultimo runtime: " TEACHER_ONNX
sha256sum "$TEACHER_ONNX"
python web_control_test.py --mode onnx --duration 60 --i2c-bus 8 --max-age-ms 50 --imu-frame yaw-minus-90 --gc-policy defer --onnx-model "$TEACHER_ONNX" --onnx-threads 0 --output ../web_imu_diagnostics
```

### C — Adicionar escrita contínua de telemetria

No mesmo terminal, usando a variável definida acima:

```bash
python web_control_test.py --mode telemetry --duration 60 --i2c-bus 8 --max-age-ms 50 --imu-frame yaw-minus-90 --gc-policy defer --onnx-model "$TEACHER_ONNX" --onnx-threads 0 --writer-yield-ms 1 --output ../web_imu_diagnostics
```

A e B acumulam os registros e gravam depois da medição; C grava continuamente
com um payload semelhante ao runtime. Os três mantêm GC suspenso somente
durante a janela limitada, com proteção de memória e restauração ao sair.
Não habilite `--camera` nem altere threads entre estes ensaios.

## Dados e interpretação

Copie as três pastas novas de `web_imu_diagnostics` para o checkout local,
incluindo metadata, status, chunks e os arquivos i2c_timing.json,
gc_timing.json, memory_timing.json e writer_timing.json. Não envie apenas o resumo.
O código de saída 2 pode indicar falhas de amostra registradas durante um
ensaio concluído; preserve os dados mesmo assim. Ctrl+C encerra antecipadamente.

- Falhas já em A: investigar leitura I²C e escalonamento do produtor sem a
  carga do ONNX ou da escrita contínua. Isso sozinho não prova defeito elétrico.
- Falhas acrescentadas em B: verificar possível contenção da inferência;
  uma comparação com uma thread será o passo seguinte se os dados justificarem.
- Falhas acrescentadas em C: correlacionar atrasos com escrita da telemetria.
- Todos limpos: investigar a sequência do loop completo e leitura das juntas.

O diagnóstico usa IMU e modelo reais, mas observações sintéticas para juntas
e histórico; não reproduz todo o runtime e não avalia qualidade de marcha.
Uma rodada sem falhas não exclui uma interrupção rara. A análise deve comparar
intervalos de publicação, duração das transações e eventos por timestamp.
Não aumentar 50 ms, remover proteção nem iniciar marcha para esta investigação.
