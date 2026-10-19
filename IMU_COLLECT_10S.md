# Coletar a IMU por 10 segundos

No Raspberry, use o ambiente Python do runtime e mantenha
`scripts/collect_imu_10s.py` ao lado de `scripts/diagnose_imu.py`.
O coletor reutiliza esse diagnostico e os modulos existentes do runtime.

Encerre a caminhada e outros leitores da IMU: o diagnostico inicializa e
reconfigura o BNO055. Mantenha o robo apoiado, com motores sem alimentacao
e Raspberry/IMU alimentados. O script nao controla motores.

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime
python scripts/collect_imu_10s.py --telemetry-dir ./imu_diagnostics --telemetry-label parado-10s
```

A janela de aquisicao dura 10 segundos, apos a inicializacao do sensor.
A frequencia solicitada e 50 Hz, com aproximadamente 500 leituras se o
barramento acompanhar essa taxa. A inicializacao e a gravacao final acrescentam
tempo a execucao total. Ctrl+C interrompe a coleta.

Usa BNO055 no endereco `0x29`, barramento Linux 8, orientacao de
`~/duck_config.json` e calibracao de `scripts/imu_calib_data.pkl`.
Os arquivos precisam ser os mesmos usados pelo runtime.
Para selecionar outro barramento ou caminhos:

```bash
python scripts/collect_imu_10s.py --i2c-bus 1 --config /caminho/duck_config.json --calibration /caminho/imu_calib_data.pkl --telemetry-dir /caminho/coletas --telemetry-label bancada-10s
```

`--telemetry-dir` seleciona a pasta de destino e `--telemetry-label` identifica
a sessao. Os nomes anteriores `--output` e `--label` continuam aceitos como
aliases. Sem essas opcoes, usa `imu_diagnostics/` na raiz do runtime e o rotulo
`coleta-10s`.

Cada execucao cria uma pasta exclusiva dentro do destino selecionado e imprime o
caminho de `summary.json`. Ela contem `metadata.json`, `status.json`,
`summary.json` e `chunk-*.jsonl`, com aceleracao (m/s2), giroscopio (rad/s),
tempos, bytes das transacoes e erros. As releituras de anomalias ficam
identificadas separadamente. Consulte tambem [IMU_DIAGNOSTIC.md](IMU_DIAGNOSTIC.md).

Validacao local: interface de linha de comando e encaminhamento dos parametros,
sem acessar a IMU fisica. A coleta real precisa ser executada no Raspberry.
