# Um teste continuo para esclarecer os sinais da IMU

Os dados existentes permitem continuar sem abrir o robo ou identificar agora
onde a IMU esta instalada. Esta coleta registra manobras e sinais na mesma
sessao, sem reinicializar/configurar o sensor entre as manobras.

Extraia `patruck-imu-guided-test.zip` na raiz
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas. Encerre runtime
e outros leitores da IMU. Sustente o robo e mantenha os servos sem alimentacao.
O script inicializa somente a IMU; nao importa HWI nem carrega ONNX.

Com o ambiente patruck-runtime ativo:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --guided-frame-test --output ../imu_diagnostics --label frame-v2-guided
```

Use yaw-plus-90 como preview para comparar com os testes anteriores. Os bytes
e vetores originais continuam preservados. Isso nao habilita a caminhada.

Leia a sequencia antes. O script pede Enter, prepara a IMU e depois mostra
COLETA INICIADA. A coleta tem 95 segundos: 5 segundos de repouso inicial e duas
rodadas de tres manobras. Cada manobra tem cinco fases de 3 segundos:

1. ready: nivelado, parado; a instrucao avisa qual sera o proximo movimento.
2. outbound: faca o movimento solicitado lentamente.
3. hold: mantenha a posicao atingida.
4. return: volte lentamente a posicao inicial.
5. rest: permaneça nivelado, parado.

As manobras, sempre com o CORPO INTEIRO, sao:

- pitch: baixar a frente; uma inclinacao pequena, cerca de 10–15 graus.
- roll: baixar o lado direito DO ROBO; cerca de 10–15 graus.
- yaw: virar para a esquerda DO ROBO, anti-horario visto de cima, mantendo
  o corpo nivelado; aproximadamente 20 graus.

Nao mova apenas a cabeca e nao force as juntas. Uma segunda pessoa pode ler
as instrucoes do terminal. O terminal tenta emitir um sinal sonoro por fase,
mas alguns terminais nao o reproduzem: acompanhe o texto.

Nao pressione Enter entre fases: elas avancam automaticamente enquanto a IMU
continua sendo lida. Se nao conseguir acompanhar, Ctrl+C encerra preservando
os dados e execute novamente para uma nova sessao. Nao ha retomada automatica.

Todas as fases ficam em guided_stage, com horarios na telemetria; cada amostra
identifica a fase. Esses marcadores sao as instrucoes emitidas, nao deteccao
automatica da manobra realizada. Informe qualquer movimento invertido ou
atrasado. O roteiro e a duracao total ficam na metadata. Video e opcional;
sem marco comum nao ha sincronizacao automatica entre video e telemetria.

Copie a nova pasta inteira de imu_diagnostics para o checkout Windows e informe
se conseguiu seguir a sequencia. Nao precisamos repetir os contatos dos pes
nem ajustar modelo, ganhos, calibracao ou a transformacao nesta rodada.
