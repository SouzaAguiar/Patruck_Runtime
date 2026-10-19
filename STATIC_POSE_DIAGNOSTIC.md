# Diagnóstico estático da postura home

`scripts/diagnose_static_pose.py` mede a postura inicial do runtime, sem carregar
ONNX, caminhada, fases, trajetórias ou controlador web. Reutiliza `DuckConfig`,
`HWI`, `Imu`, `imu_frame`, `FeetContacts` e `TelemetryRecorder`. O único comando
de posição é o de `HWI.turn_on()`, na preparação. Os módulos compartilhados e o
script de caminhada não são alterados.

## Preparação e execução no Raspberry Pi

1. Use o ambiente Python já instalado para o runtime. Encerre caminhada, controle
   web e outros processos que usem serial, IMU ou GPIO.
2. Confira `~/duck_config.json` e `scripts/imu_calib_data.pkl` do robô. Não use os
   offsets do arquivo de exemplo para uma execução física. Não há recalibração.
3. Deixe área livre, alimentação e apoio adequados. Proteja o robô contra quedas
   durante o posicionamento. O home pode movimentar as 14 juntas, inclusive a
   cabeça; a pausa posterior não garante imobilidade nem segurança mecânica.
4. Execute primeiro a validação sem hardware:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_static_pose.py \
  --support feet --config ~/duck_config.json \
  --duration 10 --frequency 50 --settle-seconds 3 \
  --imu-i2c-bus 8 --imu-frame yaw-minus-90 \
  --serial-port /dev/ttyACM0 --output ../telemetry \
  --label postura-estatica-pes --dry-run
```

O dry-run confere argumentos, configuração, conteúdo da calibração e destino,
mostra home/offsets/procedimento e termina sem importar drivers, abrir IMU,
serial ou GPIO, energizar motores ou criar sessão de telemetria. Não comprova
conectividade, permissões dos dispositivos ou capacidade de sustentação.

5. Com o robô protegido, execute a coleta:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_static_pose.py \
  --support feet --config ~/duck_config.json \
  --duration 10 --frequency 50 --settle-seconds 3 \
  --imu-i2c-bus 8 --imu-frame yaw-minus-90 \
  --serial-port /dev/ttyACM0 --output ../telemetry \
  --label postura-estatica-pes
```

O script inicializa os leitores e pede que você digite **ENERGIZAR** antes de
chamar `HWI.turn_on()`. Outra resposta cancela. Após o home, aguarda 3 segundos
sem enviar posições, verifica as velocidades e começa a coleta de 10 segundos.
O operador deve manter a condição de apoio declarada durante a medição.

Use `--support stand --label postura-estatica-suporte` se estiver sustentado por
suporte. `--support` é obrigatório e apenas identifica a condição; não muda o
home, o torque, o tratamento dos contatos ou os critérios de movimento.

Ctrl+C preserva as amostras concluídas e gera resumo parcial. Erros de leitura
abortam sem tentativa de recuperar o robô com novos movimentos. **O diagnóstico
não desliga o torque. Assegure o apoio antes de desligar os servos**, inclusive
após erro no posicionamento inicial. Não conte com o script para segurar ou
parar mecanicamente uma queda.

## Parâmetros e limites

| Opção | Padrão | Validação |
|---|---|---|
| `--duration` | 10 s | finito, >0 e ≤600; janela com ≥1 amostra nominal |
| `--frequency` | 50 Hz | finito, >0 e ≤100 |
| `--settle-seconds` | 3 s | finito, entre 0 e 60 |
| `--imu-i2c-bus` | 8 | inteiro entre 0 e 255; sem fallback |
| `--imu-frame` | yaw-minus-90 | native, yaw-plus-90 ou yaw-minus-90 |
| `--config` | ~/duck_config.json | arquivo JSON existente |
| `--serial-port` | /dev/ttyACM0 | identificador não vazio |
| `--output` | ../telemetry | relativo ao diretório de execução; ancestral gravável |
| `--label` | static-pose-feet | 1 a 128 caracteres imprimíveis |
| `--support` | obrigatório | feet ou stand |
| `--dry-run` | desativado | valida sem hardware |

Os limites de duração/frequência limitam o volume em memória. Offsets, quando
presentes, precisam definir todas as 14 juntas, com valores finitos em radianos
entre −2π e +2π. Se a chave inteira estiver ausente, `DuckConfig` usa zeros;
o dry-run mostra esses valores para conferência. O home vem de `HWI.init_pos`;
a configuração fornece os offsets, não uma postura inventada pelo diagnóstico.

O preflight lê os dicionários literais do próprio HWI sem importar o driver;
antes de energizar, compara-os com a instância real. Se o formato do HWI mudar,
a validação deve ser revisada. A calibração é sempre a da pasta `scripts/`,
independentemente do diretório de lançamento: apenas durante a construção da
IMU o script ajusta o diretório atual, restaurando-o em seguida.

## Arquivos e tempo das leituras

Cada execução física cria `telemetry/<UTC>-<identificador>/` contendo:

- `metadata.json`: identificação/data, parâmetros, hashes SHA-256 dos arquivos,
  ordem/IDs, home, offsets, eixos e condição de apoio em `settings`; âncora de
  relógio e informações do gravador no formato padrão de `TelemetryRecorder`.
  `initial_motion_attempted` identifica tentativa de ativação e
  `initial_motion_commanded` indica retorno bem-sucedido de `turn_on()`. Se a
  ativação falhar, movimentos parciais e torque ainda são possíveis.
- `chunk-*.jsonl`: eventos `static_pose_preparation`, `static_pose_settled`,
  `static_pose_start`, `static_pose_sample`, `static_pose_end` e, quando
  pertinente, `static_pose_failure`/`static_pose_cancelled`. Cada amostra inclui
  vetores, identidade e idade da IMU, posições/velocidades/offsets/home, contatos,
  tempos e motivo de rejeição. Valores não finitos tornam-se `null` pelo gravador.
- `summary.json`: contagens, frequências total/válida, cobertura nominal,
  estatísticas de aceleração/gyro/ângulos, dados dos motores, grupos por lado e
  função, contagens de contatos, estabilidade, falhas e comparação com referência.
- `status.json`: término e perdas/erros do gravador; confira também este arquivo.
- `writer_timing.json`: tempos de publicação dos blocos de telemetria.

As leituras são **sequenciais**, com timestamps separados da consulta IMU,
posições, velocidades e contatos. O produtor IMU tem seu próprio intervalo
`sample_start_monotonic_ns`/`sample_end_monotonic_ns`; a consulta lê seu cache,
não é uma nova medição I²C. O produtor lê gyro e aceleração em sequência.
Todos os tempos usam a mesma base `monotonic_ns`. `imu_read_ms` é o custo da
consulta ao leitor; o intervalo do produtor representa a aquisição da IMU.
`joints_read_ms` cobre as duas consultas seriais. Não há sincronismo físico
entre sensores ou entre os 14 servos.

A frequência é um alvo: atrasos não provocam rajadas de leituras para compensar.
A IMU produz a pelo menos 50 Hz mesmo quando a coleta solicitada é mais lenta.
Amostras repetidas ficam nos dados como rejeitadas, sem inflar as estatísticas
válidas. Não são eliminadas silenciosamente.

## Interpretação

`native` é o referencial após o remapeamento BNO055 existente, não os eixos do
chip antes desse remapeamento. A rotação existente `yaw-minus-90` fornece
`x=native_y`, `y=-native_x`, `z=native_z`. O diagnóstico nunca gira novamente um
vetor já girado: usa os vetores native preservados pelo leitor para as projeções.
Mesmo ao selecionar outro `--imu-frame`, os ângulos principais e a comparação
histórica usam explicitamente a projeção yaw-minus-90 salva nos dados.

Com gravidade predominante:

```python
pitch_deg = degrees(atan2(x, hypot(y, z)))
roll_deg = degrees(atan2(y, hypot(x, z)))
tilt_deg = degrees(atan2(hypot(x, y), abs(z)))
```

Pitch positivo acompanha X corrigido positivo; roll positivo acompanha Y
corrigido positivo. Pela convenção dos ensaios guiados existente no código,
inclinar o nariz para baixo corresponde a pitch negativo e baixar o lado
direito corresponde a roll positivo. A montagem física precisa corresponder à
configuração do robô. A inclinação total usa `abs(z)` e não distingue postura
invertida; não é um detector de queda.

`principal_angles_from_mean_acceleration` usa o **vetor médio**, enquanto
`angles_deg` descreve média, desvio-padrão populacional, mínimo e máximo dos
ângulos individuais. Referência nivelada fornecida para 19/10/2026: pitch
aproximado +0,447°, roll +0,106°. As diferenças são apenas comparação; nenhuma
correção ou tara é aplicada. Não há valores importados automaticamente dos
ensaios de 13/10: compare-os depois com a pasta completa e o mesmo referencial.

`logical_position_rad` já tem o offset subtraído pelo HWI.
`raw_estimated_position_rad = logical_position_rad + configured_offset_rad`.
O bruto é **estimado**, pois o HWI arredonda a posição lógica a 0,001 rad.
`measured_minus_home_rad` compara a leitura ao home sem adicionar offsets.
O arquivo bruto preserva os 14 motores na ordem original; `leg_groups` organiza
quadril yaw/roll/pitch, joelho e tornozelo por lado. Sinais de juntas espelhadas
não são automaticamente erros mecânicos. Não há normalização cinemática dos
lados nem soma de juntas convertida em orientação absoluta do pé.

## Validade e limitações

- Vetores incompletos/não finitos, identidade/referencial inválido, timestamps
  inválidos, IMU com mais de 50 ms e falhas de comunicação rejeitam a amostra e
  abortam. A idade é contada desde o início da aquisição IMU e novamente
  verificada após as consultas seriais/GPIO. IDs repetidos/fora de ordem são
  rejeitados e a coleta continua, preservando as leituras disponíveis.
- `posture_stable=false` quando alguma velocidade absoluta excede **0,05 rad/s**
  na verificação após estabilização ou na coleta, ou quando a amplitude de
  posição de alguma junta excede **0,02 rad** na coleta. Leituras articulares
  completas de ciclos rejeitados também entram nessa verificação e nas
  estatísticas articulares. Esses limiares são triagem, não garantia mecânica.
- `validity_passed` exige conclusão, nenhuma amostra rejeitada, pelo menos 80%
  da quantidade nominal de amostras válidas, inclinação calculável, estabilidade
  articular e ausência de falha/perda de gravação. `posture_stable=null` significa
  ausência de leituras articulares completas. Amostras instáveis não são apagadas.
- Contatos são contados como esquerdo/direito, ativos/inativos; a condição de
  apoio não modifica automaticamente sua validade. Confira se correspondem ao
  apoio declarado. Ausência de velocidade articular não exclui movimento do
  corpo inteiro, vibração ou aceleração linear; avalie gyro, contatos e dispersão.
- Códigos de saída: 0 coleta concluída (confira a validade no resumo), 1 falha,
  2 cancelamento/argumentos inválidos e 130 interrupção. Os dados anteriores são
  preservados em blocos e resumo, sujeitos a disponibilidade do armazenamento.
  Não há retomada automática de sessão nem recuperação com movimentos.
- HWI não expõe método de fechar a serial; sua referência é liberada ao encerrar,
  sem `turn_off()` ou `disable_torque()`. Leitores GPIO/IMU são encerrados pelos
  métodos existentes; a inicialização GPIO parcial libera os pinos já alocados.
  Se a thread IMU continuar ativa após `stop()`, o resumo registra a falha de
  encerramento. Não houve validação física deste diagnóstico no
  desenvolvimento; testes com fakes não comprovam equilíbrio ou precisão real.

Testes sem hardware, a partir da raiz do repositório:

```bash
python -m pytest tests/test_static_pose_diagnostic.py -q
python -m pytest tests -q
```
