# Coleta de telemetria no robô

A telemetria é opcional no script de caminhada existente. Não altera as ações,
os offsets, a frequência configurada, os ganhos ou a forma de comandar o robô.
O script continua sendo um controlador real: executá-lo inicializa os motores
da mesma forma que antes. Não há movimento de teste automático.

## Atualização e início

Copie para o Raspberry Pi os arquivos atualizados deste projeto:

- `scripts/v2_rl_walk_mujoco.py`
- `scripts/summarize_telemetry.py`
- `mini_bdx_runtime/mini_bdx_runtime/telemetry.py`
- `mini_bdx_runtime/mini_bdx_runtime/raw_imu.py`
- `mini_bdx_runtime/mini_bdx_runtime/web_controller.py`

Se o pacote do runtime não estiver instalado em modo editável, reinstale-o na
raiz do projeto usando o mesmo ambiente Python do controle:

```bash
python -m pip install --no-deps -e .
```

A partir da pasta `scripts`, use seu comando habitual, acrescentando
`--telemetry-dir ../telemetry` e um nome descritivo para o ensaio. Exemplo:

```bash
python v2_rl_walk_mujoco.py --onnx_model_path ../models/student.onnx --control-source web --telemetry-dir ../telemetry --telemetry-label student-frente-piso-a
```

O modelo precisa existir nesse caminho. Preserve os demais argumentos que você
já usa, como `--control-source web`, configuração, porta serial e parâmetros do
controle. Para comparar o teacher, altere o caminho do modelo e o rótulo do ensaio.
Abra o endereço do controle web mostrado no terminal, como de costume. A coleta
acompanha os comandos do navegador; não requer controle Xbox nem outro painel.
Não use `--replay_obs` para estes ensaios físicos: ele substitui a entrada dos
sensores por observações previamente gravadas. Não é necessário `--save_obs`.

A pasta exata da sessão aparece no terminal. Cada execução cria uma pasta nova
com data UTC e identificador; nenhuma sessão anterior é sobrescrita.

## Pausa e encerramento

Use a pausa habitual do controle (botão A ou controle web) entre movimentos.
Durante a pausa são registrados comandos e eventos a 10 Hz; não são feitas
leituras adicionais dos motores nem inferência. A pausa mantém o comportamento
anterior do runtime e não desliga o torque dos servos.

Ctrl+C encerra o controle e solicita a gravação dos dados pendentes. Isso também
mantém o comportamento anterior do runtime quanto aos motores; não equivale a
um comando de desligar torque. Em exceções, o gravador tenta finalizar a sessão.
Queda de energia ou encerramento forçado pode perder o bloco ainda em memória
e a fila pendente; os blocos já publicados permanecem disponíveis.

Comece com registros curtos da postura parada e dos primeiros passos à frente,
com apoio que impeça a queda. Anote quando houve apoio externo e quando ele foi
retirado, pois isso muda a interpretação da física. Um vídeo e o horário do
ensaio ajudam a identificar tropeços ou quedas, que não são classificados
automaticamente pelo gravador.

## Arquivos

- `metadata.json`: identificação do modelo por SHA256, rótulo, versão de Python,
  plataforma, parâmetros solicitados e vínculo entre relógio UTC e monotônico.
- `chunk-000000.jsonl`, etc.: registros em JSON, um por linha. A escrita ocorre
  em segundo plano; a cada segundo ou 256 registros o bloco é sincronizado e
  renomeado de forma atômica. O intervalo é uma meta, não uma garantia contra
  atraso do armazenamento.
- `status.json`: contagens gravadas/perdidas, estado do gravador e erros.
  `closed` indica finalização; `closed_with_loss` indica perda por fila cheia.
  `failed` indica erro de escrita. Um estado `recording` após o processo encerrar
  indica que a finalização não foi confirmada.
- Arquivos `.tmp` são blocos incompletos, ignorados pelo resumo.

A fila tem tamanho limitado. Se o cartão não acompanhar a gravação, o controle
não espera a escrita: descarta registros, avisa no terminal e contabiliza perdas.
Erro de disco é avisado e não muda automaticamente o comando dos motores.

## O que cada ciclo registra

| Campo | Conteúdo |
|---|---|
| `commands` | 7 comandos: velocidades X/Y/giro e 4 comandos da cabeça |
| `command_source` | Conexão web, idade do último comando e indicação de timeout, referentes ao comando consumido |
| `observed_obs` | 101 valores montados a partir dos sensores reais |
| `policy_obs` | Entrada efetiva da rede; igual à anterior fora de replay |
| `sensors` | Giroscópio, acelerômetro, posições/velocidades articulares, contatos e tempos das leituras |
| `action` | 14 saídas originais do ONNX |
| `unfiltered_targets_rad` | Pose neutra mais ação escalada, antes do filtro e comandos da cabeça |
| `motor_targets_rad` | Alvos finais enviados à interface dos motores, antes dos offsets |
| `servo_goal_rad` | Alvos acrescidos dos offsets, na ordem real dos IDs dos servos |
| `inference_ms`, `motor_write_ms` | Tempos da inferência e chamada de envio aos motores |
| `previous_cycle_dt_ms` | Intervalo real entre os inícios dos ciclos, incluindo espera |
| `loop_work_before_logging_ms` | Trabalho do ciclo até terminar o envio, antes de enfileirar a telemetria |

O evento `runtime_ready` registra ordem/nome/ID dos motores, pose neutra,
offsets, ganhos P/D efetivos, parâmetros de fase e hashes da calibração da IMU
e da referência de movimento. Tokens web não são registrados.

As leituras são sequenciais, não simultâneas. Seus horários são registrados no
mesmo relógio monotônico. A IMU tem índice e horário de aquisição, permitindo
identificar amostras repetidas/antigas; valores ausentes aparecem como `null`.
A primeira amostra pode ainda não ter horário de aquisição. O timestamp indica
a leitura feita pelo software, não um relógio interno do sensor.

Posições retornadas pelo HWI já têm os offsets retirados e arredondamento de
0,001 rad. O giroscópio usa rad/s; aceleração usa m/s² e inclui gravidade. As
velocidades articulares usam a unidade reportada como rad/s pelo HWI existente.
Conclusão da chamada de envio não comprova que o servo alcançou a posição.

O caminho atual `raw_imu.py` não fornece orientação absoluta. O argumento
`pitch_bias` solicitado fica documentado, mas o driver atual não o aplica aos
dados: a telemetria não muda isso. Não são medidos velocidade/deslocamento no
chão, corrente, torque ou tensão. Não se deve inferir velocidade real apenas
integrando o acelerômetro destes registros.

## Conferir e compartilhar

```bash
python summarize_telemetry.py ../telemetry/NOME_DA_SESSAO
```

O resumo funciona também no computador, sem dependências de hardware. Mostra
quantidade de ciclos, perdas, tempos, idade da IMU, faixa de comandos e erro por
articulação em relação ao alvo do ciclo anterior. Esse erro inclui o atraso
normal dos servos e não é, sozinho, diagnóstico de defeito.
Também contabiliza os ciclos com comando web válido, expirado ou desconectado.

Para análise, copie a pasta completa da sessão e informe modelo utilizado,
superfície, presença de apoio e comportamento observado. Mantenha as sessões do
teacher e do student identificadas separadamente. A coleta não treina um
adaptador; fornece evidência para calibrar a simulação e planejar esse treino.
