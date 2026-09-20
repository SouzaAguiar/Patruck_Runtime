# Investigação da IMU do Patruck

O primeiro teste mostrou 24 amostras extremas chegando ao modelo. O objetivo
agora é identificar se os bytes recebidos já contêm o erro ou se a conversão
no software produz um valor incorreto. Nenhuma correção é aplicada aos dados.

## Preparação

1. Copie `scripts/diagnose_imu.py` para a mesma pasta no Raspberry. Ele utiliza
   `mini_bdx_runtime/telemetry.py`, já instalado para a coleta anterior.
2. Encerre a caminhada e quaisquer outros programas que leiam a IMU. Não rode
   este diagnóstico simultaneamente com o controle web de caminhada. O script
   inicializa/reconfigura o BNO055, assim como o driver habitual.
3. Deixe o robô apoiado e parado, com a alimentação dos motores desligada para
   o primeiro teste, mantendo Raspberry e IMU alimentados. Desabilitar torque
   por software não é equivalente a retirar a alimentação dos motores; anote
   qual condição foi possível na sua montagem. Este script não altera torque,
   não movimenta motores e não determina sozinho se eles estão energizados.
4. Use o mesmo ambiente Python do runtime. Não altere cabos, alimentação,
   calibração ou velocidade I²C entre as repetições iniciais.

## Primeiro ensaio: dois minutos parado

Na pasta `scripts`:

```bash
python diagnose_imu.py --duration 120 --label parado-motores-desligados
```

Os padrões são endereço `0x29`, 50 Hz, modo NDOF, orientação lida de
`~/duck_config.json` e calibração `scripts/imu_calib_data.pkl`. São os parâmetros
correspondentes ao caminho de aquisição do runtime. O script exige os arquivos
para evitar usar uma configuração diferente silenciosamente. Se os seus estão
em outro local, informe-os:

```bash
python diagnose_imu.py --duration 120 --label parado-motores-desligados --config /caminho/duck_config.json --calibration /caminho/imu_calib_data.pkl
```

Use somente o arquivo de calibração local confiável já usado pelo seu runtime.
Não use `--no-calibration` como atalho para arquivo ausente: ele representa outro
ensaio, que deve ser identificado e comparado separadamente.

A cada dez segundos, o terminal mostra contagens de amostras e anomalias. O
processo termina sozinho após dois minutos de aquisição e imprime o caminho de
`summary.json`. Ctrl+C também finaliza os blocos pendentes. Cada execução cria
uma pasta exclusiva dentro de `../imu_diagnostics`.

Repita uma vez na mesma condição, mudando apenas o rótulo:

```bash
python diagnose_imu.py --duration 120 --label parado-motores-desligados-repeticao
```

## O que enviar

Copie as duas pastas completas: `metadata.json`, `status.json`, `summary.json` e
todos os `chunk-*.jsonl`. Informe:

- Modelo do Raspberry e do módulo BNO055, se souber.
- Como Raspberry, sensor e motores são alimentados; se compartilham a fonte.
- Comprimento aproximado dos fios SDA/SCL e se há conectores ou protoboard.
- Outros dispositivos ligados ao mesmo barramento I²C.
- Se o robô ficou imóvel e se os motores estavam sem alimentação ou apenas sem torque.

Não é necessário testar caminhada nesta etapa. Não compre outro sensor nem
recalibre antes de comparar os resultados.

## Como interpretar

| Resultado | O que permite concluir |
|---|---|
| `samples` próximo de 6.000 e `dropped_records=0` | Coleta aproximadamente a 50 Hz, sem descarte pelo gravador |
| `acceleration_extreme_samples` ou `gyro_extreme_samples` maior que zero | O problema reapareceu no diagnóstico isolado |
| Extremo com `conversion_mismatch=0` e bytes brutos presentes | O valor extremo já estava no buffer recebido; a conversão registrada corresponde aos bytes |
| `conversion_mismatch` maior que zero | Bytes e resultado do driver discordam; investigar conversão ou compatibilidade da instrumentação |
| `samples_with_io_error` maior que zero | Houve falha explícita de leitura do barramento |
| `missing_raw_capture` maior que zero | A instrumentação não capturou essa leitura; não concluir nada sobre bytes versus conversão |
| Releitura imediatamente normal | Intermitência, não prova de que o sensor estava correto no instante anterior |
| Nenhum extremo | O erro não apareceu nesta condição; não exclui falha intermitente ou dependente da carga |

O giroscópio acima de 20 rad/s é marcado como extremo para este diagnóstico,
não como limite físico especificado do sensor. O acelerômetro acima de 160 m/s²
ultrapassa até a maior faixa nominal de ±16 g do BNO055.

Para rever o resumo em qualquer computador, sem acessar hardware:

```bash
python diagnose_imu.py --summarize ../imu_diagnostics/NOME_DA_SESSAO
```

## Como os bytes são obtidos

O script utiliza `sensor.gyro` seguido de `sensor.acceleration`, como no runtime,
e intercepta a operação `write_then_readinto` do próprio driver. Grava os bytes
da mesma transação, os inteiros com sinal, os valores convertidos, os horários
e as exceções. Não usa uma leitura posterior como se fosse a leitura original.

Quando detecta uma anomalia, faz uma releitura, claramente separada no registro.
Essa transação adicional pode alterar o ritmo naquele ciclo e é incluída no
tempo de aquisição. `--no-reread` permite desativá-la em um ensaio comparativo.

Também registra versão/hash do driver, hashes da configuração/calibração,
identificação do chip, unidades, modo, mapeamento de eixos e estado de calibração
nos registradores. Quando disponível, lê o modelo do Raspberry e os clocks I²C
do device tree; esses valores não são uma medição elétrica do barramento.
A hipótese de inversão do bit de sinal aparece apenas como
diagnóstico; o valor original permanece intacto.

## Etapas seguintes, conforme o resultado

Se o erro aparecer com os motores desligados, priorizar conexões, alimentação
da IMU, barramento e driver. Se aparecer apenas sob carga, comparar alimentação
e interferência com o robô apoiado, uma variável por vez. Ainda será necessário
investigar hardware e comunicação para separar essas causas.

Uma comparação adicional possível, após a linha de base, é usar `--mode imuplus`
ou `--mode accgyro`, preservando o restante da configuração. Isso muda o modo
de operação/fusão, portanto deve ser tratado como outro ensaio. O padrão NDOF
permanece o primeiro teste.

A Adafruit documenta problemas do BNO055 com clock stretching e algumas
implementações I²C. Isso é uma hipótese a verificar na montagem, não prova da
causa desta sessão. Não foi alterada a configuração I²C do Raspberry.

Referências: [Adafruit — I²C e clock stretching no Raspberry](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching),
[driver 5.4.13](https://docs.circuitpython.org/projects/bno055/en/5.4.13/_modules/adafruit_bno055.html),
[datasheet Bosch](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
