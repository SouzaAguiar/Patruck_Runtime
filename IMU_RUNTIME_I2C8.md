# Integração do barramento 8 no runtime com controle web

A integração está implementada no projeto local. A caminhada usa agora o
barramento Linux **8 por padrão**, sem fallback para o barramento 1. Os testes
automáticos não acessam motores nem a IMU física. Ainda é necessário atualizar
o Raspberry e validar a versão instalada.

## O que mudou

- `raw_imu.py` abre o barramento explicitamente por `ExtendedI2C` e verifica
  configuração, modo NDOF, vetores e tempos das amostras.
- O produtor mantém a amostra mais recente, sem esperar o consumidor. A pausa
  não acumula uma amostra antiga na fila.
- A inicialização exige três amostras válidas recentes antes de configurar ou
  enviar a pose inicial aos motores; a prontidão é conferida novamente no início
  de `run()`.
- Dados ausentes, não finitos, com modo inválido ou antigos provocam pausa da
  política. A idade inclui o tempo dentro da chamada I²C, desde o início da
  aquisição, e é conferida após leituras articulares e antes de enviar o alvo.
- Uma ação bloqueada por esse teste não entra no histórico de alvos/ações
  executados nem avança a fase. As saídas no caminho sem falhas foram comparadas
  com o runtime anterior usando sensores e motores simulados em memória.
- O controle web mostra a pausa por IMU. A recuperação exige ação do operador,
  controles centralizados e três leituras válidas consecutivas recentes.

O limite padrão é **50 ms**, configurável por `--imu-max-age-ms`. É uma escolha
conservadora para o ensaio, não um limite de estabilidade mecânica validado.
Os quatro atrasos de aproximadamente 75 ms encontrados anteriormente podem
provocar pausas: o comportamento é intencional. Não aumente esse limite apenas
para esconder atrasos sem avaliar a telemetria.

Os limites de plausibilidade são 160 m/s² no acelerômetro e 35 rad/s no
giroscópio. São verificações amplas; não detectam toda corrupção pequena e não
constituem calibração. Nenhum vetor é suavizado ou tem bits corrigidos.

## Atualizar o Raspberry

Mantenha o overlay `i2c-gpio` já testado no barramento 8 e o controlador I²C1
desabilitado nos GPIO2/3. Encerre o runtime e o diagnóstico antes da atualização.
Faça uma cópia dos arquivos atuais para poder reverter a versão do código.

Copie os seguintes arquivos mantendo a estrutura, a partir da raiz do projeto:

```text
scripts/v2_rl_walk_mujoco.py
scripts/check_runtime_imu.py
mini_bdx_runtime/mini_bdx_runtime/raw_imu.py
mini_bdx_runtime/mini_bdx_runtime/imu_safety.py
mini_bdx_runtime/mini_bdx_runtime/web_controller.py
mini_bdx_runtime/mini_bdx_runtime/web/control.js
mini_bdx_runtime/mini_bdx_runtime/web/index.html
setup.cfg
```

O pacote de atualização também inclui `telemetry.py`, usado pelo verificador,
e este guia. Não substitui `duck_config.json`, calibração, modelos ONNX ou boot.
Pode ser extraído na raiz `/home/jonathan/Open_Duck_Mini_Runtime` após o backup.

No ambiente `(patruck-runtime)`:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime
python -m pip install adafruit-extended-bus==1.0.2
python -m pip install --no-deps -e .
```

Confira qual pacote o Python está importando, sem inicializar hardware:

```bash
python -c "import mini_bdx_runtime.raw_imu as m; print(m.__file__)"
```

Deve apontar para este checkout atualizado. No navegador, recarregue a página
ignorando o cache para obter o novo JavaScript e HTML.

## Primeiro: verificar o leitor do runtime sem motores

Com a alimentação dos motores desligada, Raspberry e IMU alimentados, e sem
outros leitores da IMU:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
python check_runtime_imu.py --i2c-bus 8 --duration 120
```

Esse comando importa **o mesmo `Imu` da caminhada**, mas não importa `HWI`,
controle web de caminhada ou código de acionamento dos motores. Lê a calibração
habitual da pasta `scripts` e a configuração em `~/duck_config.json`.

Uma pasta nova em `imu_runtime_checks` guarda amostras aceitas, falhas e hashes.
O resumo no terminal informa consultas aceitas, índices distintos e consultas
rejeitadas. Código de saída 2 sinaliza alguma rejeição ou falha; Ctrl+C retorna
130. Se aparecer amostra antiga, preserve os dados para avaliar o atraso.
Este verificador continua observando após falhas para registrá-las; o runtime
de caminhada, por sua vez, pausa e exige retomada explícita.

## Depois: usar o controle web

No seu comando habitual de caminhada, preserve o caminho real de `student.onnx`,
ganhos, porta serial e demais opções. Acrescente:

```text
--imu-i2c-bus 8 --start-paused --telemetry-dir ../telemetry --telemetry-label student-i2cgpio8-runtime
```

Exemplo, somente se o modelo estiver em `../models/student.onnx`:

```bash
python v2_rl_walk_mujoco.py --onnx_model_path ../models/student.onnx --control-source web --imu-i2c-bus 8 --start-paused --telemetry-dir ../telemetry --telemetry-label student-i2cgpio8-runtime
```

**Esse comando é o controlador real:** mesmo com `--start-paused`, a pose
inicial é enviada aos motores após validar a IMU. A opção pausa a política de
caminhada, não torna a inicialização um ensaio sem movimento. Use o robô apoiado.

O endereço e o token do controle web continuam aparecendo no terminal. Confirme
“Pausado” na página e use INICIAR quando estiver pronto para o ensaio assistido.

## Falha, pausa e retomada

Ao detectar problema da IMU, a política deixa de produzir novos alvos e mantém
o último alvo enviado. O torque **não é desligado**. Essa pausa não é uma
manobra de recuperação de equilíbrio e não garante que o robô permaneça em pé.

1. Apoie o robô e verifique o motivo no terminal/telemetria.
2. Pressione **PARAR** no controle web para reconhecer a falha.
3. Solte e centralize os controles de movimento e cabeça.
4. Pressione **INICIAR**. A retomada só ocorre se o leitor confirmar dados
   recentes e três amostras válidas consecutivas. Se falhar, a pausa permanece.

Mensagens de movimento ou o retorno espontâneo da IMU não retomam a política.
No Xbox, o primeiro A reconhece a falha e o seguinte solicita retomada.
Ctrl+C para a coleta/controle e encerra o leitor da IMU; não desliga torque.

## Telemetria e limites da entrega

Metadados e `runtime_ready` registram barramento e idade máxima. Os ciclos
incluem `imu_oldest_age_ms`, além do campo anterior de idade desde o término da
aquisição. Eventos `imu_fault` e `imu_fault_cleared` identificam pausa e retomada.

Não houve mudança no ONNX, na convenção dos eixos, nos offsets, ganhos ou escala
de ação. O limitador de variação de alvos continua como estava; sua validação
com a simulação é outra etapa. O adaptador ainda depende de caracterizar a
instabilidade residual com entradas confiáveis.

Para voltar ao hardware, restaure primeiro a configuração de boot e só então
use `--imu-i2c-bus 1`. Não há seleção automática se o barramento 8 estiver ausente.

Validação local: testes Python de transporte, prontidão, validade, envelhecimento,
pausa/retomada, telemetria e regressão das ações; teste Node da interface web;
compilação Python e ajuda do verificador. Nenhum teste local acionou hardware.
