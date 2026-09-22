# Runtime com sessões limitadas e GC durante as pausas

A integração está pronta no código local. Ainda precisa de validação física no
Raspberry. Nenhuma execução local acionou motores.

## Comportamento

O comando de caminhada passa a usar, por padrão:

- `--runtime-gc bounded`: coleta automática suspensa apenas enquanto a política
  está ativa; estado anterior restaurado ao pausar ou encerrar;
- `--active-window-s 30`: cada intervalo ativo dura no máximo 30 segundos;
- `--telemetry-writer-yield-ms 1`: gravador cede tempo após cada linha JSON;
- limites de crescimento RSS de 64 MiB e memória disponível mínima de 64 MiB.

O modo bounded força início pausado após a pose inicial, exige controles
centralizados para INICIAR e mantém a proteção da IMU em 50 ms. Não muda ONNX,
eixos, calibração, ganhos, escala de ações ou histórico da política.

**O controlador real ainda aciona os motores na pose inicial.** Início pausado
não equivale a um ensaio sem motores. Pausar mantém o último alvo e não desliga
torque nem garante equilíbrio. Mantenha o robô apoiado durante a validação e
interrompa voluntariamente antes do limite; ele é uma proteção adicional.

Ao atingir tempo, memória ou detectar falha/perda da telemetria, o runtime
interrompe novos ciclos da política e mostra o motivo no controle web. Retomar
requer PARAR para reconhecer, controles centralizados e INICIAR. Comandos de
início repetidos durante execução não estendem o prazo. Retomadas não são automáticas.

O GC explícito ocorre uma vez na entrada de uma pausa quando necessário,
inclusive na pausa inicial. Antes de retomar, a memória é novamente validada
e o leitor precisa confirmar dados recentes da IMU após a manutenção. Enquanto
ativo, as verificações de tempo ocorrem antes da observação e antes de enviar
alvos; a memória é consultada aproximadamente a cada segundo. Isso não é um
limite de memória imposto pelo kernel e não garante prazos rígidos do Linux.

O baseline de RSS, obtido na primeira manutenção, é mantido entre retomadas.
Assim, um crescimento acumulado não é ocultado por pausas sucessivas. RSS pode
não cair imediatamente mesmo após liberar objetos; se o limite continuar
excedido, a retomada é bloqueada e a sessão deve ser encerrada para investigação.

## Atualizar no Raspberry

Encerre o runtime anterior. Extraia o pacote na raiz
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando os caminhos:

```text
scripts/v2_rl_walk_mujoco.py
mini_bdx_runtime/mini_bdx_runtime/runtime_budget.py
mini_bdx_runtime/mini_bdx_runtime/telemetry.py
mini_bdx_runtime/mini_bdx_runtime/web/control.js
RUNTIME_GC_INTEGRATION.md
```

O pacote pressupõe a integração I²C8 já instalada e validada. Não substitui
modelos, configuração, calibração, boot ou offsets. Não há dependência nova.

Confira a importação do novo módulo, sem inicializar hardware:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime
python -c "import mini_bdx_runtime.runtime_budget as m; print(m.__file__); print(m.memory_snapshot())"
```

O caminho deve apontar para o checkout atualizado. Se o pacote ainda não estiver
instalado em modo editável, use o mesmo ambiente do runtime e execute na raiz:

```bash
pip install --no-deps -e .
```

Recarregue a página do controle web ignorando cache. A nova interface mostra
pausas por limite de sessão/memória como motivos do runtime, distintos da IMU.

## Primeiro ensaio físico assistido

Preserve o comando habitual e o caminho real do student, porta serial, ganhos
e outras opções usadas nos testes anteriores. Adicione ou ajuste:

```text
--control-source web --imu-i2c-bus 8 --imu-max-age-ms 50 --runtime-gc bounded --active-window-s 10 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-i2c8-gc-window10
```

Exemplo com caminho de modelo a substituir:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
python v2_rl_walk_mujoco.py --onnx_model_path /caminho/real/student.onnx --control-source web --imu-i2c-bus 8 --imu-max-age-ms 50 --runtime-gc bounded --active-window-s 10 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-i2c8-gc-window10
```

1. Com o robô apoiado, confirme a pose inicial e o estado Pausado no navegador.
2. Centralize os controles e pressione INICIAR. Primeiro observe alguns segundos
   com comandos zerados e use PARAR antes de 10 segundos.
3. Espere a manutenção terminar e retome explicitamente. Faça um comando curto
   para frente, mantendo apoio, e pause novamente.
4. Para verificar o limite automático, mantenha apoio e observe uma janela
   atingir 10 segundos: a página deve mostrar o motivo e bloquear retomada até
   PARAR/reconhecimento. Não provoque falhas de memória ou disco no robô.
5. Encerre com Ctrl+C após algumas janelas e copie a pasta completa de telemetria.

Ctrl+C encerra a política/leitor e restaura o GC; não desliga torque. Depois de
avaliarmos idade da IMU, memória e retomadas, ampliar gradualmente as janelas
para 30 segundos. O máximo aceito é 120 segundos; não usar como validação de
operação contínua. Ainda não há evidência de estabilidade física da caminhada.

## Evidências registradas

- Metadados: política de GC, janela ativa, limites de memória e pausa do gravador;
- `runtime_active_start` e `runtime_active_end`: intervalos ativos;
- `runtime_memory`: RSS, memória disponível, baseline e tempo ativo;
- `runtime_gc_maintenance`: início/fim, objetos coletados e RSS antes/depois;
- `runtime_guard_pause` / `runtime_guard_cleared`: motivo e retomada explícita;
- Eventos existentes de IMU, ciclos, comandos e envios aos motores preservados.

Uma coleta de lixo em pausa pode produzir rejeições temporárias no leitor; a
prontidão posterior impede retomar com a amostra envelhecida. Falha de gravação
pode impedir registrar o próprio motivo em disco; nesse caso a página/terminal
continuam indicando a pausa. Não apague o `status.json` do ensaio.

## Comparação/reversão de software

`--runtime-gc default --telemetry-writer-yield-ms 0` restaura o comportamento
anterior de GC/gravação. Esse modo não aplica as janelas nem os limites de memória
da nova integração; não é recomendado para o próximo ensaio, pois os atrasos
anteriores já foram reproduzidos. A validação da idade da IMU continua ativa.

A entrega inclui testes locais com hardware simulado. Testes verificam prazo,
memória, manutenção apenas em pausa, retomada, restauração do GC em falhas,
ausência de envio após atingir limite e preservação dos registros do gravador.
