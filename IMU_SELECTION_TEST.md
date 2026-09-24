# Seleção recente da IMU antes da inferência

Esta atualização trata a falta de margem temporal observada em 24/09/2026.
Não altera ONNX, ganhos, calibração, barramento, controles web ou o limite final
de 50 ms. A alteração de comportamento está em `scripts/v2_rl_walk_mujoco.py`.

## Comportamento

Após as leituras dos servos, o runtime consulta novamente o cache da IMU.
Seleciona os valores de giro e aceleração dessa amostra para a observação antes
de executar a inferência. Não reaproveita uma ação calculada com outra amostra.

Com limite de 50 ms, exige idade de até 40 ms na seleção, reservando 10 ms para
o restante do trabalho. A reserva é heurística, não garantia de tempo real.
Para limites menores que 20 ms, a reserva é metade do limite configurado.

Se a amostra ainda é válida mas não tem margem, aguarda em intervalos de até
0,5 ms por uma melhor. O prazo é o menor entre 5 ms desde a seleção e o fim do
período de controle menos a reserva. Não há espera se esse prazo já passou.
Um despertar além do prazo também é rejeitado. Agendamento do Linux pode
prolongar a espera real; nesse caso a inferência não prossegue.

Erro do leitor, amostra inválida ou falta de margem causam a pausa existente.
Não há nova tentativa de inferência após falha. Permanecem as checagens após
leitura dos servos, após inferência e imediatamente antes de enviar aos motores.
Há também uma checagem adicional do orçamento de execução antes da inferência.
Histórico de ações e fase só avançam após envio efetivo.

## Instalar no Raspberry

1. Encerre o runtime. Faça cópia do script atual para permitir retorno à versão anterior.
2. Extraia `patruck-imu-selection-update.zip` na raiz
   `/home/jonathan/Open_Duck_Mini_Runtime`, preservando a estrutura das pastas.
   O pacote pressupõe a instrumentação anterior já instalada e inclui apenas
   o script alterado, este guia e o manifesto.
3. Confirme sem importar o controlador nem iniciar hardware:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime
python -m py_compile scripts/v2_rl_walk_mujoco.py
grep -n imu_selection_version scripts/v2_rl_walk_mujoco.py
```

4. Ative o mesmo ambiente Python e execute o comando anterior, preservando
   modelo, parâmetros do robô e controle web. Mantenha as opções:

```text
--imu-i2c-bus 8 --imu-max-age-ms 50 --runtime-gc bounded --active-window-s 10 --telemetry-writer-yield-ms 1 --start-paused --telemetry-dir ../telemetry --telemetry-label student-imu-selection-v1
```

Essas opções são complemento do comando usado em `scripts`, não um comando
independente. Não execute simultaneamente outro leitor da IMU.

## Ensaio

1. Mantenha o robô apoiado: o início pausado ainda envia a pose inicial; pausa
   mantém o último alvo e não garante equilíbrio.
2. Faça três janelas com controles centralizados, permitindo a pausa de 10 s.
3. Reconheça cada pausa pelo controle web: PARAR, centralizar e INICIAR.
4. Se não houver falha de IMU, faça três janelas com comandos breves para frente,
   mantendo apoio contra queda. Registre vídeo incluindo início e fim do runtime.
5. Havendo falha, preserve os dados e o motivo. Não aumente o limite de idade.
6. Encerre com Ctrl+C e aguarde finalizar a gravação.

## Dados a enviar

Envie a pasta completa, incluindo chunks, metadata, status e writer_timing.
O metadata deve conter `settings.imu_selection_version=1`.

Nos ciclos concluídos, `sensors` registra a amostra efetivamente usada pela
política, além de `imu_initial_sample_index`, `imu_selected_monotonic_ns`,
`imu_selection_wait_ms`, `imu_selection_polls` e `imu_reserve_ms`.
`imu_oldest_age_ms` passa a indicar idade no instante de seleção; a idade final
continua calculável por `motor_write_start_monotonic_ns` menos
`imu_sample_start_monotonic_ns`. Tempos dos servos continuam disponíveis e
podem anteceder a aquisição da amostra selecionada.

Falhas na seleção aparecem como `select_imu_before_inference`, com prazo,
reserva, número de consultas, índice e idade no diagnóstico. Se o leitor falhar
antes de fornecer um snapshot, os últimos campos podem não existir.

A análise deverá separar pausas por duração de falhas de IMU, conferir a
identidade dos valores usados pela política e medir espera, idade final e
tempo do ciclo. Sucesso nos testes locais não valida latência ou marcha no Pi.
