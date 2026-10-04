# Renovação da amostra IMU — próximo teste parado

Preparado em 03/10/2026 após os ensaios trace-a/trace-b. A seleção passou para
versão 3: se a amostra vencer durante a leitura das juntas, o runtime pode
aguardar uma amostra válida na seleção antes de decidir pela pausa.
Erros do leitor, vetores/timestamps inválidos e falhas de comunicação não
recebem essa tentativa de recuperação.

Os limites continuam os mesmos: idade máxima 50 ms, reserva de 10 ms antes
de inferir, orçamento de espera de até 5 ms e prazo de seleção do ciclo.
A amostra efetivamente usada é validada novamente antes do envio aos motores.
Não há repetição de ação antiga nem renovação artificial de timestamps.
A rotina existente pode observar atraso de agendamento na última consulta;
o trace registra o excesso sobre o orçamento de espera e o prazo do ciclo
continua sendo verificado. A correção não garante eliminar todas as pausas.

**O ensaio real ativa os servos e prepara a postura inicial.** Mesmo com
comandos zero, a política ajusta as juntas para manter a postura. Apoie/proteja
o robô na preparação como nos testes anteriores. Pausa/encerramento não cortam
torque e mantêm o último alvo.

## Atualização e conferência

1. Encerre o runtime e qualquer outro leitor da IMU.
2. Extraia `patruck-runtime-refresh-tests.zip` na raiz
   `/home/jonathan/Open_Duck_Mini_Runtime`, preservando todas as pastas.
   Atualize o pacote inteiro: `imu_safety.py` e o script do runtime precisam
   estar na mesma revisão. Mantenha calibração, config, referência e ONNX.
3. Ative seu ambiente `patruck-runtime`. No mesmo terminal:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
read -r -p "Cole o caminho do ONNX teacher usado no ultimo runtime: " TEACHER_ONNX
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --dry-run
```

O dry-run não inicializa hardware. Deve informar **teacher e seleção IMU v3
verificados**. O teacher esperado é o mesmo BEST_WALK_ONNX_2.onnx, SHA-256
`3c606f9381a1710cc8fecdb7442787dcbfce3ee9bc02a6f1224774ab2b3a1067`.
Se houver erro, corrija a atualização/caminho antes de executar.

O lançador usa `~/duck_config.json`, porta serial `/dev/ttyACM0`, web 8080
e token `duck-test`. Se usava outra config/serial, preserve os valores com
`--config CAMINHO` e `--serial-port PORTA` no dry-run e nos dois comandos abaixo.

## Duas sessões iguais

Primeira sessão:

```bash
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --label imu-runtime-refresh-a
```

1. Aguarde terminar a preparação e aparecer `Starting`.
2. Abra uma aba de `http://IP_DO_PI:8080`. Deixe câmera desligada, joysticks
   centralizados, cabeça neutra e “Só frente/trás” ligado. Não envie movimentos
   nem use controles de frequência/expressões.
3. Clique **INICIAR** e observe o equilíbrio parado por até 10 segundos.
4. Se pausar por **“Limite de 10 s atingido”**, a janela terminou normalmente.
   Apoie se necessário, clique **PARAR** para reconhecer a pausa, aguarde a
   manutenção e clique **INICIAR** com controles centrados. Faça até três janelas.
5. Se aparecer **“PAUSA IMU”**, encerre aquela sessão com **Ctrl+C** e aguarde
   a exportação. Não retome depois da falha nessa mesma sessão. Isso preserva
   um evento de comparação claro; a pausa ainda pode ser necessária.
6. Depois de três janelas, encerre com Ctrl+C. Se não encerrar manualmente,
   o lançador prepara saída automática após 90 segundos de coleta, contados
   depois da preparação dos motores; esse tempo inclui pausas.

Depois de encerrar completamente, repita com as mesmas condições:

```bash
python runtime_stationary_test.py --onnx-model "$TEACHER_ONNX" --label imu-runtime-refresh-b
```

O modo de teste pausa diante de qualquer comando externo não zero, incluindo
cabeça, giro e lateral. A mensagem “Teste parado: comando nao zero” e o evento
stationary_command_rejected identificam esse caso, separado de falha da IMU.

## O que disponibilizar

Copie as duas pastas novas de `../telemetry` para
`E:\www\Patruck_Runtime\telemetry`, incluindo todos os chunks e:

- `metadata.json` e `status.json`;
- `runtime_timing_trace.json`;
- `writer_timing.json`.

Use Ctrl+C/saída automática para encerrar; não desligue a alimentação antes
da exportação. O trace permanece em memória durante o controle.
Anote se houve apoio, desequilíbrio e qual mensagem interrompeu a sessão.
Marque “Apoio da mão” se intervier. Vídeo é útil, mas não obrigatório para
a análise dos tempos. Esta rodada continua sem comandos de marcha.

Resumo opcional, apenas depois de encerrar; substitua PASTA_DA_SESSAO:

```bash
python summarize_runtime_trace.py ../telemetry/PASTA_DA_SESSAO --output ../telemetry/PASTA_DA_SESSAO/runtime_trace_summary.json
```

Na análise verificaremos `imu_selection_version: 3`, ciclos recuperados
depois de a amostra inicial vencer, recuperações após espera de cache antigo,
idade da amostra no início do envio ao motor, pausas restantes e tempos do
produtor. Em ciclos recuperados, observação, telemetria e ação devem usar a
mesma amostra selecionada. Mais tempo ativo sozinho não comprova a correção.

O trace continua opcional, com memória fixa de cerca de 8,2 MiB e capacidade
configurada para 120 segundos a 50 Hz. Não adiciona leituras do barramento
nem gravação JSON durante o controle. Modelo, PID, escala, barramento 8,
yaw-minus-90 e yield de telemetria 1 ms foram preservados no lançador.
