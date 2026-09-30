# Correção de orientação IMU por software

**Validação de 30/09/2026: `yaw-plus-90` ainda não está aprovada para caminhada.**
As coletas têm divergências de sinal entre poses e movimentos confirmados pelo
operador. Manter este modo como candidato de diagnóstico. O roteiro anterior
indicava os sinais estáticos de aceleração ao contrário; os esperados abaixo
foram corrigidos por comparação com MuJoCo. Não corrigir sinais individualmente
para tentar aprovar uma rotação inconsistente.

O novo modo `yaw-plus-90` transforma os vetores já remapeados pelo BNO055:

```text
X novo = -Y atual
Y novo =  X atual
Z novo =  Z atual
```

A mesma rotação é aplicada a aceleração e giroscópio, uma única vez no leitor.
Não modifica calibração gravada, remapeamento do chip, ganhos ou modelo.
`native` é o padrão, preservando o comportamento anterior. A rotação não
renova timestamps e não relaxa as proteções de idade da IMU. Não corrige os
atrasos temporais encontrados nas coletas anteriores.

## Instalar

Encerre o runtime e extraia `patruck-imu-frame-correction.zip` em
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas. O pacote traz
o novo módulo imu_frame.py, o leitor raw_imu.py, o script de caminhada e o
diagnóstico atualizado. Feche processos antigos antes de usar a nova versão.

## Primeiro validar sem motores

Mantenha o runtime e outros leitores da IMU encerrados, os servos sem
alimentação e o robô sustentado. Não execute o script de caminhada nesta fase.
No ambiente patruck-runtime:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_imu.py --help
```

Confira a opção `--imu-frame`. O diagnóstico preserva os valores do driver e
os bytes em `primary`; adiciona os valores corrigidos em `frame_preview`.
Não há aplicação de comandos aos motores nem carregamento de ONNX.

Prepare cada posição antes do comando e mantenha o corpo inteiro imóvel,
com inclinação de aproximadamente 15 graus. Não mova apenas a cabeça. As
direções esquerda/direita são as do robô. Execute um comando por vez:

```bash
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 10 --output ../imu_diagnostics --label frame-v1-level
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 10 --output ../imu_diagnostics --label frame-v1-nose-down
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 10 --output ../imu_diagnostics --label frame-v1-right-down
```

No primeiro, nivelado; no segundo, frente do corpo desce; no terceiro, lado
direito do corpo desce. A resposta corrigida esperada é aumento principal de
X NEGATIVO no nose-down, aumento principal de Y no right-down, Z positivo.

## Depois conferir giro durante a gravação

Aqui é diferente das poses acima: comece nivelado, aguarde cerca de 2 segundos
após aparecer o início da aquisição e mova lentamente durante a gravação.
Uma segunda pessoa pode operar o terminal. Faça uma ida, mantenha brevemente,
e volte ao centro. Não sacuda nem force juntas.

```bash
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 15 --output ../imu_diagnostics --label frame-v1-pitch-motion
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 15 --output ../imu_diagnostics --label frame-v1-roll-motion
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-plus-90 --duration 15 --output ../imu_diagnostics --label frame-v1-yaw-motion
```

1. pitch-motion: frente do corpo desce e volta. Esperado gyro Y positivo na
   ida, negativo na volta.
2. roll-motion: lado direito do corpo desce e volta. Esperado gyro X positivo
   na ida, negativo na volta.
3. yaw-motion: mantenha nivelado; vire o corpo inteiro para a esquerda dele
   (anti-horário visto de cima) e volte. Esperado gyro Z positivo na ida,
   negativo na volta.

Os sinais se referem ao vetor CORRIGIDO. Informe se mudou a ordem ou o sentido.
Se possível, grave as manobras para eliminar dúvida de orientação. Não há
sincronização automática de vídeo: não relacionaremos segundos entre arquivos
sem um marco comum. Ctrl+C salva o diagnóstico parcial.

Copie as seis novas pastas imu_diagnostics para análise. Os testes antigos não
precisam ser apagados. Este roteiro verifica a candidata antes da caminhada.

## Ativação no runtime, após validar os dados

No comando habitual de caminhada, a opção que habilita a correção é:

```text
--imu-frame yaw-plus-90
```

Use também um novo telemetry-label para distinguir a coleta. Modelo, controle
web e demais opções permanecem os escolhidos para o ensaio. A UI não precisa
de novo botão: a seleção ocorre ao iniciar o processo.

Metadata/runtime_ready registram imu_frame; cada ciclo registra os vetores
efetivamente selecionados e os campos gyro_native_rad_s e accel_native_m_s2.
Esses campos são anteriores à nova rotação, após o remapeamento e o offset
legado do leitor. imu_frame.py entra no manifesto de hashes.

Para voltar ao comportamento anterior, encerre o processo e use
`--imu-frame native` (ou remova a opção). Não gire fisicamente a IMU ao mesmo
tempo nem aplique a transformação novamente em outro ponto do pipeline.
