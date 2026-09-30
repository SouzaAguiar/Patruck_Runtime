# Revisão: yaw-minus-90 e confirmação de giro horizontal

A coleta guiada indicou a rotação no sentido contrário à primeira candidata:

```text
X novo =  Y original
Y novo = -X original
Z novo =  Z original
```

Original significa o vetor que já passou pelo remapeamento BNO055 atual.
A opção é `--imu-frame yaw-minus-90`, aplicada igualmente a aceleração e
giroscópio. O padrão continua native; a mudança é explicitamente selecionada.

## Instalar e fazer somente o giro horizontal

Encerre o runtime e extraia `patruck-imu-frame-v2.zip` na raiz
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas. Mantenha os
servos sem alimentação e sustente o robô. Não execute a caminhada agora.

No ambiente patruck-runtime:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_imu.py --i2c-bus 8 --imu-frame yaw-minus-90 --duration 15 --output ../imu_diagnostics --label frame-v3-yaw-horizontal
```

O diagnóstico não aciona motores. Os valores originais permanecem em primary;
os corrigidos são registrados em frame_preview, com identificação na metadata.

1. Comece com o corpo nivelado e parado. Aguarde COLETA INICIADA e conte 2 s.
2. Gire lentamente o corpo inteiro para a ESQUERDA do robô, mantendo-o
   NIVELADO, por cerca de 3 s. Imagine mudar para onde ele aponta sobre uma
   mesa: anti-horário visto de cima. Nenhum lado deve ser abaixado.
3. Mantenha essa direção durante aproximadamente 2 s.
4. Gire de volta à direção inicial lentamente, por aproximadamente 3 s.
5. Permaneça parado até o teste terminar.

Use giro pequeno, cerca de 20 graus. Não mova só cabeça, não incline o tronco
e não force as juntas. Esperamos gyro Z positivo na ida para esquerda e
negativo na volta, com aceleração próxima ao estado nivelado.

Nesta rodada não use --guided-frame-test, pois queremos uma única manobra.
Se houver atraso ou movimento diferente, informe ao enviar a pasta completa
de imu_diagnostics. Ctrl+C preserva dados parciais.

## Runtime após análise desta confirmação

A opção prevista para o próximo ensaio é --imu-frame yaw-minus-90, com novo
telemetry-label. Somente instalar os arquivos não ativa a rotação. Remover a
opção ou usar --imu-frame native restaura o comportamento anterior ao iniciar
um novo processo. Não usar yaw-plus-90 por engano nem modificar a montagem
física simultaneamente.

A revisão não altera idade máxima, timestamps, PID, modelo ou calibração;
os atrasos intermitentes de leitura continuam exigindo tratamento separado.
