# Teste da IMU com I²C por software, mantendo os fios

O operador confirmou GPIO2=SDA1, GPIO3=SCL1, controlador bcm2835 no barramento
1. A IMU é o único dispositivo nesses fios, com aproximadamente 300 mm. Vamos
substituir temporariamente esse controlador por `i2c-gpio`, usando os mesmos
GPIO2/3 e o barramento 8, que não aparece na listagem atual.

O ensaio anterior em 10 kHz apresentou erros e cerca de 21 Hz de aquisição.
Este teste investiga outro controlador com clock nominal próximo de 100 kHz.
Não permite separar sozinho o efeito de controlador do efeito de clock; as
tentativas iniciais de hardware a 100 kHz também estão preservadas. O clock
efetivo do software depende do sistema e de clock stretching.

## 1. Preparar o Raspberry

Mantenha o robô apoiado, motores sem alimentação e Raspberry/IMU alimentados.
Encerre o runtime e outros leitores da IMU. Se a caminhada inicia como serviço,
mantenha esse serviço parado também após reiniciar. Não rode o controle web de
caminhada durante este ensaio.

Copie a versão atualizada de `scripts/diagnose_imu.py` deste projeto para
`/home/jonathan/Open_Duck_Mini_Runtime/scripts/diagnose_imu.py`. Nenhuma alteração
em `raw_imu.py` é necessária para este ensaio isolado. O runtime de caminhada
agora tem integração própria com o barramento 8; consulte
[a atualização e uso no controle web](IMU_RUNTIME_I2C8.md).

No ambiente `(patruck-runtime)` já usado:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
python -m pip install adafruit-extended-bus
python diagnose_imu.py --help
```

Confirme que `--i2c-bus` aparece na ajuda. A instalação acrescenta a biblioteca
para selecionar um barramento Linux explicitamente; não atualize o driver da
IMU ou a calibração neste teste.

## 2. Backup e edição da configuração de boot

Execute no Raspberry:

```bash
if [ -f /boot/firmware/config.txt ]; then
    imu_boot_config=/boot/firmware/config.txt
else
    imu_boot_config=/boot/config.txt
fi
imu_boot_backup="$imu_boot_config.imu-before-software-$(date +%Y%m%d-%H%M%S)"
sudo cp -- "$imu_boot_config" "$imu_boot_backup"
printf 'Backup: %s\n' "$imu_boot_backup"
sudo nano "$imu_boot_config"
```

Guarde o caminho impresso. Preserve as opções de câmera, áudio e demais
periféricos. Ao final do arquivo, acrescente este bloco uma única vez:

```ini
[all]
# BEGIN PATRUCK IMU SOFTWARE TEST
dtparam=i2c_arm=off
dtoverlay=i2c-gpio,bus=8,i2c_gpio_sda=2,i2c_gpio_scl=3,i2c_gpio_delay_us=2
# END PATRUCK IMU SOFTWARE TEST
```

GPIO usa numeração BCM: `2` e `3`, conforme `pinctrl`; não numeração física do
conector. O `i2c_arm=off` é necessário para liberar esses pinos do controlador
de hardware. Não ative os dois controladores nos mesmos pinos. O bloco deve
ficar depois de definições anteriores de `i2c_arm` e de `include`. Se houver
overlay que force o I²C1, ou outro `i2c-gpio`, não duplique: envie a configuração
para avaliarmos a coexistência. Não desabilite globalmente o módulo bcm2835,
pois os outros barramentos podem atender à câmera.

O `i2c_arm_baudrate=10000` do teste anterior pode permanecer: aplica-se ao
controlador de hardware, agora desabilitado. No software, o parâmetro é
`i2c_gpio_delay_us=2`, nominalmente próximo de 100 kHz conforme a documentação.

Salve e reinicie:

```bash
sudo reboot
```

## 3. Conferir a troca antes de ler a IMU

Reconecte e confirme que o runtime permanece parado. Execute:

```bash
i2cdetect -l
pinctrl get 2,3
ls -l /dev/i2c-8
cat /sys/bus/i2c/devices/i2c-8/name
```

O barramento 8 deve existir; o controlador bcm2835 de `/dev/i2c-1`, em
`i2c@7e804000`, deve ter desaparecido. GPIO2/3 não devem continuar na função
alternativa `a0` SDA1/SCL1. Os barramentos 0, 10 e 11 podem continuar presentes.
`i2cdetect -l` apenas lista adaptadores; não é uma varredura de endereços.

Se o barramento 8 faltar, o I²C1 de hardware continuar presente ou os GPIOs
continuarem como SDA1/SCL1, não execute a aquisição: envie a saída para corrigir
a configuração. Não use `pinctrl set` nem `raspi-config` para forçar os pinos.

## 4. Duas coletas, sem mover o robô

Ative o mesmo ambiente Python e execute na pasta `scripts`:

```bash
cd /home/jonathan/Open_Duck_Mini_Runtime/scripts
python diagnose_imu.py --i2c-bus 8 --duration 120 --frequency 50 --label parado-motores-desligados-i2cgpio8
python diagnose_imu.py --i2c-bus 8 --duration 120 --frequency 50 --label parado-motores-desligados-i2cgpio8-repeticao
```

Preserve modo NDOF, endereço 0x29, configuração, calibração e fios. O script não
aciona motores; sua inicialização reconfigura a IMU. `--i2c-bus 8` abre somente
esse barramento e falha se ele não estiver disponível; não volta ao hardware.
Ctrl+C finaliza os blocos pendentes.

Copie as duas pastas novas de `imu_diagnostics`, mesmo se ocorrer falha, e a
saída das verificações do passo 3. `metadata.json` registra `i2c_backend` como
`extended_bus`, `i2c_bus` como 8 e versões das bibliotecas. Essa biblioteca pode
abrir barramentos Linux de diferentes tipos: o número sozinho não comprova que
o controlador é software; por isso conferimos também a configuração do sistema.

## 5. Como avaliar

O novo resumo inclui `effective_frequency_hz`,
`samples_with_invalid_mode_byte` e `samples_with_any_detected_anomaly`.
Contaremos os erros de modo durante toda a aquisição, além de extremos,
conversão, dados ausentes e erros de I/O. A releitura continua disparando pelos
mesmos critérios anteriores; a nova contagem de modo não adiciona transações.
O campo antigo `abnormal_samples` não inclui sozinho todas as anomalias.

Precisamos verificar integridade, frequência próxima de 50 Hz e distribuição
das latências nas duas repetições. Ausência de extremos não basta para declarar
todos os valores corretos. Mesmo um resultado bom parado precisa de validação
posterior sob carga antes de migrar o runtime ou retomar caminhada.

## 6. Reverter

No mesmo arquivo de boot, remova somente o bloco entre os comentários
`BEGIN PATRUCK IMU SOFTWARE TEST` e `END PATRUCK IMU SOFTWARE TEST`.
Preserve as opções anteriores do arquivo; o backup registra seu estado original.
Reinicie e confira novamente `i2cdetect -l` e `pinctrl get 2,3`: o I²C1 deve
retornar com SDA1/SCL1. Isso restaura a configuração anterior, inclusive os
10 kHz, se mantidos. A reversão não significa que a falha da IMU foi resolvida.

## Fontes e validação local

- [Adafruit: I²C por software e ExtendedI2C](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes/software-i2c).
- [Raspberry Pi: parâmetros do overlay i2c-gpio](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README).

Os testes locais verificaram seleção explícita do barramento, ausência de
fallback, captura/conversão, erros de I/O, inicialização e contagem de modo.
A ajuda e a análise de sessões antigas funcionam sem hardware. Nenhuma mudança
de boot ou aquisição no Raspberry foi executada pelo assistente.
