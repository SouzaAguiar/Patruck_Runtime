# Próximos testes: contatos dos pés e orientação da IMU

Esta rodada é estática. Não iniciar caminhada nem o controle web do runtime.
Objetivo: validar as observações usadas pelos dois modelos antes de voltar à
marcha. Os testes não corrigem ainda as sete pausas temporais do Teacher 2.

## 1. Copiar os arquivos e preparar o robô

Extraia `patruck-static-sensor-tests.zip` na raiz do runtime do Pi:
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas.
O pacote contém os dois scripts de diagnóstico e este roteiro. Não altera
modelo, controle web, PID, offsets, configuração de boot ou script de caminhada.
O diagnóstico IMU usa o módulo de telemetria já instalado no runtime atual.

Encerre o runtime e qualquer outro leitor da IMU/GPIO. Mantenha o Pi ligado,
mas os servos sem alimentação; sustente o robô para não tombar. Os scripts
não enviam comandos aos motores e também não desligam motores previamente
acionados. Use a separação de alimentação disponível no robô, sem desconectar
fios energizados. Se não puder manter Pi ligado com servos desligados, informe
essa limitação antes da coleta.

No terminal já com `(patruck-runtime)` ativo:

```bash
cd ~/Open_Duck_Mini_Runtime/scripts
python diagnose_feet_contacts.py --help
```

`--help` não acessa hardware. Não é necessário instalar pacotes novos se o
runtime atual já lê os pés e a IMU normalmente.

## 2. Teste guiado dos pés

```bash
python diagnose_feet_contacts.py --output ../feet_diagnostics
```

O script orienta duas rodadas de seis etapas. Esquerdo e direito são os lados
do robô, não os de quem está olhando para ele de frente.

| Etapa | Condição mantida por 3 segundos | Esperado [esquerdo, direito] |
|---|---|---|
| Livres | Nenhum sensor pressionado | [false, false] |
| Esquerdo | Somente sensor esquerdo pressionado | [true, false] |
| Liberação | Soltar os dois | [false, false] |
| Direito | Somente sensor direito pressionado | [false, true] |
| Ambos | Pressionar os dois | [true, true] |
| Liberação final | Soltar os dois novamente | [false, false] |

Prepare a condição solicitada e pressione Enter. Após 1 segundo começa a
coleta de 3 segundos; mantenha a condição até aparecer o resultado. Apoie o
corpo do robô de modo que consiga deixar ambos os sensores livres e acioná-los
manualmente, sem forçar as juntas. Não use a pose dos motores para sustentá-lo.

O script salva GPIO bruto e contato lógico, sem filtragem ou inversão diferente
da usada pelo runtime: GPIO22 esquerdo, GPIO27 direito, pull-up, ativo em nível
baixo. A configuração corresponde ao código atual; não presume que a fiação
física esteja correta.

`OK` exige pelo menos 95% de leituras iguais ao estado solicitado e 80% das
amostras previstas. É uma triagem, não uma certificação do sensor. Se aparecer
`DIVERGENCIA`, termine as etapas restantes para termos o padrão completo; não
troque fios, sinais ou offsets para fazer o teste passar.

Pode pausar entre etapas, antes de pressionar Enter. Ctrl+C encerra e preserva
a coleta parcial; execute novamente para começar uma nova sessão. O script
não retoma automaticamente uma sessão interrompida. Guarde a pasta inteira
indicada no terminal, inclusive se houver erro.

## 3. IMU — três coletas isoladas, sem motores

Mantenha o runtime encerrado. O diagnóstico inicializa/configura a IMU, por
isso não deve disputar o sensor com outro processo. Usa barramento 8, modo
NDOF e os arquivos de configuração/calibração já usados no Pi. Não recalibre
nem use `--no-calibration` para contornar erro de arquivo.

Primeiro, robô apoiado e nivelado, corpo parado:

```bash
python diagnose_imu.py --i2c-bus 8 --mode ndof --duration 30 --frequency 50 --output ../imu_diagnostics --label static-v1-level
```

Depois incline **o corpo inteiro para frente**, cerca de 15 graus, apoiado ou
segurado firmemente nessa posição. Não dobre juntas para produzir a inclinação.
Prepare a posição antes de executar e mantenha-a até o fim:

```bash
python diagnose_imu.py --i2c-bus 8 --mode ndof --duration 10 --frequency 50 --output ../imu_diagnostics --label static-v1-nose-down
```

Por último, volte ao centro e incline **o corpo para a direita do robô**,
cerca de 15 graus, mantendo-o parado:

```bash
python diagnose_imu.py --i2c-bus 8 --mode ndof --duration 10 --frequency 50 --output ../imu_diagnostics --label static-v1-right-down
```

O ângulo aproximado serve para distinguir eixos/sinais, não para calibrar
numericamente a IMU. Informe qualquer movimento ou dificuldade em manter a
posição. Não altere `imu_upside_down` nem remapeamento nesta rodada. Se houver
erro, preserve a pasta e a mensagem, sem iniciar marcha.

Este teste isolado verifica orientação, ruído e comunicação básica. Um resultado
bom **não comprova** que a IMU cumpre os prazos sob carga no runtime; essa será
uma validação separada depois da revisão da seleção temporal.

## 4. O que enviar para análise

- A nova pasta inteira em `feet_diagnostics`.
- As três novas pastas em `imu_diagnostics`.
- Informe se alguma etapa foi difícil, se os sensores pressionam/liberam
  normalmente e se os servos ficaram sem alimentação.
- Vídeo é opcional nesta rodada. Se gravar, mostre qual sensor está sendo
  pressionado ou a inclinação aplicada; não precisamos de vídeo de caminhada.

Copie as pastas de volta para as mesmas áreas do checkout em
`E:\www\Patruck_Runtime`. Podemos analisar arquivos incompletos também.

O próximo passo dependerá desses resultados: corrigir observação/fiação se
houver divergência, revisar a seleção temporal da IMU e só então repetir
movimentos curtos com proteção contra queda. Ainda não trocar modelo nem
ajustar ganhos para compensar os sintomas.
