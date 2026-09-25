# Bloqueio lateral no servidor e diagnóstico dos comandos

## Instalação

Com o runtime encerrado, extraia `patruck-web-server-lock.zip` na raiz
`/home/jonathan/Open_Duck_Mini_Runtime`, preservando as pastas. O pacote contém
o controlador web Python e os três arquivos da interface compacta. Reinicie o
runtime pelo comando habitual e feche/reabra a página do controle. O HTML recebe
no-store e os recursos usam uma versão na URL para reduzir uso de cache antigo.
Uma aba já aberta precisa ser recarregada.

O interruptor **Só frente/trás** continua inicialmente desligado. Ligue-o antes
de iniciar o ensaio. Ele deve aparecer com a trilha ciano e o indicador à direita.
Ao alternar, os comandos são zerados e um novo gesto é necessário. O bloqueio
não impede o controle de giro do joystick direito. O modo de cabeça autorizado
continua com os eixos próprios. A página mantém sua seleção durante pausas;
recarregar a página volta ao estado desligado.

## Comportamento no servidor

Cada mensagem do cliente atualizado inclui `lateral_locked` como booleano e
`client_version=web-lateral-lock-v2`. No modo efetivo de caminhada, o servidor
força o comando lateral a zero quando recebe `lateral_locked=true`, mesmo se
`left_x` vier diferente de zero. Frente/trás e giro mantêm seu mapeamento.

Clientes antigos sem o campo continuam com o mapeamento anterior; a ausência
é registrada como desconhecida, nunca como comprovação de bloqueio desligado.
Um campo explícito de tipo inválido é registrado como não reconhecido e tratado
conservadoramente como bloqueio lateral. O estado é avaliado por mensagem:
um cliente antigo não herda o bloqueio de outro cliente. Use uma única página
de controle no ensaio para evitar alternância de remetentes.

## Campos na telemetria

Em `command_source` dos ciclos e registros de pausa:

- `control_diagnostics_version`: 1;
- `client_version`: versão declarada pelo cliente, ou null para cliente antigo;
- `command_sequence`: contador de mensagens aceitas pelo servidor;
- `requested_mode` e `effective_mode`: modo solicitado e modo aplicado;
- `lateral_lock_reported`: o campo recebido é um booleano válido;
- `lateral_lock_requested`: true/false recebido, ou null se ausente/inválido;
- `lateral_lock_applied`: restrição aplicada no mapeamento da última mensagem;
- `received_axes`: eixos left_x, left_y e right_x recebidos, após limitação a
  [-1,1]. O cliente já pode ter zerado left_x: não são coordenadas do dedo;
- `effective_commands`: vetor entregue pelo servidor ao runtime nesse snapshot,
  incluindo zeros em caso de timeout ou desconexão.

Em timeout/desconexão, os dados da última mensagem permanecem como contexto;
`command_fresh`/`connected` e `effective_commands` identificam a saída atual.
A versão declarada não é hash do arquivo do navegador.

## Próximo ensaio

Mantenha o modelo, configuração física documentada e janelas de 10 segundos.
Com apoio contra queda, faça uma janela de comandos breves para frente e outra
para trás, ambas com o interruptor ligado e sem usar o joystick de giro.
Anote em qual janela ocorreu deriva e se houve deslocamento lateral ou rotação.
Não é necessário testar lateral livre no robô para validar este ajuste.

Encerre normalmente e disponibilize a pasta completa da telemetria. Esperado:
cliente v2, bloqueio solicitado/aplicado true nos ciclos ativos do ensaio,
`commands[1]` e `effective_commands[1]` iguais a zero. O giro deve ficar zero
por não ter sido solicitado, e não por esse bloqueio.

Validação local: 15 testes Python e 2 testes Node aprovados, com hardware
simulado. Incluem bloqueio no servidor mesmo com eixo recebido não nulo,
frente/trás, desligamento, clientes antigos, modo cabeça, timeout/desconexão,
telemetria e observação da política. Nenhum hardware acionado.

Esta atualização trata a origem dos comandos laterais. Os critérios de margem
temporal da IMU permanecem iguais aos do último teste; suas duas pausas ainda
exigem investigação separada.
