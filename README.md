# MyCobot 280 JN — MoveIt 2 + DDS Distribuído

Setup distribuído para controlar um **MyCobot 280 JN** com **MoveIt 2** rodando
em Docker no PC e o bridge de hardware rodando nativamente no **Jetson Nano**.

```
┌─────────────────────────────┐         ┌─────────────────────────────┐
│  PC (Linux + Docker)        │         │  Jetson Nano                │
│  ────────────────           │ DDS     │  ────────────              │
│  • MoveIt 2 + RViz          │ ◄────►  │  • mycobot_bridge          │
│  • joint_state_relay        │ Cyclone │  • pymycobot ↔ /dev/ttyTHS1│
│  • robot_state_publisher    │ peer    │                             │
│                             │ static  │                             │
│  192.168.0.79               │         │  192.168.0.250             │
└─────────────────────────────┘         └─────────────────────────────┘
                                                       │
                                                       ▼
                                            ┌──────────────────┐
                                            │  MyCobot 280 JN  │
                                            │  (hardware real) │
                                            └──────────────────┘
```

---

## Como rodar

### Pré-requisitos
- Docker e docker-compose no PC
- `sshpass` instalado no PC: `sudo apt install sshpass`
- Nano alcançável em `192.168.0.250` com user `er` / senha `Elephant`
- Bridge instalado no Nano em `~/custom_ws` (ver "Setup inicial do Nano" abaixo)

### Uso normal — um único comando

```bash
./mycobot_docker/RUN_PLANNING_PC.sh
```

Esse script faz tudo:
1. **Reinicia o bridge no Nano** com estado DDS limpo (via SSH+setsid)
2. **Limpa processos ros2 antigos** no container Docker
3. **Lança MoveIt 2 + RViz** no container

Em seguida, no RViz:
- Arrasta a **bola laranja** (rings/setas) ao redor do efetuador para definir uma pose-alvo
- Clica **"Plan"** para visualizar a trajetória planejada
- Clica **"Plan & Execute"** — o braço físico se move

### Quando alterar código do bridge

Se editar `mycobot_bridge.py`, `joint_state_relay.py`, ou outros arquivos em
`mycobot_docker/custom_ws/src/mycobot_hw_interface/`, sincroniza para o Nano:

```bash
./mycobot_docker/DEPLOY_TO_NANO.sh
```

Isso faz rsync, rebuild no Nano, e restart do bridge.

### Build do container Docker (primeira vez)

```bash
cd mycobot_docker
docker compose build
docker compose up -d
docker exec mycobot_ros2 bash -c "source /opt/ros/galactic/setup.bash && cd /root/custom_ws && colcon build --symlink-install"
```

---

## Arquitetura

### Fluxo de dados

```
[Físico] MyCobot 280 JN
   ↑↓ serial 1Mbps via /dev/ttyTHS1
[Nano] mycobot_bridge (pymycobot)
   • publica /joint_states_raw a 10Hz (pose atual)
   • action server /mycobot_arm_controller/follow_joint_trajectory
   ↑↓ DDS (CycloneDDS, peer estático)
[Docker PC] joint_state_relay
   • re-carimba timestamps com clock do PC (evita "stale states" do MoveIt)
   • publica /joint_states
[Docker PC] robot_state_publisher
   • lê /joint_states → publica TF para todos os links
[Docker PC] move_group (MoveIt)
   • lê /joint_states + URDF/SRDF
   • envia trajetórias planejadas → action server do bridge
[Docker PC] RViz
   • visualização + interação (arrastar bola, Plan & Execute)
```

### Por que `joint_state_relay`?

O Nano e o PC podem ter relógios não-sincronizados. Quando o bridge carimba uma
mensagem com o tempo do Nano e o MoveIt no PC compara com seu relógio local, ele
pode rejeitar a mensagem como "stale" (velha demais). O relay simplesmente
re-carimba a mensagem com o clock do PC antes de republicar como `/joint_states`.

### Configuração CycloneDDS

Ambos os lados usam **multicast habilitado** (default) para descoberta local
entre processos da mesma máquina, e **peer estático** para descoberta
cross-machine garantida via unicast UDP (mesmo se o switch não suportar
multicast bem).

- PC: `mycobot_docker/custom_ws/cyclonedds_pc.xml` → peer `192.168.0.250`
- Nano: `~/cyclonedds.xml` → peer `192.168.0.79`

Ambos usam `ROS_DOMAIN_ID=42`.

---

## Bugs corrigidos nesta branch (commits `8326dce` → `a70021b`)

### 1. XACRO com origens de junta erradas (`8326dce`)
O XACRO em `mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn.urdf.xacro`
tinha as 6 origens de junta deslocadas em uma posição (cada junta herdou a
origem da junta anterior do URDF original) e a primeira junta estava em
`0 0 0.0706` em vez de `0 0 0.15756`. Resultado: robô completamente
"transfigurado" no RViz.

Fix: substituídas as 6 definições de junta com origens e limites do
`mycobot_280_jn.urdf` original (commit `90414a7`).

### 2. CycloneDDS não descobria tópicos (`8326dce`)
`<NetworkInterfaceAddress>` + `<AllowMulticast>false</AllowMulticast>` no
`cyclonedds_pc.xml` excluía a interface loopback, impedindo descoberta local
E cross-machine.

Fix: removido `<General>` inteiro do XML (CycloneDDS usa defaults: multicast
habilitado para descoberta local, peer estático garante unicast cross-machine).
Mesma alteração no `~/cyclonedds.xml` do Nano.

### 3. Inconsistência de joint names (`8326dce`)
Bridge, controllers, joint_limits, SRDF e RViz config usavam nomes diferentes
em locais diferentes (`joint1` vs `cobot_joint_1` vs `joint2_to_joint1`).

Fix: tudo padronizado para `joint2_to_joint1`, `joint3_to_joint2`, ...,
`joint6output_to_joint6` (nomes oficiais elephantrobotics).

### 4. Marcador interativo do MoveIt invisível (`aab1f7f`)
`Interactive Marker Size: 0` no `moveit.rviz` ocultava completamente a bola/
rings que permite arrastar a pose-alvo.

Fix: setado para `0.15` (15cm), visível para um braço de ~280mm.

### 5. Bridge no Nano morria com SIGKILL (`8effc7e`)
O `start_bridge.sh` no Nano tinha `pkill -9 -u er -f python3` no início, que
matava QUALQUER processo python3 do user `er` — incluindo um bridge previamente
iniciado se o script fosse re-executado.

Fix: substituído `pkill -9 -u er -f python3` por `pkill -9 -f mycobot_bridge`
(targeted). Adicionado `setsid` no `RUN_PLANNING_PC.sh` para fazer o bridge
sobreviver ao SIGHUP do fim da sessão SSH.

### 6. Bridge instalado no Nano era um stub (`a70021b`)
O `mycobot_bridge.py` instalado em `~/custom_ws/install/` no Nano tinha um
`execute_callback` placeholder que retornava success sem nunca chamar
`self.mc.send_angles()`. MoveIt enviava trajetória → bridge dizia "ok" → braço
nunca se movia.

Fix: criado `DEPLOY_TO_NANO.sh` que sincroniza o código correto do PC para o
Nano, rebuilda lá e reinicia o bridge.

### 7. Estado DDS preso bloqueava entrega de dados
CycloneDDS retém referências a subscribers mortos com `kill -9`. Subscribers
novos viam o tópico via discovery mas nunca recebiam dados.

Fix: `RUN_PLANNING_PC.sh` agora reinicia o bridge no Nano antes de cada launch,
limpando o estado DDS.

---

## Setup inicial do Nano (uma vez só)

Se o bridge ainda não está instalado no Nano:

```bash
# No PC
./mycobot_docker/DEPLOY_TO_NANO.sh
```

Antes do primeiro deploy, garante que o Nano tem:
- ROS 2 Galactic instalado
- `~/custom_ws/src/` criado (`mkdir -p ~/custom_ws/src`)
- pymycobot: `pip3 install pymycobot --upgrade`
- CycloneDDS: `sudo apt install ros-galactic-rmw-cyclonedds-cpp`
- `~/cyclonedds.xml` com peer `192.168.0.79` (ver template em
  `mycobot_docker/custom_ws/cyclonedds_pc.xml` e adapta IP)
- `~/start_bridge.sh` (criado pelo `RUN_PLANNING_PC.sh` na primeira execução,
  mas pode ser criado manualmente — ver template em commit `8effc7e`)

---

## Troubleshooting

### Robô aparece em pose zero no RViz
`/joint_states` está vazio. Causa típica: bridge no Nano morreu.
```bash
sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/bridge.log'
```
Solução: re-rodar `RUN_PLANNING_PC.sh` (ele reinicia o bridge).

### Plan & Execute diz "Executed" mas o robô não se move
Bridge instalado no Nano é stub antigo. Roda:
```bash
./mycobot_docker/DEPLOY_TO_NANO.sh
```

### Tópicos visíveis com `ros2 topic list` mas dados não fluem
CycloneDDS com estado preso. Reinicia o bridge:
```bash
./mycobot_docker/RUN_PLANNING_PC.sh   # já faz isso automaticamente
```

### "Invalid frame ID 'link1' - frame does not exist" no RViz
robot_state_publisher não está recebendo `/joint_states`. Verifica:
```bash
docker exec mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=/root/custom_ws/cyclonedds_pc.xml
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 topic echo /joint_states 2>&1 | head -10
"
```
Se vazio: bridge morto, re-roda `RUN_PLANNING_PC.sh`.

---

## Estrutura do repositório

```
mycobot_docker/
├── docker-compose.yml          # Container com host network + X11
├── Dockerfile                  # ROS 2 Galactic + MoveIt + pymycobot
├── RUN_PLANNING_PC.sh          # Script principal (bridge + MoveIt)
├── RUN_ROBOT_NANO.sh           # SSH + start_bridge no Nano
├── DEPLOY_TO_NANO.sh           # Sincroniza código do bridge para o Nano
└── custom_ws/                  # Workspace ROS 2 (volume mount)
    ├── cyclonedds_pc.xml       # Config DDS do PC
    └── src/
        ├── mycobot_description/        # URDF/XACRO + meshes
        ├── mycobot_280_jn_moveit_config/  # SRDF, controllers, RViz
        └── mycobot_hw_interface/       # mycobot_bridge + joint_state_relay
```
