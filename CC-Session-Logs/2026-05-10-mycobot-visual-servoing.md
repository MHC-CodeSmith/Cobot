# Session: MyCobot 280 JN — Visual Servoing & Face Tracking
**Date:** 2026-05-10  
**Branch:** master  
**Last commit:** 25d9a65

---

## Estado Final do Sistema

### Arquitetura de controle (implementada e commitada)

```
Camera (Nano /dev/video0)
  → arm_camera_node (Nano) → /arm_camera/image_raw (CycloneDDS)
  → vision_node (Docker/PC)
      - MediaPipe Pose → nose landmark normalizado
      - Publica: /human/face_center (Point), /human/tracking_ok (Bool)
      - Overlay no debug image: retângulo alvo verde/laranja + ponto nariz
  → face_follower_node (Docker/PC)
      - Easing cúbico In/Out sobre o erro (suave perto do centro)
      - Publica JointState → /joint_states_commands (DIRETO, sem action)
  → mycobot_bridge (Nano)
      - set_fresh_mode(1): sempre executa o mais recente, descarta fila
      - tracking_speed=30 (suave), moveit_speed=60 (MoveIt plan+execute)
      - mc.send_angles(angles, speed)
```

### DDS: CycloneDDS em tudo
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=/root/custom_ws/cyclonedds_pc.xml` (peer=192.168.0.250)
- `ROS_DOMAIN_ID=42`

---

## Arquivos Modificados (commits 65b27e3 → 25d9a65)

### Novos arquivos
| Arquivo | Descrição |
|---------|-----------|
| `mycobot_docker/custom_ws/src/mycobot_vision_teleop/` | Pacote ROS2 completo — vision + face follower |
| `mycobot_docker/custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/arm_camera_node.py` | Câmera no Nano → publica /arm_camera/image_raw |
| `mycobot_docker/RUN_ROBOT_EYE.sh` | All-in-one: câmera Nano + vision + follower + habilita + logs |
| `mycobot_docker/RUN_LAPTOP_3D.sh` | Placeholder para modo laptop webcam (futuro) |
| `mycobot_docker/DEPLOY_TO_NANO.sh` | Sync + rebuild + restart bridge no Nano via SSH |

### Arquivos modificados
| Arquivo | O que mudou |
|---------|-------------|
| `mycobot_bridge.py` | +set_fresh_mode(1), +tracking_speed=30, +moveit_speed=60, +cancel check |
| `face_follower_node.py` | Mudou de FollowJointTrajectory action → JointState publish direto; +easing cúbico |
| `vision_node.py` | +dual mode (câmera local/tópico), +_draw_target_overlay() |
| `galactic_demo.launch.py` | +trajectory_execution timeout scaling=5x, margin=10s |
| `02_face_tracking.launch.py` | +use_arm_camera arg, +target_window param |
| `moveit.rviz` | +Image panel "Olho do Robo" (/human/image_debug), cores laranja goal |

---

## Bugs Corrigidos

### 1. MoveIt não funcionava (DDS mismatch)
- **Causa**: `RUN_PLANNING_PC.sh` usava FastDDS Discovery Server; bridge do Nano usa CycloneDDS
- **Fix**: Reescrito para CycloneDDS em tudo

### 2. SSH travava em [1/3] Nano bridge restart
- **Causa**: processo filho herdava fds do canal SSH
- **Fix**: `bash -c "nohup ... &" </dev/null >/dev/null 2>/dev/null`

### 3. Movimento robótico/travado no face tracking
- **Causa 1**: face_follower usava FollowJointTrajectory action → fila de goals acumulava
- **Causa 2**: speed=80 muito rápido (overshoot + brusco)
- **Causa 3**: bridge não tinha set_fresh_mode → executava goals antigos
- **Fix**: Direto via `/joint_states_commands` + set_fresh_mode(1) + speed=30

### 4. MoveIt Plan+Execute TIMED_OUT (0.5s)
- **Causa**: sem config de trajectory_execution, timeout default = 0.5s; robô físico leva 1-3s
- **Fix**: `allowed_execution_duration_scaling=5.0`, `allowed_goal_duration_margin=10.0`

### 5. Terminal do RUN_ROBOT_EYE.sh fechava
- **Fix**: `docker exec mycobot_ros2 tail -f /tmp/teleop.log` no final

### 6. MoveIt goal state (laranja) invisível
- **Causa**: Scene Robot overlay com alpha=1 encobria o goal
- **Fix**: `Scene Robot: Robot Alpha: 0.0, Show Robot Visual: false`

---

## Parâmetros Chave

### face_follower_node
```
kp_x=0.5, kp_y=0.3         # ganhos proporcionais
deadband=0.06               # zona morta 6% da imagem
max_delta_rad=0.08          # passo máximo por ciclo (~4.6°)
rate_hz=20.0                # frequência de controle
j2_offset=0.4 rad           # inclinação base joint2
```

### mycobot_bridge
```
tracking_speed=30           # suave para visual servoing
moveit_speed=60             # para MoveIt plan+execute
set_fresh_mode(1)           # sempre executa comando mais recente
```

---

## Scripts de Uso

```bash
# 1. Primeira vez ou após mudar mycobot_bridge.py:
./mycobot_docker/DEPLOY_TO_NANO.sh

# 2. Iniciar MoveIt (terminal 1 — deixa aberto):
./mycobot_docker/RUN_PLANNING_PC.sh

# 3. Robot Eye — robô segue seu rosto (terminal 2):
./mycobot_docker/RUN_ROBOT_EYE.sh
# → faz rebuild Docker automático (~15s)
# → inicia câmera no Nano
# → inicia vision + follower no Docker
# → habilita movimento
# → abre showimage (se DISPLAY configurado)
# → fica aberto mostrando logs em tempo real
```

---

## Estado Pendente / Próximos Passos

1. **Testar movimento suave**: verificar se set_fresh_mode + speed=30 melhorou a fluidez
2. **Calibrar parâmetros**: kp_x, kp_y, deadband podem precisar de ajuste para a câmera específica
3. **invert_x**: dependendo da orientação da câmera no braço, pode precisar `invert_x=True`
4. **RUN_LAPTOP_3D.sh**: ainda é placeholder — estimativa 3D do braço via webcam não implementada
5. **arm_mapper_node.py + target_follower_node.py**: stubs para seguimento do pulso humano

---

## Notas de Infraestrutura

- **Docker**: `mycobot_ros2` container, volume mount `./custom_ws:/root/custom_ws:rw`
- **Nano IP**: 192.168.0.250, user: er, pass: Elephant
- **PC Docker IP**: 192.168.0.79
- **Git worktrees**: 
  - master = `~/Germany/Cobot/` (working dir do usuário)
  - `interesting-darwin-bb0b35` = CWD do Claude (commits vão para master)
  - `angry-agnesi-a90742` = montado no Docker (mesma fonte via volume)
- **Rebuild Python no Docker**: `colcon build --symlink-install --packages-select mycobot_vision_teleop`
- **Rebuild no Nano**: `DEPLOY_TO_NANO.sh` (rsync + colcon build + restart bridge)

---

## Nota sobre /graphify e /obsidian-save
Esses comandos não existem como skills neste ambiente Claude Code.  
Para exportar para Obsidian manualmente, copie este arquivo para:  
`/home/mhc/Documents/Obsidian_AI_Brain/Projects/MyCobot/`
