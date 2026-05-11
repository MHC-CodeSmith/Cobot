# Graph Report - custom_ws/src  (2026-05-11)

## Corpus Check
- Corpus is ~9,875 words - fits in a single context window. You may not need a graph.

## Summary
- 182 nodes · 244 edges · 24 communities (21 shown, 3 thin omitted)
- Extraction: 93% EXTRACTED · 5% INFERRED · 2% AMBIGUOUS · INFERRED: 12 edges (avg confidence: 0.83)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Eye-in-Hand Visual Servo Pipeline|Eye-in-Hand Visual Servo Pipeline]]
- [[_COMMUNITY_MoveIt  MyCobotArm URDF & Launch|MoveIt / MyCobotArm URDF & Launch]]
- [[_COMMUNITY_FaceFollower Legacy Servo|FaceFollower Legacy Servo]]
- [[_COMMUNITY_VisualServoController (IBVS + EMA + State Machine)|VisualServoController (IBVS + EMA + State Machine)]]
- [[_COMMUNITY_SimpleMasterBridge (Serial Command Bridge)|SimpleMasterBridge (Serial Command Bridge)]]
- [[_COMMUNITY_JointStateRelay (Joint Feedback)|JointStateRelay (Joint Feedback)]]
- [[_COMMUNITY_VisionNode (MediaPipe FaceMesh)|VisionNode (MediaPipe FaceMesh)]]
- [[_COMMUNITY_MyCobotBridge (Low-Level HW Interface)|MyCobotBridge (Low-Level HW Interface)]]
- [[_COMMUNITY_FaceDetectorNode (FaceMesh + Multi-Face Select)|FaceDetectorNode (FaceMesh + Multi-Face Select)]]
- [[_COMMUNITY_ArmCameraNode (Camera Capture)|ArmCameraNode (Camera Capture)]]
- [[_COMMUNITY_generate_launch_description()  galactic_demo.launch.py|generate_launch_description() / galactic_demo.launch.py]]
- [[_COMMUNITY_02_face_tracking.launch.py  generate_launch_description()|02_face_tracking.launch.py / generate_launch_description()]]
- [[_COMMUNITY_03_visual_servo.launch.py  generate_launch_description()|03_visual_servo.launch.py / generate_launch_description()]]

## God Nodes (most connected - your core abstractions)
1. `VisualServoControllerNode` - 16 edges
2. `VisualServoControllerNode` - 15 edges
3. `SimpleMasterBridge` - 13 edges
4. `FaceFollowerNode` - 13 edges
5. `FaceFollowerNode` - 12 edges
6. `VisionNode` - 11 edges
7. `FaceDetectorNode` - 10 edges
8. `VisionNode` - 9 edges

## Surprising Connections (you probably didn't know these)
- `joint6.png (URDF visual — end-effector connector GPIO: 5V0, GND, 3V3, G22, G19, G23, G33)` --conceptually_related_to--> `SimpleMasterBridge ROS2 Node`  [INFERRED]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint6.png → custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- `visual_servo.yaml config` --references--> `FaceDetectorNode`  [EXTRACTED]
  custom_ws/src/mycobot_vision_teleop/config/visual_servo.yaml → custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- `visual_servo.yaml config` --references--> `VisualServoControllerNode`  [EXTRACTED]
  custom_ws/src/mycobot_vision_teleop/config/visual_servo.yaml → custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- `mycobot_vision_teleop ROS2 package` --references--> `target_follower_node (stub - Session C)`  [EXTRACTED]
  custom_ws/src/mycobot_vision_teleop/setup.py → custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/target_follower_node.py
- `FaceDetectorNode` --shares_data_with--> `VisualServoControllerNode`  [INFERRED]
  custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py → custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py

## Communities (24 total, 3 thin omitted)

### Community 0 - "Eye-in-Hand Visual Servo Pipeline"
Cohesion: 0.09
Nodes (33): ArmCameraNode ROS2 Node, arm_mapper_node (stub - Session B), Commanded-position integrator (anti-jitter fix), S-curve ease function for smooth tracking response, EMA - Exponential Moving Average filter, Eye-in-hand camera configuration (camera on wrist link6), IBVS - Image-Based Visual Servo, MediaPipe FaceMesh (landmark 4 = nose tip as face center) (+25 more)

### Community 1 - "MoveIt / MyCobotArm URDF & Launch"
Cohesion: 0.16
Nodes (14): ROS2 action: mycobot_arm_controller/follow_joint_trajectory, joint2.png (URDF visual — blank), joint3.png (URDF visual — blank), joint4.png (URDF visual — blank), joint5.png (URDF visual — blank), joint6.png (URDF visual — end-effector connector GPIO: 5V0, GND, 3V3, G22, G19, G23, G33), JointStateRelay ROS2 Node, kdl_kinematics_plugin/KDLKinematicsPlugin (+6 more)

### Community 2 - "FaceFollower Legacy Servo"
Cohesion: 0.17
Nodes (8): clamp(), ease_error(), _ease_inout_cubic(), FaceFollowerNode, main(), Extrapola na última direção de ex para tentar re-encontrar o rosto., S-curve: suave perto do centro, agressivo longe., Mapeia erro → saída eased em [-1,1], zero dentro do deadband.     max_error alin

### Community 3 - "VisualServoController (IBVS + EMA + State Machine)"
Cohesion: 0.18
Nodes (5): clamp(), main(), Reseta toda a state machine e integrador., State, VisualServoControllerNode

### Community 4 - "SimpleMasterBridge (Serial Command Bridge)"
Cohesion: 0.18
Nodes (6): main(), Retorna lista de 6 floats (graus) ou None se falhar., Mapeia índice de incoming_names → índice em JOINT_NAMES., Sleep não-bloqueante compatível com o executor rclpy., Aguarda até o robô atingir o alvo ou esgotar o timeout., SimpleMasterBridge

### Community 5 - "JointStateRelay (Joint Feedback)"
Cohesion: 0.2
Nodes (6): JointStateRelay, main(), Relay node to fix timestamp synchronization issues in distributed ROS 2 systems., main(), ShowFace, Node

### Community 6 - "VisionNode (MediaPipe FaceMesh)"
Cohesion: 0.33
Nodes (3): main(), Detecta rosto via MediaPipe FaceMesh e publica posição normalizada do nariz., VisionNode

### Community 7 - "MyCobotBridge (Low-Level HW Interface)"
Cohesion: 0.25
Nodes (4): main(), MyCobotBridge, Controle direto para face tracking — executa imediatamente., Executa trajectória do MoveIt (plan+execute do RViz).

### Community 9 - "ArmCameraNode (Camera Capture)"
Cohesion: 0.32
Nodes (3): ArmCameraNode, main(), Captura frames continuamente e armazena o mais recente.

### Community 10 - "generate_launch_description() / galactic_demo.launch.py"
Cohesion: 0.83
Nodes (3): generate_launch_description(), load_file(), load_yaml()

## Ambiguous Edges - Review These
- `joint2.png (URDF visual — blank)` → `joint_limits.yaml`  [AMBIGUOUS]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint2.png · relation: conceptually_related_to
- `joint3.png (URDF visual — blank)` → `joint_limits.yaml`  [AMBIGUOUS]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint3.png · relation: conceptually_related_to
- `joint4.png (URDF visual — blank)` → `joint_limits.yaml`  [AMBIGUOUS]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint4.png · relation: conceptually_related_to
- `joint5.png (URDF visual — blank)` → `joint_limits.yaml`  [AMBIGUOUS]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint5.png · relation: conceptually_related_to
- `joint6.png (URDF visual — end-effector connector GPIO: 5V0, GND, 3V3, G22, G19, G23, G33)` → `joint_limits.yaml`  [AMBIGUOUS]
  custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint6.png · relation: conceptually_related_to

## Knowledge Gaps
- **32 isolated node(s):** `S-curve: suave perto do centro, agressivo longe.`, `Mapeia erro → saída eased em [-1,1], zero dentro do deadband.     max_error alin`, `Extrapola na última direção de ex para tentar re-encontrar o rosto.`, `State`, `Reseta toda a state machine e integrador.` (+27 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **3 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `joint2.png (URDF visual — blank)` and `joint_limits.yaml`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `joint3.png (URDF visual — blank)` and `joint_limits.yaml`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `joint4.png (URDF visual — blank)` and `joint_limits.yaml`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `joint5.png (URDF visual — blank)` and `joint_limits.yaml`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What is the exact relationship between `joint6.png (URDF visual — end-effector connector GPIO: 5V0, GND, 3V3, G22, G19, G23, G33)` and `joint_limits.yaml`?**
  _Edge tagged AMBIGUOUS (relation: conceptually_related_to) - confidence is low._
- **What connects `S-curve: suave perto do centro, agressivo longe.`, `Mapeia erro → saída eased em [-1,1], zero dentro do deadband.     max_error alin`, `Extrapola na última direção de ex para tentar re-encontrar o rosto.` to the rest of the system?**
  _32 weakly-connected nodes found - possible documentation gaps or missing edges._