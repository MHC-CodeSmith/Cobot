---
source_file: "custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "103-442"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# VisualServoControllerNode

## Connections
- [[Commanded-position integrator (anti-jitter fix)]] - `implements` [EXTRACTED]
- [[EMA - Exponential Moving Average filter]] - `implements` [EXTRACTED]
- [[Eye-in-hand camera configuration (camera on wrist link6)]] - `conceptually_related_to` [EXTRACTED]
- [[FaceFollowerNode_1]] - `conceptually_related_to` [INFERRED]
- [[IBVS - Image-Based Visual Servo]] - `implements` [EXTRACTED]
- [[ROS topic face_detectioncenter (PointStamped)]] - `references` [EXTRACTED]
- [[ROS topic face_detectiondetected (Bool)]] - `references` [EXTRACTED]
- [[ROS topic visual_servoactive (Bool)]] - `calls` [EXTRACTED]
- [[ROS topic visual_servoenabled (Bool)]] - `references` [EXTRACTED]
- [[ROS2 topic joint_states]] - `references` [EXTRACTED]
- [[ROS2 topic joint_states_commands]] - `calls` [EXTRACTED]
- [[State machine WAITING_FACE  TRACKING  TEMPORARY_LOST]] - `implements` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline