---
source_file: "custom_ws/src/mycobot_vision_teleop/launch/02_face_tracking.launch.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "1-88"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# 02_face_tracking.launch.py

## Connections
- [[FaceFollowerNode_1]] - `calls` [EXTRACTED]
- [[VisionNode_1]] - `calls` [EXTRACTED]
- [[joint_state_relay node (mycobot_hw_interface)]] - `calls` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline