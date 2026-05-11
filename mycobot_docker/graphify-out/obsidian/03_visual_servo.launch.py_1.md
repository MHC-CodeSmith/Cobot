---
source_file: "custom_ws/src/mycobot_vision_teleop/launch/03_visual_servo.launch.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "1-144"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# 03_visual_servo.launch.py

## Connections
- [[FaceDetectorNode_1]] - `calls` [EXTRACTED]
- [[VisualServoControllerNode_1]] - `calls` [EXTRACTED]
- [[joint_state_relay node (mycobot_hw_interface)]] - `calls` [EXTRACTED]
- [[static_transform_publisher link6 - camera_optical_frame]] - `calls` [EXTRACTED]
- [[visual_servo.yaml config]] - `references` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline