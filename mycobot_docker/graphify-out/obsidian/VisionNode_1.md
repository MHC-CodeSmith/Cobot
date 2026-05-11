---
source_file: "custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/vision_node.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "17-208"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# VisionNode

## Connections
- [[FaceFollowerNode_1]] - `shares_data_with` [INFERRED]
- [[MediaPipe FaceMesh (landmark 4 = nose tip as face center)]] - `implements` [EXTRACTED]
- [[ROS topic humanface_center (Point)]] - `calls` [EXTRACTED]
- [[ROS topic humanimage_debug (Image)]] - `calls` [EXTRACTED]
- [[ROS topic humantracking_ok (Bool)]] - `calls` [EXTRACTED]
- [[ROS2 topic arm_cameraimage_raw]] - `references` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline