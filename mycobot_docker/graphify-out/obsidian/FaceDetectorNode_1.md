---
source_file: "custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "50-184"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# FaceDetectorNode

## Connections
- [[MediaPipe FaceMesh (landmark 4 = nose tip as face center)]] - `implements` [EXTRACTED]
- [[ROS topic face_detectioncenter (PointStamped)]] - `calls` [EXTRACTED]
- [[ROS topic face_detectiondetected (Bool)]] - `calls` [EXTRACTED]
- [[ROS topic face_detectionimage_debug (Image)]] - `calls` [EXTRACTED]
- [[ROS2 topic arm_cameraimage_raw]] - `references` [EXTRACTED]
- [[VisionNode_1]] - `conceptually_related_to` [INFERRED]
- [[VisualServoControllerNode_1]] - `shares_data_with` [INFERRED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline