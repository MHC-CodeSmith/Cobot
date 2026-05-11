---
type: community
cohesion: 0.09
members: 36
---

# Eye-in-Hand Visual Servo Pipeline

**Cohesion:** 0.09 - loosely connected
**Members:** 36 nodes

## Members
- [[01_vision_test.launch.py_1]] - code - custom_ws/src/mycobot_vision_teleop/launch/01_vision_test.launch.py
- [[02_face_tracking.launch.py_1]] - code - custom_ws/src/mycobot_vision_teleop/launch/02_face_tracking.launch.py
- [[03_visual_servo.launch.py_1]] - code - custom_ws/src/mycobot_vision_teleop/launch/03_visual_servo.launch.py
- [[ArmCameraNode ROS2 Node]] - code - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/arm_camera_node.py
- [[Commanded-position integrator (anti-jitter fix)]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[EMA - Exponential Moving Average filter]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[Eye-in-hand camera configuration (camera on wrist link6)]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[FaceDetectorNode_1]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- [[FaceFollowerNode_1]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[IBVS - Image-Based Visual Servo]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[MediaPipe FaceMesh (landmark 4 = nose tip as face center)]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- [[ROS topic face_detectioncenter (PointStamped)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- [[ROS topic face_detectiondetected (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- [[ROS topic face_detectionimage_debug (Image)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_detector_node.py
- [[ROS topic face_followeractive (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[ROS topic face_followerenabled (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[ROS topic humanface_center (Point)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/vision_node.py
- [[ROS topic humanimage_debug (Image)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/vision_node.py
- [[ROS topic humantracking_ok (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/vision_node.py
- [[ROS topic visual_servoactive (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[ROS topic visual_servoenabled (Bool)]] - document - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[ROS2 topic arm_cameraimage_raw]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/arm_camera_node.py
- [[ROS2 topic joint_states]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/joint_state_relay.py
- [[ROS2 topic joint_states_commands]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py
- [[S-curve ease function for smooth tracking response]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[Search protocol extrapolate last pan direction on face loss]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[ShowFace (debug visualizer)]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/show_face.py
- [[State machine WAITING_FACE  TRACKING  TEMPORARY_LOST]] - rationale - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[VisionNode_1]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/vision_node.py
- [[VisualServoControllerNode_1]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/visual_servo_controller_node.py
- [[arm_mapper_node (stub - Session B)]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/arm_mapper_node.py
- [[joint_state_relay node (mycobot_hw_interface)]] - code - custom_ws/src/mycobot_vision_teleop/launch/02_face_tracking.launch.py
- [[mycobot_vision_teleop ROS2 package]] - code - custom_ws/src/mycobot_vision_teleop/setup.py
- [[static_transform_publisher link6 - camera_optical_frame]] - code - custom_ws/src/mycobot_vision_teleop/launch/03_visual_servo.launch.py
- [[target_follower_node (stub - Session C)]] - code - custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/target_follower_node.py
- [[visual_servo.yaml config]] - document - custom_ws/src/mycobot_vision_teleop/config/visual_servo.yaml

## Live Query (requires Dataview plugin)

```dataview
TABLE source_file, type FROM #community/Eye-in-Hand_Visual_Servo_Pipeline
SORT file.name ASC
```

## Connections to other communities
- 3 edges to [[_COMMUNITY_MoveIt  MyCobotArm URDF & Launch]]
