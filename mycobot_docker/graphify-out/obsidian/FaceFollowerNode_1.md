---
source_file: "custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py"
type: "code"
community: "Eye-in-Hand Visual Servo Pipeline"
location: "78-286"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/Eye-in-Hand_Visual_Servo_Pipeline
---

# FaceFollowerNode

## Connections
- [[Commanded-position integrator (anti-jitter fix)]] - `implements` [EXTRACTED]
- [[ROS topic face_followeractive (Bool)]] - `calls` [EXTRACTED]
- [[ROS topic face_followerenabled (Bool)]] - `references` [EXTRACTED]
- [[ROS topic humanface_center (Point)]] - `references` [EXTRACTED]
- [[ROS topic humantracking_ok (Bool)]] - `references` [EXTRACTED]
- [[ROS2 topic joint_states]] - `references` [EXTRACTED]
- [[ROS2 topic joint_states_commands]] - `calls` [EXTRACTED]
- [[S-curve ease function for smooth tracking response]] - `implements` [EXTRACTED]
- [[Search protocol extrapolate last pan direction on face loss]] - `implements` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/Eye-in-Hand_Visual_Servo_Pipeline