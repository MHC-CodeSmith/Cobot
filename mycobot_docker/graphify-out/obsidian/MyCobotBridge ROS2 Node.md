---
source_file: "custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py"
type: "code"
community: "MoveIt / MyCobotArm URDF & Launch"
tags:
  - graphify/code
  - graphify/EXTRACTED
  - community/MoveIt_/_MyCobotArm_URDF__Launch
---

# MyCobotBridge ROS2 Node

## Connections
- [[JointStateRelay ROS2 Node]] - `shares_data_with` [INFERRED]
- [[ROS2 action mycobot_arm_controllerfollow_joint_trajectory]] - `implements` [EXTRACTED]
- [[ROS2 topic joint_states_commands]] - `calls` [EXTRACTED]
- [[ROS2 topic joint_states_raw]] - `calls` [EXTRACTED]
- [[pymycobot.mycobot280.MyCobot280 (hardware driver)]] - `calls` [EXTRACTED]

#graphify/code #graphify/EXTRACTED #community/MoveIt_/_MyCobotArm_URDF__Launch