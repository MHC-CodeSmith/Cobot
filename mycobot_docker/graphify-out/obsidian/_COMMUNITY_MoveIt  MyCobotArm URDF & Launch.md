---
type: community
cohesion: 0.16
members: 19
---

# MoveIt / MyCobotArm URDF & Launch

**Cohesion:** 0.16 - loosely connected
**Members:** 19 nodes

## Members
- [[JointStateRelay ROS2 Node]] - code - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/joint_state_relay.py
- [[MoveItSimpleControllerManager]] - rationale - custom_ws/src/mycobot_280_jn_moveit_config/config/moveit_controllers.yaml
- [[MyCobotBridge ROS2 Node]] - code - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py
- [[ROS2 action mycobot_arm_controllerfollow_joint_trajectory]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py
- [[ROS2 topic joint_states_raw]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py
- [[SimpleMasterBridge ROS2 Node]] - code - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[galactic_demo.launch.py_1]] - code - custom_ws/src/mycobot_280_jn_moveit_config/launch/galactic_demo.launch.py
- [[joint2.png (URDF visual — blank)]] - image - custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint2.png
- [[joint3.png (URDF visual — blank)]] - image - custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint3.png
- [[joint4.png (URDF visual — blank)]] - image - custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint4.png
- [[joint5.png (URDF visual — blank)]] - image - custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint5.png
- [[joint6.png (URDF visual — end-effector connector GPIO 5V0, GND, 3V3, G22, G19, G23, G33)]] - image - custom_ws/src/mycobot_description/urdf/mycobot_280_jn/joint6.png
- [[joint_limits.yaml]] - document - custom_ws/src/mycobot_280_jn_moveit_config/config/joint_limits.yaml
- [[kdl_kinematics_pluginKDLKinematicsPlugin]] - rationale - custom_ws/src/mycobot_280_jn_moveit_config/config/kinematics.yaml
- [[kinematics.yaml]] - document - custom_ws/src/mycobot_280_jn_moveit_config/config/kinematics.yaml
- [[moveit_controllers.yaml]] - document - custom_ws/src/mycobot_280_jn_moveit_config/config/moveit_controllers.yaml
- [[mycobot_hw.launch.py_1]] - code - custom_ws/src/mycobot_hw_interface/launch/mycobot_hw.launch.py
- [[mycobot_hw_interface setup.py]] - code - custom_ws/src/mycobot_hw_interface/setup.py
- [[pymycobot.mycobot280.MyCobot280 (hardware driver)]] - rationale - custom_ws/src/mycobot_hw_interface/mycobot_hw_interface/mycobot_bridge.py

## Live Query (requires Dataview plugin)

```dataview
TABLE source_file, type FROM #community/MoveIt_/_MyCobotArm_URDF__Launch
SORT file.name ASC
```

## Connections to other communities
- 3 edges to [[_COMMUNITY_Eye-in-Hand Visual Servo Pipeline]]
- 1 edge to [[_COMMUNITY_MyCobotBridge (Low-Level HW Interface)]]
- 1 edge to [[_COMMUNITY_SimpleMasterBridge (Serial Command Bridge)]]

## Top bridge nodes
- [[MyCobotBridge ROS2 Node]] - degree 8, connects to 1 community
- [[JointStateRelay ROS2 Node]] - degree 6, connects to 1 community
- [[mycobot_hw_interface setup.py]] - degree 3, connects to 1 community