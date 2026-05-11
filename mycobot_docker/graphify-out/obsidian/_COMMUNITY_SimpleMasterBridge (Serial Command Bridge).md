---
type: community
cohesion: 0.18
members: 17
---

# SimpleMasterBridge (Serial Command Bridge)

**Cohesion:** 0.18 - loosely connected
**Members:** 17 nodes

## Members
- [[.__init__()_8]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._async_sleep()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._build_reorder_map()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._cancel_callback()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._execute_trajectory()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._goal_callback()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._publish_joint_states()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._read_angles()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._reorder()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[._wait_for_arrival()]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[Aguarda até o robô atingir o alvo ou esgotar o timeout.]] - rationale - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[Mapeia índice de incoming_names → índice em JOINT_NAMES.]] - rationale - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[Retorna lista de 6 floats (graus) ou None se falhar.]] - rationale - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[SimpleMasterBridge]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[Sleep não-bloqueante compatível com o executor rclpy.]] - rationale - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[main()_10]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py
- [[simple_master_bridge.py]] - code - mycobot_hw_interface/mycobot_hw_interface/simple_master_bridge.py

## Live Query (requires Dataview plugin)

```dataview
TABLE source_file, type FROM #community/SimpleMasterBridge_Serial_Command_Bridge
SORT file.name ASC
```

## Connections to other communities
- 1 edge to [[_COMMUNITY_MoveIt  MyCobotArm URDF & Launch]]
- 1 edge to [[_COMMUNITY_JointStateRelay (Joint Feedback)]]

## Top bridge nodes
- [[SimpleMasterBridge]] - degree 13, connects to 1 community
- [[simple_master_bridge.py]] - degree 3, connects to 1 community