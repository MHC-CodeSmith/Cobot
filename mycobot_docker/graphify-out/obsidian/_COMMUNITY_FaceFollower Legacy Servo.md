---
type: community
cohesion: 0.17
members: 18
---

# FaceFollower Legacy Servo

**Cohesion:** 0.17 - loosely connected
**Members:** 18 nodes

## Members
- [[.__init__()_1]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._control_loop()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._do_search()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._do_track()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._enable_cb()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._face_cb()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._js_cb()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._send_joints()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[._tracking_cb()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[Extrapola na última direção de ex para tentar re-encontrar o rosto.]] - rationale - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[FaceFollowerNode]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[Mapeia erro → saída eased em -1,1, zero dentro do deadband.     max_error alin]] - rationale - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[S-curve suave perto do centro, agressivo longe.]] - rationale - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[_ease_inout_cubic()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[clamp()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[ease_error()]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[face_follower_node.py]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py
- [[main()_1]] - code - mycobot_vision_teleop/mycobot_vision_teleop/face_follower_node.py

## Live Query (requires Dataview plugin)

```dataview
TABLE source_file, type FROM #community/FaceFollower_Legacy_Servo
SORT file.name ASC
```

## Connections to other communities
- 1 edge to [[_COMMUNITY_JointStateRelay (Joint Feedback)]]

## Top bridge nodes
- [[FaceFollowerNode]] - degree 12, connects to 1 community