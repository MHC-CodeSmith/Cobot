#!/usr/bin/env python3
# ============================================================
# move_arm.py — Utilitário de movimentação para testes da visão
# ============================================================
import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningOptions, Constraints, JointConstraint, MotionPlanRequest

JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GROUP = "mycobot_arm"
POSES_FILE = "/root/custom_ws/config/test_table_poses.yaml"
SAVED_INITIAL_FILE = "/tmp/yolo_test_initial_pose.yaml"

target_arg = sys.argv[1] if len(sys.argv) > 1 else "scan"

rclpy.init()
node = rclpy.create_node("move_arm_helper")
move_cli = ActionClient(node, MoveGroup, "/move_action")

current_joints = None
def js_cb(msg):
    global current_joints
    if set(JOINT_NAMES).issubset(set(msg.name)):
        idx = {n: i for i, n in enumerate(msg.name)}
        current_joints = [msg.position[idx[n]] for n in JOINT_NAMES]

node.create_subscription(JointState, "/joint_states", js_cb, 10)

if not move_cli.wait_for_server(timeout_sec=5.0):
    print("ERRO: MoveGroup /move_action indisponível.")
    sys.exit(1)

t0 = time.time()
while current_joints is None and time.time() - t0 < 3.0:
    rclpy.spin_once(node, timeout_sec=0.1)

if current_joints is None:
    print("ERRO: Não foi possível obter /joint_states.")
    sys.exit(1)

if target_arg == "save_initial":
    with open(SAVED_INITIAL_FILE, "w") as f:
        yaml.safe_dump({"initial": current_joints}, f)
    print("✓ Pose inicial salva para restauração posterior.")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)

target_joints = None
if target_arg == "restore":
    if os.path.exists(SAVED_INITIAL_FILE):
        try:
            with open(SAVED_INITIAL_FILE) as f:
                data = yaml.safe_load(f)
                target_joints = data.get("initial")
        except Exception:
            pass
    if target_joints is None:
        target_arg = "home"

if target_joints is None:
    if not os.path.exists(POSES_FILE):
        print(f"ERRO: {POSES_FILE} não existe.")
        sys.exit(1)
    with open(POSES_FILE) as f:
        poses = yaml.safe_load(f) or {}
    if target_arg not in poses:
        print(f"ERRO: Pose '{target_arg}' não encontrada em {POSES_FILE}.")
        sys.exit(1)
    target_joints = poses[target_arg]

mpr = MotionPlanRequest()
mpr.group_name = GROUP
mpr.allowed_planning_time = 5.0
mpr.num_planning_attempts = 5
mpr.max_velocity_scaling_factor = 0.20
mpr.max_acceleration_scaling_factor = 0.20
mpr.start_state.joint_state.name = list(JOINT_NAMES)
mpr.start_state.joint_state.position = [float(v) for v in current_joints]

c = Constraints()
for n, p in zip(JOINT_NAMES, target_joints):
    jc = JointConstraint()
    jc.joint_name = n
    jc.position = float(p)
    jc.tolerance_above = jc.tolerance_below = 0.01
    jc.weight = 1.0
    c.joint_constraints.append(jc)
mpr.goal_constraints = [c]

po = PlanningOptions()
po.plan_only = False

goal = MoveGroup.Goal()
goal.request = mpr
goal.planning_options = po

print(f"Movendo robô para pose '{target_arg}'...")
send_goal_fut = move_cli.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_goal_fut, timeout_sec=5.0)
gh = send_goal_fut.result()
if gh and gh.accepted:
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=10.0)
    print(f"✓ Chegou na pose '{target_arg}'.")

node.destroy_node()
rclpy.shutdown()
