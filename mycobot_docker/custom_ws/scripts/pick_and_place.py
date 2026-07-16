#!/usr/bin/env python3
# ============================================================
# pick_and_place.py — rotina pick & place com a Suction Pump
#
# Sequência:
#   home → acima do pick → desce → PUMP ON (suga) → sobe →
#   aguarda a base chegar ao waypoint de entrega → acima do place →
#   desce → PUMP OFF (solta) → sobe → home
#
# Usa o move_group (que precisa estar rodando — RUN_MOCK_PC.sh
# ou RUN_PLANNING_PC.sh) para IK (/compute_ik) e planejamento
# (/plan_kinematic_path), e executa as trajetórias no bridge
# (action mycobot_arm_controller/follow_joint_trajectory).
# Funciona igual em mock e no robô real.
#
# Poses são do pump_tcp (face do copo de sucção) no frame
# mycobot_base_link, com o copo apontando para BAIXO.
# ============================================================
import argparse
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GROUP = "mycobot_arm"
TCP_LINK = "pump_tcp"
BASE_FRAME = "mycobot_base_link"


def cup_down_quat(yaw):
    """Quaternion para a orientação do TCP da ventosa.

    A pose do TCP em URDF já é definida com o copo apontando para fora do
    copo; aqui apenas variamos o yaw em torno do eixo vertical para manter o
    copo radial ao alvo detectado."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        self.ik_cli = self.create_client(GetPositionIK, "/compute_ik")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.pump_on_cli = self.create_client(Trigger, "pump_on")
        self.pump_off_cli = self.create_client(Trigger, "pump_off")
        self.traj_cli = ActionClient(
            self,
            FollowJointTrajectory,
            "mycobot_arm_controller/follow_joint_trajectory",
        )
        self.current = None
        self.latest_pick_pose = None
        self.latest_detection_label = None
        self.delivery_state = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(PointStamped, "/pick_detection_pose", self._on_pick_pose, 10)
        self.create_subscription(String, "/product_class", self._on_detection_label, 10)
        self.create_subscription(String, "/delivery_state", self._on_delivery_state, 10)

    def _js_cb(self, msg):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            idx = {n: i for i, n in enumerate(msg.name)}
            self.current = [msg.position[idx[n]] for n in JOINT_NAMES]

    def _on_pick_pose(self, msg):
        self.latest_pick_pose = msg

    def _on_detection_label(self, msg):
        self.latest_detection_label = (msg.data or "unknown").strip().lower()

    def _on_delivery_state(self, msg):
        self.delivery_state = (msg.data or "").strip().lower()

    # ── infra ────────────────────────────────────────────────────────
    def wait_ready(self, timeout=20.0):
        for cli, name in [
            (self.ik_cli, "/compute_ik"),
            (self.plan_cli, "/plan_kinematic_path"),
            (self.pump_on_cli, "pump_on"),
            (self.pump_off_cli, "pump_off"),
        ]:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Serviço {name} indisponível — o stack está rodando?")
        if not self.traj_cli.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Action server do bridge indisponível")
        t_end = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.current is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > t_end:
                raise RuntimeError("Sem /joint_states — bridge está rodando?")

    def wait_for_pick_target(self, timeout=30.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.latest_pick_pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError("Timeout aguardando /pick_detection_pose")
        return self.latest_pick_pose

    def wait_for_delivery_release(self, expected_state, timeout=60.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.delivery_state != expected_state:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().warn(
                    f"Timeout aguardando entrega {expected_state}; liberando objeto mesmo assim"
                )
                return False
        self.get_logger().info(f"Base chegou em {expected_state}; liberando objeto")
        return True

    def _call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    # ── blocos da rotina ─────────────────────────────────────────────
    def _ik_once(self, x, y, z, yaw, seed):
        pose = PoseStamped()
        pose.header.frame_id = BASE_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, z
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = cup_down_quat(yaw)

        req = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = GROUP
        r.ik_link_name = TCP_LINK
        r.pose_stamped = pose
        r.avoid_collisions = True
        r.robot_state.joint_state.name = list(JOINT_NAMES)
        r.robot_state.joint_state.position = [float(v) for v in seed]
        r.timeout.sec = 0
        r.timeout.nanosec = 200_000_000  # 0.2s por tentativa
        req.ik_request = r
        res = self._call(self.ik_cli, req)
        if res.error_code.val != 1:
            return None
        idx = {n: i for i, n in enumerate(res.solution.joint_state.name)}
        return [res.solution.joint_state.position[idx[n]] for n in JOINT_NAMES]

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _score(self, sol, base_yaw):
        """Menor = pose mais natural: junta 1 apontando p/ o alvo,
        pouco movimento desde o estado atual, punho pouco torcido."""
        s = sum(abs(self._wrap(q - c)) for q, c in zip(sol, self.current))
        s += 3.0 * min(
            abs(self._wrap(sol[0] - base_yaw)),
            abs(self._wrap(sol[0] - base_yaw + math.pi)),
        )
        s += 0.5 * (abs(sol[3]) + abs(sol[5]))  # punho reto é melhor
        return s

    def ik(self, x, y, z):
        """IK do pump_tcp com copo p/ baixo. O yaw do copo é livre:
        varre yaws e sementes, junta todas as soluções e escolhe a
        mais natural (evita poses contorcidas)."""
        base_yaw = math.atan2(y, x)
        yaws = [
            base_yaw,
            base_yaw + math.pi,
            0.0,
            math.pi / 2,
            -math.pi / 2,
            math.pi,
            base_yaw + math.pi / 2,
            base_yaw - math.pi / 2,
        ]
        seeds = [list(self.current), [base_yaw, 0, 0, 0, 0, 0], [base_yaw - math.pi, 0, 0, 0, 0, 0], [0.0] * 6]
        best, best_s = None, float("inf")
        for seed in seeds:
            for yaw in yaws:
                sol = self._ik_once(x, y, z, yaw, seed)
                if sol is None:
                    continue
                s = self._score(sol, base_yaw)
                if s < best_s:
                    best, best_s = sol, s
            if best is not None:
                break  # soluções desta semente já dão diversidade de yaw
        if best is None:
            raise RuntimeError(
                f"IK falhou p/ ({x:.3f},{y:.3f},{z:.3f}) após {len(seeds) * len(yaws)} tentativas"
            )
        return best

    def plan(self, target_joints):
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 8.0
        mpr.num_planning_attempts = 8
        mpr.max_velocity_scaling_factor = 0.25
        mpr.max_acceleration_scaling_factor = 0.25
        mpr.start_state.joint_state.name = list(JOINT_NAMES)
        mpr.start_state.joint_state.position = [float(v) for v in self.current]
        c = Constraints()
        for n, p in zip(JOINT_NAMES, target_joints):
            jc = JointConstraint()
            jc.joint_name = n
            jc.position = float(p)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        mpr.goal_constraints = [c]
        req.motion_plan_request = mpr
        res = self._call(self.plan_cli, req)
        if res.motion_plan_response.error_code.val != 1:
            raise RuntimeError(
                f"Planejamento falhou (error_code={res.motion_plan_response.error_code.val})"
            )
        return res.motion_plan_response.trajectory.joint_trajectory

    def execute(self, joint_traj):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        fut = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Trajetória rejeitada pelo bridge")
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut)
        last = joint_traj.points[-1].positions
        self.current = list(last)

    def move_to_pose(self, label, x, y, z):
        self.get_logger().info(f"→ {label}: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.execute(self.plan(self.ik(x, y, z)))

    def move_to_joints(self, label, joints):
        self.get_logger().info(f"→ {label} (joint-space)")
        self.execute(self.plan(joints))

    def pump(self, on):
        cli = self.pump_on_cli if on else self.pump_off_cli
        res = self._call(cli, Trigger.Request())
        state = "ON (sugando)" if on else "OFF (soltou)"
        if not res.success:
            raise RuntimeError(f"Pump falhou: {res.message}")
        self.get_logger().info(f"  Pump {state}")

    # ── rotina completa ──────────────────────────────────────────────
    def run(self, pick, place, hover, settle, delivery_timeout=60.0):
        px, py, pz = pick
        qx, qy, qz = place

        self.move_to_joints("home", [0.0] * 6)
        self.move_to_pose("acima do pick", px, py, pz + hover)
        self.move_to_pose("descendo no objeto", px, py, pz)
        self.pump(True)
        time.sleep(settle)
        self.move_to_pose("subindo com objeto", px, py, pz + hover)

        if self.latest_detection_label in {"red", "lata_vermelha", "vermelha"}:
            expected_delivery = "delivery_red"
        elif self.latest_detection_label in {"blue", "lata_azul", "azul"}:
            expected_delivery = "delivery_blue"
        else:
            expected_delivery = "delivery_blue"
            self.get_logger().warn(
                f"Classe detectada desconhecida ({self.latest_detection_label}); usando delivery_blue como fallback"
            )

        self.get_logger().info(
            f"Objeto preso. Aguardando base em {expected_delivery} antes de soltar"
        )
        self.wait_for_delivery_release(expected_delivery, timeout=delivery_timeout)

        self.move_to_pose("acima do place", qx, qy, qz + hover)
        self.move_to_pose("descendo p/ soltar", qx, qy, qz)
        self.pump(False)
        time.sleep(max(settle, 1.2))
        self.move_to_pose("subindo vazio", qx, qy, qz + hover)
        self.move_to_joints("home", [0.0] * 6)
        self.get_logger().info("Pick & place concluído!")

    def run_from_target(self, args):
        target = self.wait_for_pick_target(timeout=args.target_timeout)
        x, y, z = target.point.x, target.point.y, target.point.z
        self.get_logger().info(
            f"Recebi alvo visual em ({x:.3f}, {y:.3f}, {z:.3f}) no frame {target.header.frame_id}"
        )

        pick = (x, y, z)
        place = args.place
        if self.latest_detection_label == "red" and args.place_red is not None:
            place = args.place_red
        elif self.latest_detection_label == "blue" and args.place_blue is not None:
            place = args.place_blue
        self.run(pick, place, args.hover, args.settle, delivery_timeout=args.delivery_timeout)


def main():
    ap = argparse.ArgumentParser(description="Pick & place com suction pump")
    ap.add_argument("--pick", nargs=3, type=float, default=[0.15, 0.12, 0.0], metavar=("X", "Y", "Z"), help="pose do objeto (base frame)")
    ap.add_argument("--place", nargs=3, type=float, default=[0.15, -0.12, 0.0], metavar=("X", "Y", "Z"), help="pose de destino padrão")
    ap.add_argument("--place-red", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"), help="pose de destino para objetos vermelhos")
    ap.add_argument("--place-blue", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"), help="pose de destino para objetos azuis")
    ap.add_argument("--hover", type=float, default=0.06, help="altura de aproximação acima das poses (m)")
    ap.add_argument("--settle", type=float, default=1.0, help="pausa após ligar/desligar a pump (s)")
    ap.add_argument("--target-timeout", type=float, default=30.0, help="timeout para esperar /pick_detection_pose")
    ap.add_argument("--delivery-timeout", type=float, default=60.0, help="timeout para esperar /delivery_state")
    ap.add_argument("--wait-for-target", action="store_true", help="espera por /pick_detection_pose em vez de usar --pick")
    args = ap.parse_args()

    rclpy.init()
    node = PickAndPlace()
    try:
        node.wait_ready()
        if args.wait_for_target:
            node.run_from_target(args)
        else:
            node.run(tuple(args.pick), tuple(args.place), args.hover, args.settle, delivery_timeout=args.delivery_timeout)
    except Exception as e:
        node.get_logger().error(str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
