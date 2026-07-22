#!/usr/bin/env python3
# ============================================================
# pick_and_place.py — rotina pick & place com a Suction Pump
#
# Três modos (podem ser combinados):
#
# 1) POSES FIXAS (recomendado p/ lata em lugar fixo):
#    Grave as poses com RUN_POSE_TUNER.sh + SAVE_POSE.sh e rode:
#      --pick-pose pick --place-pose place
#      [--pick-approach-pose pick_approach]   (pose de aproximação)
#      [--place-approach-pose place_approach]
#      [--delivery red|blue]  (espera a base chegar antes de soltar)
#
# 2) CARTESIANO (xyz no frame da base, copo p/ baixo, IK automático):
#      --pick 0.15 0.12 0.0 --place 0.15 -0.12 0.0 --hover 0.06
#
# 3) VISÃO (YOLO): espera detecção em /pick_detection_pose e classe
#    em /product_class; separa por cor e espera /delivery_state:
#      --wait-for-target --place-pose-red vermelho --place-pose-blue azul
#
# Precisa do stack rodando (RUN_MOCK_PC.sh ou RUN_PLANNING_PC.sh).
# Sequência: home → aproximação → pega → PUMP ON → recua →
# [espera base] → aproximação → solta → PUMP OFF → recua → home
# ============================================================
import argparse
import math
import sys
import time

import rclpy
import yaml
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
POSES_FILE_DEFAULT = "/root/custom_ws/config/arm_poses.yaml"


def cup_down_quat(yaw):
    """Quaternion (x,y,z,w) de rpy(pi, 0, yaw).

    O eixo z do pump_tcp sai do copo de sucção; o roll de pi vira o copo
    para BAIXO (de frente para o objeto na mesa). O yaw em torno da
    vertical é livre — variamos para dar opções ao IK.
    ATENÇÃO: sem o roll de pi (rotação pura de yaw) o copo aponta para
    CIMA e o IK falha ou acha poses absurdas."""
    return (math.cos(yaw / 2.0), math.sin(yaw / 2.0), 0.0, 0.0)


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
        self.valid_detection_label = None
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
        label = (msg.data or "unknown").strip()
        label_low = label.lower()
        self.latest_detection_label = label_low
        if label_low.startswith("tin_valid_"):
            if self.valid_detection_label != label_low:
                self.get_logger().info(f"Valid tin detected: {label_low}")
            self.valid_detection_label = label_low
        elif label_low == "tin_invalid":
            self.valid_detection_label = None
            self.get_logger().warn(
                "Invalid tin detected (flipped/wrong size/side). Ignoring pick request.",
                throttle_duration_sec=2.0)
        else:
            self.valid_detection_label = None
            self.get_logger().warn(f"Ignoring unsupported label: {label}",
                                   throttle_duration_sec=2.0)

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
        # descarta estado antigo (latched de execuções anteriores) e
        # espera uma mensagem NOVA com o estado esperado
        self.delivery_state = None
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

    # ── IK ───────────────────────────────────────────────────────────
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
        varre yaws e sementes, junta as soluções e escolhe a mais
        natural (evita poses contorcidas)."""
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

    # ── planejamento e execução ──────────────────────────────────────
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

    def goto(self, label, target):
        """target: ("joints", [6 floats]) ou ("xyz", (x, y, z))."""
        kind, value = target
        if kind == "joints":
            self.get_logger().info(f"→ {label} (pose gravada)")
            self.execute(self.plan(value))
        else:
            x, y, z = value
            self.get_logger().info(f"→ {label}: ({x:.3f}, {y:.3f}, {z:.3f})")
            self.execute(self.plan(self.ik(x, y, z)))

    def pump(self, on):
        cli = self.pump_on_cli if on else self.pump_off_cli
        res = self._call(cli, Trigger.Request())
        state = "ON (sugando)" if on else "OFF (soltou)"
        if not res.success:
            raise RuntimeError(f"Pump falhou: {res.message}")
        self.get_logger().info(f"  Pump {state}")

    # ── rotina completa ──────────────────────────────────────────────
    def run(self, pick_t, place_t, settle, pick_app=None, place_app=None,
            delivery_expected=None, delivery_timeout=60.0):
        """pick_t/place_t: alvo do contato; pick_app/place_app: alvo de
        aproximação (opcional — recomendado p/ não arrastar a lata)."""
        self.goto("home", ("joints", [0.0] * 6))

        if pick_app is not None:
            self.goto("aproximação do pick", pick_app)
        self.goto("descendo no objeto", pick_t)
        self.pump(True)
        time.sleep(settle)  # tempo p/ vácuo pegar
        if pick_app is not None:
            self.goto("subindo com objeto", pick_app)

        if delivery_expected:
            self.get_logger().info(
                f"Objeto preso. Aguardando base em {delivery_expected} antes de soltar")
            self.wait_for_delivery_release(delivery_expected, timeout=delivery_timeout)

        if place_app is not None:
            self.goto("aproximação do place", place_app)
        self.goto("descendo p/ soltar", place_t)
        self.pump(False)
        time.sleep(max(settle, 1.2))  # pulso de deflação ~1s
        if place_app is not None:
            self.goto("subindo vazio", place_app)
        self.goto("home", ("joints", [0.0] * 6))
        self.get_logger().info("Pick & place concluído!")

    def run_from_target(self, args, poses):
        """Modo visão: espera detecção YOLO; só pega lata válida."""
        target = self.wait_for_pick_target(timeout=args.target_timeout)
        x, y, z = target.point.x, target.point.y, target.point.z
        self.get_logger().info(
            f"Recebi alvo visual em ({x:.3f}, {y:.3f}, {z:.3f}) no frame {target.header.frame_id}"
        )
        if not self.valid_detection_label or not self.valid_detection_label.startswith("tin_valid_"):
            self.get_logger().warn("Invalid or unsupported tin label; skipping pick-and-place sequence.")
            return

        color = "red" if self.valid_detection_label.startswith("tin_valid_red") else "blue"
        place_t, place_app = resolve_place_for_color(args, poses, color)
        pick_t = ("xyz", (x, y, z))
        pick_app = ("xyz", (x, y, z + args.hover))
        self.run(pick_t, place_t, args.settle,
                 pick_app=pick_app, place_app=place_app,
                 delivery_expected=f"delivery_{color}",
                 delivery_timeout=args.delivery_timeout)


# ── resolução de alvos (CLI/poses.yaml) ──────────────────────────────
def load_poses(path):
    try:
        with open(path) as f:
            poses = yaml.safe_load(f) or {}
    except FileNotFoundError:
        poses = {}
    return poses


def pose_target(poses, name):
    if name not in poses:
        raise RuntimeError(
            f"Pose '{name}' não existe em arm_poses.yaml — grave com ./SAVE_POSE.sh {name}")
    joints = [float(v) for v in poses[name]]
    if len(joints) != 6:
        raise RuntimeError(f"Pose '{name}' inválida (precisa de 6 juntas)")
    return ("joints", joints)


def resolve_place_for_color(args, poses, color):
    """Prioridade: --place-pose-<cor> > --place-<cor> xyz > --place-pose > --place."""
    name = getattr(args, f"place_pose_{color}")
    xyz = getattr(args, f"place_{color}")
    if name:
        return pose_target(poses, name), maybe_pose(poses, args.place_approach_pose)
    if xyz is not None:
        return ("xyz", tuple(xyz)), ("xyz", (xyz[0], xyz[1], xyz[2] + args.hover))
    if args.place_pose:
        return pose_target(poses, args.place_pose), maybe_pose(poses, args.place_approach_pose)
    p = args.place
    return ("xyz", tuple(p)), ("xyz", (p[0], p[1], p[2] + args.hover))


def maybe_pose(poses, name):
    return pose_target(poses, name) if name else None


def main():
    ap = argparse.ArgumentParser(description="Pick & place com suction pump")
    # modo cartesiano
    ap.add_argument("--pick", nargs=3, type=float, default=[0.15, 0.12, 0.0], metavar=("X", "Y", "Z"), help="pose do objeto (base frame)")
    ap.add_argument("--place", nargs=3, type=float, default=[0.15, -0.12, 0.0], metavar=("X", "Y", "Z"), help="pose de destino padrão")
    ap.add_argument("--place-red", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"), help="destino xyz para latas vermelhas")
    ap.add_argument("--place-blue", nargs=3, type=float, default=None, metavar=("X", "Y", "Z"), help="destino xyz para latas azuis")
    ap.add_argument("--hover", type=float, default=0.06, help="altura de aproximação (m, modo xyz)")
    # modo poses gravadas (sliders)
    ap.add_argument("--poses-file", default=POSES_FILE_DEFAULT, help="yaml com poses gravadas")
    ap.add_argument("--pick-pose", default=None, help="nome da pose de pegar (arm_poses.yaml)")
    ap.add_argument("--place-pose", default=None, help="nome da pose de soltar")
    ap.add_argument("--pick-approach-pose", default=None, help="pose de aproximação do pick")
    ap.add_argument("--place-approach-pose", default=None, help="pose de aproximação do place")
    ap.add_argument("--place-pose-red", default=None, help="pose de soltar p/ latas vermelhas")
    ap.add_argument("--place-pose-blue", default=None, help="pose de soltar p/ latas azuis")
    # entrega/base
    ap.add_argument("--delivery", choices=["red", "blue"], default=None, help="espera a base chegar em delivery_<cor> antes de soltar")
    ap.add_argument("--delivery-timeout", type=float, default=60.0, help="timeout p/ esperar /delivery_state")
    # modo visão
    ap.add_argument("--wait-for-target", action="store_true", help="espera detecção YOLO em /pick_detection_pose")
    ap.add_argument("--target-timeout", type=float, default=30.0, help="timeout p/ esperar /pick_detection_pose")
    ap.add_argument("--settle", type=float, default=1.0, help="pausa após ligar/desligar a pump (s)")
    args = ap.parse_args()

    poses = load_poses(args.poses_file)

    rclpy.init()
    node = PickAndPlace()
    try:
        node.wait_ready()
        if args.wait_for_target:
            node.run_from_target(args, poses)
        else:
            if args.pick_pose:
                pick_t = pose_target(poses, args.pick_pose)
                pick_app = maybe_pose(poses, args.pick_approach_pose)
            else:
                p = args.pick
                pick_t = ("xyz", tuple(p))
                pick_app = ("xyz", (p[0], p[1], p[2] + args.hover))
            if args.place_pose:
                place_t = pose_target(poses, args.place_pose)
                place_app = maybe_pose(poses, args.place_approach_pose)
            else:
                q = args.place
                place_t = ("xyz", tuple(q))
                place_app = ("xyz", (q[0], q[1], q[2] + args.hover))
            delivery = f"delivery_{args.delivery}" if args.delivery else None
            node.run(pick_t, place_t, args.settle,
                     pick_app=pick_app, place_app=place_app,
                     delivery_expected=delivery,
                     delivery_timeout=args.delivery_timeout)
    except Exception as e:
        node.get_logger().error(str(e))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
