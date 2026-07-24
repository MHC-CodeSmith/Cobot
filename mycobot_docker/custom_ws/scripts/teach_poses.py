#!/usr/bin/env python3
# ============================================================
# teach_poses.py — utilitário interativo de gravação de poses
#
# Salva as 5 poses de mesa de teste em test_table_poses.yaml
# e permite a reprodução assistida em baixa velocidade.
# ============================================================
import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from control_msgs.action import FollowJointTrajectory

JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GROUP = "mycobot_arm"
POSES_FILE = "/root/custom_ws/config/test_table_poses.yaml"
REQUIRED_POSES = ["scan", "pick_approach", "pick", "place_approach", "place"]

class PoseTeacher(Node):
    def __init__(self):
        super().__init__("pose_teacher")
        self.release_cli = self.create_client(Trigger, "/release_servos")
        self.lock_cli = self.create_client(Trigger, "/lock_servos")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.traj_cli = ActionClient(self, FollowJointTrajectory, "mycobot_arm_controller/follow_joint_trajectory")
        
        self.current_joints = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        
        self.poses = {}
        if os.path.exists(POSES_FILE):
            try:
                with open(POSES_FILE) as f:
                    self.poses = yaml.safe_load(f) or {}
                self.get_logger().info(f"Carregadas {len(self.poses)} poses de {POSES_FILE}")
            except Exception as e:
                self.get_logger().error(f"Erro ao carregar poses: {e}")

    def _js_cb(self, msg):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            idx = {n: i for i, n in enumerate(msg.name)}
            self.current_joints = [msg.position[idx[n]] for n in JOINT_NAMES]

    def wait_for_services(self, timeout=5.0):
        print("Conectando aos nós do ROS...")
        self.release_cli.wait_for_service(timeout_sec=timeout)
        self.lock_cli.wait_for_service(timeout_sec=timeout)
        self.plan_cli.wait_for_service(timeout_sec=timeout)
        self.traj_cli.wait_for_server(timeout_sec=timeout)

        # Aguarda receber /joint_states
        t_end = time.time() + timeout
        while self.current_joints is None and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def call_trigger(self, cli, label):
        if not cli.service_is_ready():
            print(f"Erro: Serviço de {label} indisponível.")
            return False
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            print(f"✓ {label} efetuado com sucesso!")
            return True
        else:
            msg = res.message if res else "Sem resposta"
            print(f"✗ Falha ao {label.lower()}: {msg}")
            return False

    def release_servos(self):
        return self.call_trigger(self.release_cli, "Liberar Servos")

    def lock_servos(self):
        return self.call_trigger(self.lock_cli, "Travar Servos")

    def record_pose(self, name):
        # Garante leitura fresca do tópico
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            
        if self.current_joints is None:
            print("Erro: Não foi possível obter posição atual de /joint_states.")
            return False
            
        self.poses[name] = [float(v) for v in self.current_joints]
        deg = [round(v * 57.2958, 1) for v in self.current_joints]
        print(f"✓ Pose '{name}' gravada: {[round(v,3) for v in self.current_joints]} rad ({deg} graus)")
        return True

    def plan_and_execute(self, target_joints, label):
        # Garante leitura fresca do tópico antes de planejar
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            
        if self.current_joints is None:
            print("Erro: Sem leitura das juntas atuais.")
            return False

        if not self.plan_cli.service_is_ready() or not self.traj_cli.server_is_ready():
            print("Erro: MoveIt desativado ou indisponível.")
            return False

        # Planejador MoveIt
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 5
        mpr.max_velocity_scaling_factor = 0.10  # Velocidade 10% para segurança
        mpr.max_acceleration_scaling_factor = 0.10
        mpr.start_state.joint_state.name = list(JOINT_NAMES)
        mpr.start_state.joint_state.position = [float(v) for v in self.current_joints]
        
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
        
        fut = self.plan_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        
        if not res or res.motion_plan_response.error_code.val != 1:
            print(f"✗ Erro de planejamento para a pose: {label}")
            return False

        print(f"Movendo suavemente para {label}...")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = res.motion_plan_response.trajectory.joint_trajectory
        
        send_goal_fut = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_fut)
        gh = send_goal_fut.result()
        if gh is None or not gh.accepted:
            print("✗ Trajetória rejeitada pelo bridge do robô.")
            return False
            
        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        
        # Atualiza a posição em memória
        self.current_joints = list(target_joints)
        print("✓ Pose alcançada.")
        return True

    def save_poses(self):
        os.makedirs(os.path.dirname(POSES_FILE), exist_ok=True)
        try:
            with open(POSES_FILE, "w") as f:
                yaml.safe_dump(self.poses, f, default_flow_style=None, sort_keys=True)
            print(f"✓ Poses salvas com sucesso em: {POSES_FILE}")
            return True
        except Exception as e:
            print(f"✗ Erro ao salvar arquivo: {e}")
            return False


def print_menu(poses):
    print("\n" + "="*45)
    print("      MENU MODO ENSINO MYCOBOT - MESA       ")
    print("="*45)
    print("1. [RELEASE] Liberar torque (ATENÇÃO: Segure o braço!)")
    print("2. [LOCK]    Travar torque na posição atual")
    print("3. [RECORD]  Gravar pose atual do robô")
    print("4. [PLAYBACK] Testar trajetória (velocidade reduzida)")
    print("5. [SAVE]    Salvar no arquivo test_table_poses.yaml")
    print("6. [EXIT]    Sair")
    print("-"*45)
    print("Status das poses:")
    for p in REQUIRED_POSES:
        status = "GRAVADA" if p in poses else "PENDENTE"
        print(f"  - {p:15s}: {status}")
    print("="*45)


def main():
    rclpy.init()
    teacher = PoseTeacher()
    teacher.wait_for_services()

    try:
        while True:
            print_menu(teacher.poses)
            opcao = input("Escolha uma opção (1-6): ").strip()
            
            if opcao == "1":
                print("\nATENÇÃO: Segure o braço com a mão! Motores desligando em 3s...")
                time.sleep(3)
                teacher.release_servos()
            
            elif opcao == "2":
                teacher.lock_servos()
                
            elif opcao == "3":
                print("\nQual pose deseja gravar?")
                for i, p in enumerate(REQUIRED_POSES, 1):
                    print(f"  {i}. {p}")
                sel = input("Selecione (1-5): ").strip()
                try:
                    idx = int(sel) - 1
                    if 0 <= idx < len(REQUIRED_POSES):
                        pose_name = REQUIRED_POSES[idx]
                        teacher.record_pose(pose_name)
                    else:
                        print("Seleção inválida.")
                except ValueError:
                    print("Seleção inválida.")
                    
            elif opcao == "4":
                # Validação de trajetória completa
                missing = [p for p in REQUIRED_POSES if p not in teacher.poses]
                if missing:
                    print(f"\nErro: Gravação incompleta. Poses pendentes: {missing}")
                    continue
                
                confirm = input("\nIniciar teste de trajetória a 10% de velocidade? (s/n): ").strip().lower()
                if confirm != "s":
                    print("Cancelado.")
                    continue
                
                print("\n[PLAYBACK] Travando motores para iniciar...")
                teacher.lock_servos()
                
                # Sequência de teste recomendada
                sequence = [
                    ("scan", teacher.poses["scan"]),
                    ("pick_approach", teacher.poses["pick_approach"]),
                    ("pick", teacher.poses["pick"]),
                    ("pick_approach", teacher.poses["pick_approach"]),
                    ("place_approach", teacher.poses["place_approach"]),
                    ("place", teacher.poses["place"]),
                    ("place_approach", teacher.poses["place_approach"]),
                    ("scan", teacher.poses["scan"])
                ]
                
                success = True
                for label, joints in sequence:
                    if not teacher.plan_and_execute(joints, label):
                        print(f"\n✗ Trajetória interrompida na pose: {label}")
                        success = False
                        break
                    time.sleep(1.0)
                    
                if success:
                    print("\n✓ Trajetória completa testada com sucesso!")
                    
            elif opcao == "5":
                teacher.save_poses()
                
            elif opcao == "6":
                print("Saindo...")
                break
            else:
                print("Opção inválida.")
                
    except KeyboardInterrupt:
        print("\nInterrompido pelo usuário.")
    finally:
        teacher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
