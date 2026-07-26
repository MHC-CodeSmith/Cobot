#!/usr/bin/env python3
# ============================================================
# teach_poses.py — utilitário interativo de gravação de poses
#
# Salva as poses de mesa de teste em test_table_poses.yaml
# e permite a reprodução assistida via MoveGroup (/move_action)
# incluindo teste real da bomba de sucção (/pump_on e /pump_off).
# ============================================================
import os
import sys
import time
import threading
from datetime import datetime
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import PlanningOptions, Constraints, JointConstraint, MotionPlanRequest

JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GROUP = "mycobot_arm"
POSES_FILE = "/root/custom_ws/config/test_table_poses.yaml"
REQUIRED_POSES = ["home", "scan", "pick_approach", "pick", "place_approach", "place"]

class PoseTeacher(Node):
    def __init__(self):
        super().__init__("pose_teacher")
        self.release_cli = self.create_client(Trigger, "/release_servos")
        self.lock_cli = self.create_client(Trigger, "/lock_servos")
        self.pump_on_cli = self.create_client(Trigger, "/pump_on")
        self.pump_off_cli = self.create_client(Trigger, "/pump_off")
        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        
        self.current_joints = None
        # Subscreve em /joint_states e /joint_states_raw para garantia total de captura
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(JointState, "/joint_states_raw", self._js_cb, 10)
        
        self.poses = {}
        self.load_poses()

        # Executor multithreaded rodando em background para recepção contínua de tópicos
        self.node_executor = MultiThreadedExecutor()
        self.node_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.node_executor.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()

    def load_poses(self):
        if os.path.exists(POSES_FILE):
            try:
                with open(POSES_FILE) as f:
                    self.poses = yaml.safe_load(f) or {}
                valid_count = len([k for k in self.poses if not str(k).startswith("_")])
                self.get_logger().info(f"Carregadas {valid_count} poses de {POSES_FILE}")
            except Exception as e:
                self.get_logger().error(f"Erro ao carregar poses: {e}")
                self.poses = {}
        else:
            self.poses = {}

    def _js_cb(self, msg):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            idx = {n: i for i, n in enumerate(msg.name)}
            self.current_joints = [msg.position[idx[n]] for n in JOINT_NAMES]

    def wait_for_services(self, timeout=5.0):
        print("Conectando aos nós do ROS e ao MoveIt...")
        self.release_cli.wait_for_service(timeout_sec=2.0)
        self.lock_cli.wait_for_service(timeout_sec=2.0)
        self.move_cli.wait_for_server(timeout_sec=5.0)

        # Aguarda receber /joint_states em background
        t_end = time.time() + timeout
        while self.current_joints is None and time.time() < t_end:
            time.sleep(0.1)

    def call_trigger(self, cli, label):
        if not cli.service_is_ready():
            print(f"⚠️ [WARN] Serviço de {label} indisponível no robô.")
            return False
        req = Trigger.Request()
        fut = cli.call_async(req)
        t_start = time.time()
        while not fut.done() and (time.time() - t_start) < 3.0:
            time.sleep(0.05)
            
        if not fut.done():
            print(f"✗ Timeout de 3s ao chamar serviço de {label.lower()}.")
            return False
            
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

    def set_pump(self, on):
        cli = self.pump_on_cli if on else self.pump_off_cli
        state_label = "LIGAR (Sucção)" if on else "DESLIGAR (Válvula)"
        print(f"\n💨 Bomba -> {state_label}...")
        return self.call_trigger(cli, f"Bomba {state_label}")

    def record_pose(self, name):
        if self.current_joints is None:
            print("Erro: Não foi possível obter posição atual de /joint_states. Verifique se o bridge no Nano está ativo.")
            return False
            
        self.poses[name] = [float(v) for v in self.current_joints]
        deg = [round(v * 57.2958, 1) for v in self.current_joints]
        print(f"✓ Pose '{name}' gravada em memória: {[round(v,3) for v in self.current_joints]} rad ({deg} graus)")
        return True

    def plan_and_execute(self, target_joints, label):
        if self.current_joints is None:
            print("Erro: Sem leitura das juntas atuais.")
            return False

        if not self.move_cli.server_is_ready():
            print("Erro: Action Server /move_action do MoveIt desativado ou indisponível.")
            return False

        # Prepara requisição do MoveGroup Goal
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 5
        mpr.max_velocity_scaling_factor = 0.10  # Velocidade 10% para playback seguro
        mpr.max_acceleration_scaling_factor = 0.10
        
        mpr.start_state.is_diff = True
        if self.current_joints is not None:
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
        
        po = PlanningOptions()
        po.plan_only = False
        
        goal = MoveGroup.Goal()
        goal.request = mpr
        goal.planning_options = po
        
        print(f"Planejando e movendo suavemente para {label}...")
        send_goal_fut = self.move_cli.send_goal_async(goal)
        t_start = time.time()
        while not send_goal_fut.done() and (time.time() - t_start) < 5.0:
            time.sleep(0.05)
            
        if not send_goal_fut.done():
            print(f"✗ Timeout ao enviar planejamento para a pose: {label}")
            return False
            
        gh = send_goal_fut.result()
        if gh is None or not gh.accepted:
            print(f"✗ Trajetória rejeitada pelo MoveIt para a pose: {label}")
            return False
            
        result_fut = gh.get_result_async()
        t_start = time.time()
        while not result_fut.done() and (time.time() - t_start) < 15.0:
            time.sleep(0.05)
            
        if not result_fut.done():
            print(f"✗ Timeout aguardando execução da pose: {label}")
            return False
            
        res = result_fut.result()
        if not res or res.result.error_code.val != 1:
            print(f"✗ Falha na execução da pose {label}. Código de erro: {res.result.error_code.val if res else 'N/A'}")
            return False
        
        print(f"✓ Pose '{label}' alcançada com sucesso!")
        return True

    def save_poses(self):
        os.makedirs(os.path.dirname(POSES_FILE), exist_ok=True)
        self.poses["_last_saved"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        try:
            with open(POSES_FILE, "w") as f:
                yaml.safe_dump(self.poses, f, default_flow_style=None, sort_keys=True)
            print(f"✓ Poses salvas com sucesso em: {POSES_FILE} (Data/Hora: {self.poses['_last_saved']})")
            return True
        except Exception as e:
            print(f"✗ Erro ao salvar arquivo: {e}")
            return False

    def clear_poses(self):
        self.poses = {}
        if os.path.exists(POSES_FILE):
            try:
                os.remove(POSES_FILE)
                print(f"✓ Arquivo {POSES_FILE} apagado com sucesso. Calibragem zerada!")
            except Exception as e:
                print(f"✗ Erro ao remover arquivo de poses: {e}")
        else:
            print("✓ Poses zeradas em memória (arquivo ainda não existia).")


def print_menu(poses):
    print("\n" + "="*50)
    print("      MENU MODO ENSINO MYCOBOT - MESA       ")
    print("="*50)
    print("1. [RELEASE] Liberar torque (ATENÇÃO: Segure o braço!)")
    print("2. [LOCK]    Travar torque na posição atual")
    print("3. [RECORD]  Gravar pose atual do robô")
    print("4. [PLAYBACK] Testar trajetória completa + bomba (vel 10%)")
    print("5. [SAVE]    Salvar no arquivo test_table_poses.yaml")
    print("6. [CLEAR]   Apagar todas as poses salvas (Zerar calibragem)")
    print("7. [EXIT]    Sair")
    print("-"*50)
    saved_at = poses.get("_last_saved", "Nenhum salvamento registrado ainda")
    print(f"📅 Última gravação salva: {saved_at}")
    print("Status das poses:")
    for p in REQUIRED_POSES:
        status = "✓ GRAVADA" if p in poses else "❌ PENDENTE"
        print(f"  - {p:15s}: {status}")
    print("="*50)


def main():
    rclpy.init()
    teacher = PoseTeacher()
    teacher.wait_for_services()

    try:
        while True:
            print_menu(teacher.poses)
            opcao = input("Escolha uma opção (1-7): ").strip()
            
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
                sel = input(f"Selecione (1-{len(REQUIRED_POSES)}): ").strip()
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
                missing = [p for p in REQUIRED_POSES if p not in teacher.poses]
                if missing:
                    print(f"\nErro: Gravação incompleta. Poses pendentes: {missing}")
                    continue
                
                confirm = input("\nIniciar teste de trajetória completa + BOMBA a 10% de velocidade? (s/n): ").strip().lower()
                if confirm != "s":
                    print("Cancelado.")
                    continue
                
                print("\n[PLAYBACK] Travando motores para iniciar...")
                teacher.lock_servos()
                
                # Sequência de teste completa incluindo acionamento e desligamento da bomba de sucção
                success = True
                
                # 1. home -> scan -> pick_approach -> pick
                for label, joints in [
                    ("home", teacher.poses["home"]),
                    ("scan", teacher.poses["scan"]),
                    ("pick_approach", teacher.poses["pick_approach"]),
                    ("pick", teacher.poses["pick"])
                ]:
                    if not teacher.plan_and_execute(joints, label):
                        success = False
                        break
                    time.sleep(0.5)

                if not success:
                    print("\n✗ Trajetória interrompida antes da coleta.")
                    continue

                # 2. Ativar bomba de sucção na pose de pick
                teacher.set_pump(True)
                print("Selando vácuo / simulando pega (1s)...")
                time.sleep(1.0)

                # 3. Subida vertical pós-coleta -> altitude HOME -> place_approach -> place
                for label, joints in [
                    ("pick_approach", teacher.poses["pick_approach"]),
                    ("home", teacher.poses["home"]),
                    ("place_approach", teacher.poses["place_approach"]),
                    ("place", teacher.poses["place"])
                ]:
                    if not teacher.plan_and_execute(joints, label):
                        success = False
                        break
                    time.sleep(0.5)

                if not success:
                    print("\n✗ Trajetória interrompida durante o transporte.")
                    teacher.set_pump(False)
                    continue

                # 4. Desativar bomba de sucção na pose de place
                teacher.set_pump(False)
                print("Soltando vácuo / simulando soltura (1s)...")
                time.sleep(1.0)

                # 5. Subida vertical pós-soltura -> altitude HOME -> scan
                for label, joints in [
                    ("place_approach", teacher.poses["place_approach"]),
                    ("home", teacher.poses["home"]),
                    ("scan", teacher.poses["scan"])
                ]:
                    if not teacher.plan_and_execute(joints, label):
                        success = False
                        break
                    time.sleep(0.5)

                if success:
                    print("\n✓ Trajetória completa com acionamento da bomba testada com sucesso!")
                    
            elif opcao == "5":
                teacher.save_poses()
                
            elif opcao == "6":
                confirm = input("\nTem certeza que deseja APAGAR todas as poses salvas e zerar a calibragem? (s/n): ").strip().lower()
                if confirm == "s":
                    teacher.clear_poses()
                else:
                    print("Operação cancelada.")
                
            elif opcao == "7":
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
