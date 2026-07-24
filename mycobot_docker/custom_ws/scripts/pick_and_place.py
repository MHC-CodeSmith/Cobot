#!/usr/bin/env python3
# ============================================================
# pick_and_place.py — máquina de estados de controle do cobot
#
# Comportamento:
#   1. Início: vai para pose 'scan', aguarda YOLO em /product_class
#   2. Gatilho: 3 frames consecutivos de 'tin_valid_red_triangle'
#      ou 'tin_valid_blue_square' com conf > 0.70
#   3. Executa pick & place: pump_on -> pick -> pump_off -> scan
#   4. Recarrega as poses dinamicamente no início de cada ciclo
# ============================================================
import argparse
import os
import sys
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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

class PickAndPlaceStateMachine(Node):
    def __init__(self, mock=False):
        super().__init__("pick_and_place")
        self.mock = mock
        self.pump_on_cli = self.create_client(Trigger, "pump_on")
        self.pump_off_cli = self.create_client(Trigger, "pump_off")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.traj_cli = ActionClient(self, FollowJointTrajectory, "mycobot_arm_controller/follow_joint_trajectory")
        
        self.current_joints = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(String, "/product_class", self._on_detection_label, 10)
        
        # Poses carregadas dinamicamente
        self.poses = {}
        self.reload_poses()
        
        # Variáveis da máquina de estados
        self.consecutive_frames = 0
        self.last_class = None
        self.triggered_class = None

    def reload_poses(self):
        """Carrega poses do arquivo de configuração diretamente do disco sem cache."""
        self.get_logger().info(f"Recarregando arquivo de poses: {POSES_FILE}")
        if not os.path.exists(POSES_FILE):
            self.get_logger().error(f"Arquivo de poses {POSES_FILE} não existe! Grave-as primeiro com o ./RUN_TEACH.sh")
            sys.exit(1)
        try:
            with open(POSES_FILE) as f:
                self.poses = yaml.safe_load(f) or {}
            
            missing = [p for p in REQUIRED_POSES if p not in self.poses]
            if missing:
                self.get_logger().error(f"Poses pendentes em {POSES_FILE}: {missing}. Grave todas as 5 poses.")
                sys.exit(1)
                
            self.get_logger().info("✓ Poses recarregadas com sucesso!")
        except Exception as e:
            self.get_logger().error(f"Falha ao ler o arquivo de configuração de poses: {e}")
            sys.exit(1)

    def _js_cb(self, msg):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            idx = {n: i for i, n in enumerate(msg.name)}
            self.current_joints = [msg.position[idx[n]] for n in JOINT_NAMES]

    def _on_detection_label(self, msg):
        # Ignora se já estiver executando um ciclo de pick&place
        if self.triggered_class is not None:
            return
            
        data = (msg.data or "").strip()
        self.get_logger().info(f"[YOLO Input] Recebido em /product_class: '{data}'")
        if not data:
            return
            
        # Suporta formatos "class_name conf" ou "class_name:conf"
        parts = data.replace(":", " ").split()
        cls_name = parts[0].lower()
        conf = 1.0
        if len(parts) > 1:
            try:
                conf = float(parts[1])
            except ValueError:
                pass
                
        # Filtro de detecção: tin_valid_red_triangle ou tin_valid_blue_square, conf > 0.70
        if cls_name in ["tin_valid_red_triangle", "tin_valid_blue_square"] and conf > 0.70:
            if self.last_class == cls_name:
                self.consecutive_frames += 1
            else:
                self.consecutive_frames = 1
                self.last_class = cls_name
                
            if self.consecutive_frames >= 3:
                self.get_logger().info(f"✓ GATILHO ATIVO: '{cls_name}' detectado por {self.consecutive_frames} frames consecutivos (conf={conf:.2f})!")
                self.triggered_class = cls_name
        else:
            if self.consecutive_frames > 0:
                self.get_logger().warn(f"Filtro resetado: classe '{cls_name}' inválida ou confiança baixa ({conf:.2f})")
            self.consecutive_frames = 0
            self.last_class = None

    def wait_ready(self, timeout=10.0):
        self.get_logger().info("Aguardando conexões com os nós do robô...")
        if self.mock:
            self.get_logger().info("[MOCK] Ignorando conexões físicas e inicializando juntas simuladas...")
            if self.current_joints is None:
                self.current_joints = [0.0] * 6
            return
            
        for cli, name in [
            (self.pump_on_cli, "pump_on"),
            (self.pump_off_cli, "pump_off"),
            (self.plan_cli, "/plan_kinematic_path")
        ]:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Serviço {name} indisponível — o stack está rodando?")
                
        if not self.traj_cli.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Action server de trajetória do robô indisponível.")
            
        t_end = time.time() + timeout
        while self.current_joints is None and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_joints is None:
            raise RuntimeError("Sem dados de /joint_states — a bridge está ativa?")

    def set_pump(self, on):
        state_label = "LIGAR (Sucção)" if on else "DESLIGAR (Válvula)"
        self.get_logger().info(f"Bomba -> {state_label}...")
        
        if self.mock:
            self.get_logger().info(f"[MOCK] Bomba {state_label} simulada com sucesso.")
            return

        cli = self.pump_on_cli if on else self.pump_off_cli
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            self.get_logger().info(f"✓ Bomba: {res.message}")
        else:
            msg = res.message if res else "Sem resposta"
            self.get_logger().error(f"✗ Falha no controle da bomba: {msg}")
            raise RuntimeError(f"Erro na bomba: {msg}")

    def goto(self, label, target_joints):
        self.get_logger().info(f"Movendo para: {label}...")
        
        if self.mock:
            self.get_logger().info(f"[MOCK] Simulando movimento para a pose: {label} (1.5s)...")
            time.sleep(1.5)
            self.current_joints = list(target_joints)
            self.get_logger().info(f"[MOCK] ✓ Chegou na pose {label}")
            return True

        # Garante leitura fresca de juntas
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            
        # Planeja trajetória com MoveIt
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 5
        mpr.max_velocity_scaling_factor = 0.20  # Velocidade 20%
        mpr.max_acceleration_scaling_factor = 0.20
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
            raise RuntimeError(f"Planejamento falhou para a pose: {label}")
            
        # Executa no robô
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = res.motion_plan_response.trajectory.joint_trajectory
        
        send_goal_fut = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_fut)
        gh = send_goal_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("Trajetória rejeitada pelo bridge.")
            
        result_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_fut)
        
        self.current_joints = list(target_joints)
        self.get_logger().info(f"✓ Chegou na pose {label}")

    def wait_for_target(self):
        self.get_logger().info("Aguardando detecção de objeto válido no scan...")
        self.triggered_class = None
        self.consecutive_frames = 0
        self.last_class = None
        
        # Aguarda até o callback de detecção acionar o gatilho
        while rclpy.ok() and self.triggered_class is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        return self.triggered_class

def main():
    parser = argparse.ArgumentParser(description="Máquina de estados de Pick & Place")
    parser.add_argument("--mock", action="store_true", help="Ativa modo simulação (sem hardware real/MoveIt)")
    args = parser.parse_args()

    rclpy.init()
    sm = PickAndPlaceStateMachine(mock=args.mock)
    
    try:
        sm.wait_ready()
        
        print("\n" + "="*50)
        print(f"      INICIANDO MÁQUINA DE ESTADOS PICK & PLACE      ")
        print(f"      Modo: {'SIMULAÇÃO (MOCK)' if sm.mock else 'ROBÔ REAL'} ")
        print("="*50)

        while rclpy.ok():
            # 1. Início de ciclo: Garante leitura atualizada das poses e vai para SCAN
            sm.reload_poses()
            sm.goto("scan", sm.poses["scan"])
            
            # 2. Aguarda sinal do YOLO (3 frames seguidos de detecção válida)
            detected_class = sm.wait_for_target()
            sm.get_logger().info(f"Iniciando ciclo de pick & place para o objeto: {detected_class}")
            
            # 3. Lógica de movimento e bomba
            # 3.1. Ligar bomba
            sm.set_pump(True)
            
            # 3.2. Mover: scan -> pick_approach -> pick
            sm.goto("pick_approach", sm.poses["pick_approach"])
            sm.goto("pick", sm.poses["pick"])
            
            # 3.3. Selagem do vácuo
            sm.get_logger().info("Selando vácuo / coletando objeto (1s)...")
            time.sleep(1.0)
            
            # 3.4. Subir com o objeto
            sm.goto("pick_approach", sm.poses["pick_approach"])
            
            # 3.5. Mover para place_approach -> place
            sm.goto("place_approach", sm.poses["place_approach"])
            sm.goto("place", sm.poses["place"])
            
            # 3.6. Desligar bomba
            sm.set_pump(False)
            
            # 3.7. Soltura do objeto
            sm.get_logger().info("Liberando objeto na mesa (1s)...")
            time.sleep(1.0)
            
            # 3.8. Retornar
            sm.goto("place_approach", sm.poses["place_approach"])
            
            # Reseta estado e volta para o início do loop
            sm.triggered_class = None
            sm.get_logger().info("✓ Ciclo concluído. Retornando ao SCAN...")
            print("\n" + "-"*50 + "\n")

    except KeyboardInterrupt:
        sm.get_logger().info("Interrompido pelo usuário. Desligando...")
    except Exception as e:
        sm.get_logger().error(f"Erro na máquina de estados: {e}")
        # Desliga a bomba por segurança em caso de erros
        try:
            sm.set_pump(False)
        except Exception:
            pass
        sys.exit(1)
    finally:
        sm.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
