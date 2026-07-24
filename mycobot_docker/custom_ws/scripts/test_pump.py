#!/usr/bin/env python3
# ============================================================
# test_pump.py — rotina de teste físico da bomba de sucção
#
# Lê test_table_poses.yaml e executa o ciclo completo de
# pick & place lento para validar a atração de vácuo.
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

class PumpTester(Node):
    def __init__(self):
        super().__init__("pump_tester")
        self.pump_on_cli = self.create_client(Trigger, "pump_on")
        self.pump_off_cli = self.create_client(Trigger, "pump_off")
        self.plan_cli = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.traj_cli = ActionClient(self, FollowJointTrajectory, "mycobot_arm_controller/follow_joint_trajectory")
        
        self.current_joints = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        
        # Carrega poses
        self.poses = {}
        if not os.path.exists(POSES_FILE):
            self.get_logger().error(f"Arquivo de poses não encontrado: {POSES_FILE}. Grave as poses primeiro usando ./RUN_TEACH.sh")
            sys.exit(1)
            
        try:
            with open(POSES_FILE) as f:
                self.poses = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Erro ao ler poses: {e}")
            sys.exit(1)
            
        # Valida se todas as poses necessárias estão salvas
        missing = [p for p in REQUIRED_POSES if p not in self.poses]
        if missing:
            self.get_logger().error(f"Poses pendentes no arquivo: {missing}. Conclua a gravação de todas as 5 poses.")
            sys.exit(1)

    def _js_cb(self, msg):
        if set(JOINT_NAMES).issubset(set(msg.name)):
            idx = {n: i for i, n in enumerate(msg.name)}
            self.current_joints = [msg.position[idx[n]] for n in JOINT_NAMES]

    def wait_ready(self, timeout=10.0):
        self.get_logger().info("Conectando aos serviços do robô...")
        for cli, name in [
            (self.pump_on_cli, "pump_on"),
            (self.pump_off_cli, "pump_off"),
            (self.plan_cli, "/plan_kinematic_path")
        ]:
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Serviço {name} indisponível — o stack está rodando?")
                
        if not self.traj_cli.wait_for_server(timeout_sec=timeout):
            raise RuntimeError("Action server de trajetória do robô indisponível.")
            
        # Aguarda leitura fresca de juntas
        t_end = time.time() + timeout
        while self.current_joints is None and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.current_joints is None:
            raise RuntimeError("Sem dados de /joint_states — a bridge está ativa?")

    def set_pump(self, on):
        cli = self.pump_on_cli if on else self.pump_off_cli
        state_label = "LIGAR (Sucção)" if on else "DESLIGAR (Válvula)"
        self.get_logger().info(f"Acionando bomba: {state_label}...")
        
        req = Trigger.Request()
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res and res.success:
            self.get_logger().info(f"✓ Bomba respondida: {res.message}")
        else:
            msg = res.message if res else "Sem resposta"
            self.get_logger().error(f"✗ Falha no acionamento da bomba: {msg}")
            raise RuntimeError(f"Falha ao acionar a bomba: {msg}")

    def goto(self, label, target_joints):
        self.get_logger().info(f"Movendo para {label}...")
        # Garante leitura fresca antes de planejar
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)
            
        # Planejar
        req = GetMotionPlan.Request()
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 5
        mpr.max_velocity_scaling_factor = 0.15  # Velocidade segura reduzida
        mpr.max_acceleration_scaling_factor = 0.15
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
            raise RuntimeError(f"Falha de planejamento para a pose: {label}")
            
        # Executar
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

def main():
    rclpy.init()
    tester = PumpTester()
    
    try:
        tester.wait_ready()
        
        print("\n" + "="*50)
        print("    TESTE FÍSICO DE ATUAÇÃO DA BOMBA DE SUCÇÃO")
        print("="*50)
        print("Certifique-se de posicionar a lata na mesa sob o bocal.")
        input("Pressione ENTER para iniciar a sequência de teste...")
        
        # 1. Mover para scan -> pick_approach -> pick
        tester.goto("scan", tester.poses["scan"])
        time.sleep(1.0)
        
        tester.goto("pick_approach", tester.poses["pick_approach"])
        time.sleep(0.5)
        
        tester.goto("pick", tester.poses["pick"])
        time.sleep(0.5)
        
        # 2. Ligar a bomba de sucção
        tester.set_pump(True)
        print(">> Criando vácuo (1.0s)...")
        time.sleep(1.0)
        
        # 3. Subir de volta para pick_approach (Verificação visual de içamento)
        tester.goto("pick_approach", tester.poses["pick_approach"])
        print("\n>> VERIFICAÇÃO: A lata foi içada da mesa? (Deixe a lata grudada)")
        input("Pressione ENTER para continuar para a pose de descarte...")
        
        # 4. Mover para place_approach -> place
        tester.goto("place_approach", tester.poses["place_approach"])
        time.sleep(0.5)
        
        tester.goto("place", tester.poses["place"])
        time.sleep(0.5)
        
        # 5. Desligar a bomba / acionar válvula de liberação
        tester.set_pump(False)
        print(">> Liberando lata (1.5s)...")
        time.sleep(1.5)
        
        # 6. Subir de volta para place_approach -> retornar para scan
        tester.goto("place_approach", tester.poses["place_approach"])
        time.sleep(0.5)
        
        tester.goto("scan", tester.poses["scan"])
        
        print("\n" + "="*50)
        print("             TESTE CONCLUÍDO COM SUCESSO!")
        print("="*50)
        
    except Exception as e:
        tester.get_logger().error(f"Erro durante o teste: {e}")
        # Desliga a bomba em caso de exceção por segurança
        try:
            tester.set_pump(False)
        except Exception:
            pass
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
