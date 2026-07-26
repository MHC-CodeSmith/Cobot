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
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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

class PickAndPlaceStateMachine(Node):
    def __init__(self, mock=False):
        super().__init__("pick_and_place")
        self.mock = mock
        self.pump_on_cli = self.create_client(Trigger, "/pump_on")
        self.pump_off_cli = self.create_client(Trigger, "/pump_off")
        
        # Cliente de Ação único do MoveGroup (Planeja + Executa nativamente)
        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        
        self.pump_available = True
        self.current_joints = None
        self.last_valid_time = None
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.product_sub = self.create_subscription(String, "/product_class", self._on_detection_label, qos_profile)
        self.get_logger().info(f"📡 Subscription criada em /product_class (QoS: RELIABLE, KEEP_LAST, depth=10)")
        
        # Poses carregadas dinamicamente
        self.poses = {}
        self.reload_poses()
        
        # Variáveis da máquina de estados
        self.consecutive_frames = 0
        self.last_class = None
        self.triggered_class = None

        # Inicia thread de spin em background com MultiThreadedExecutor para processamento paralelo de callbacks
        self.node_executor = MultiThreadedExecutor()
        self.node_executor.add_node(self)
        self.spin_thread = threading.Thread(target=self.node_executor.spin)
        self.spin_thread.daemon = True
        self.spin_thread.start()

    def destroy_node(self):
        # Destrói explicitamente o cliente de ação para evitar traceback no __del__
        if hasattr(self, 'move_cli') and self.move_cli is not None:
            self.move_cli.destroy()
        super().destroy_node()

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
        self.get_logger().info(f"🔔 [CALLBACK TRIGGERED] Mensagem recebida crua: '{msg.data}' | triggered_class='{self.triggered_class}'")
        try:
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
                    
            # 1. Tratamento de lata inválida (tin_invalid)
            if cls_name.startswith("tin_invalid"):
                self.get_logger().warn("❌ [tin_invalid] Lata inválida ou virada detectada! O robô NÃO irá coletar. Aguardando objeto válido...", throttle_duration_sec=3.0)
                self.get_logger().info("[LED] Alterando cor do LED do MyCobot para VERMELHO (Lata Inválida)")
                return

            # 2. Filtro de detecção: tin_valid_* (vermelha ou azul), conf >= 0.60
            if (cls_name.startswith("tin_valid") or "valid" in cls_name) and conf >= 0.60:
                now = time.time()
                self.last_valid_time = now  # Atualiza o tempo da última detecção válida
                
                if self.last_class == cls_name:
                    self.consecutive_frames += 1
                else:
                    self.consecutive_frames = 1
                    self.last_class = cls_name
                    
                if self.consecutive_frames >= 3:
                    self.get_logger().info(f"✓ GATILHO ATIVO: '{cls_name}' detectado por {self.consecutive_frames} frames consecutivos (conf={conf:.2f})!")
                    self.get_logger().info(f"[LED] Alterando cor do LED do MyCobot para VERDE (Lata Válida: {cls_name})")
                    self.triggered_class = cls_name
            else:
                self.get_logger().info(f"ℹ️ [YOLO Ignorado] Classe='{cls_name}' | conf={conf:.2f} (Limiar mínimo=0.60)")
        except Exception as e:
            self.get_logger().error(f"❌ Erro no processamento do callback: {e}")

    def wait_ready(self, timeout=10.0):
        self.get_logger().info("Aguardando conexões com os nós do robô...")
        if self.mock:
            self.get_logger().info("[MOCK] Ignorando conexões físicas e inicializando juntas simuladas...")
            self.pump_available = False
            if self.current_joints is None:
                self.current_joints = [0.0] * 6
            return
            
        # 1. Action Server do MoveGroup (Crítico) - Aguarda conexão real
        while rclpy.ok():
            if self.move_cli.wait_for_server(timeout_sec=3.0):
                self.get_logger().info("✓ Action Server /move_action (MoveGroup) conectado com sucesso!")
                break
            self.get_logger().warn("⚠️ [WARN] Aguardando o Action Server do MoveIt (/move_action) iniciar...")
            
        # Aguarda a leitura fresca das juntas (alimentado pela thread de spin)
        t_end = time.time() + timeout
        while self.current_joints is None and time.time() < t_end:
            time.sleep(0.1)
        if self.current_joints is None:
            raise RuntimeError("Sem dados de /joint_states — a bridge está ativa?")
            
        # 2. Serviços da bomba de sucção (Opcional com timeout de 2.0s)
        self.get_logger().info("Verificando serviços da bomba de sucção (/pump_on / /pump_off)...")
        pump_on_ready = self.pump_on_cli.wait_for_service(timeout_sec=2.0)
        pump_off_ready = self.pump_off_cli.wait_for_service(timeout_sec=2.0)
        
        if pump_on_ready and pump_off_ready:
            self.pump_available = True
            self.get_logger().info("✓ Serviços da bomba de sucção conectados.")
        else:
            self.pump_available = False
            self.get_logger().warn("⚠️ [WARN] Serviços de bomba (/pump_on e /pump_off) indisponíveis. A bomba física será ignorada e o robô continuará a trajetória física normalmente.")

    def set_pump(self, on):
        state_label = "LIGAR (Sucção)" if on else "DESLIGAR (Válvula)"
        self.get_logger().info(f"Bomba -> {state_label}...")
        
        if self.mock:
            self.get_logger().info(f"[MOCK] Bomba {state_label} simulada com sucesso.")
            return

        if not self.pump_available:
            self.get_logger().warn(f"⚠️ [WARN] A bomba física está indisponível/desativada. Ignorando comando '{state_label}' e prosseguindo com a trajetória.")
            return

        cli = self.pump_on_cli if on else self.pump_off_cli
        req = Trigger.Request()
        
        try:
            fut = cli.call_async(req)
            t_start = time.time()
            while not fut.done() and (time.time() - t_start) < 2.0:
                time.sleep(0.05)
            
            if not fut.done():
                self.get_logger().warn(f"⚠️ [WARN] Timeout ao chamar serviço da bomba para '{state_label}'. Continuando movimento mesmo assim.")
                return
            res = fut.result()
            if res and res.success:
                self.get_logger().info(f"✓ Bomba: {res.message}")
            else:
                msg = res.message if res else "Sem resposta"
                self.get_logger().warn(f"⚠️ [WARN] Falha no controle da bomba: {msg}. Continuando trajetória.")
        except Exception as e:
            self.get_logger().warn(f"⚠️ [WARN] Erro ao chamar serviço da bomba: {e}. Continuando trajetória.")

    def goto(self, label, target_joints):
        self.get_logger().info(f"Movendo para: {label}...")
        
        if self.mock:
            self.get_logger().info(f"[MOCK] Simulando movimento para a pose: {label} (1.5s)...")
            time.sleep(1.5)
            self.current_joints = list(target_joints)
            self.get_logger().info(f"[MOCK] ✓ Chegou na pose {label}")
            return True

        # Prepara requisição do MoveGroup Goal
        mpr = MotionPlanRequest()
        mpr.group_name = GROUP
        mpr.allowed_planning_time = 5.0
        mpr.num_planning_attempts = 5
        mpr.max_velocity_scaling_factor = 0.20  # Velocidade 20%
        mpr.max_acceleration_scaling_factor = 0.20
        
        # Garante uso da leitura física atualizada do robô
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
        
        # Opções de Planejamento do MoveGroup (plan_only = False executa no hardware)
        po = PlanningOptions()
        po.plan_only = False
        
        goal = MoveGroup.Goal()
        goal.request = mpr
        goal.planning_options = po
        
        # Envia comando de ação ao MoveGroup
        send_goal_fut = self.move_cli.send_goal_async(goal)
        while not send_goal_fut.done():
            time.sleep(0.05)
        gh = send_goal_fut.result()
        if gh is None or not gh.accepted:
            raise RuntimeError(f"Planejamento/Execução rejeitado pelo MoveIt para a pose: {label}")
            
        result_fut = gh.get_result_async()
        while not result_fut.done():
            time.sleep(0.05)
        res = result_fut.result()
        
        if not res or res.result.error_code.val != 1:
            raise RuntimeError(f"Execução de movimento falhou para a pose: {label}. Código de erro: {res.result.error_code.val}")
        
        self.get_logger().info(f"✓ Chegou na pose {label}")

    def wait_for_target(self):
        self.get_logger().info("Aguardando detecção de objeto válido no scan...")
        self.triggered_class = None
        self.consecutive_frames = 0
        self.last_class = None
        self.last_valid_time = time.time()  # Inicializa contagem do timeout
        diag_interval = 3.0  # Intervalo entre logs de diagnóstico
        last_diag = 0.0
        
        while rclpy.ok() and self.triggered_class is None:
            time.sleep(0.1)
            
            now = time.time()
            
            # Diagnóstico periódico: mostra se há publishers conectados ao tópico
            if (now - last_diag) > diag_interval:
                last_diag = now
                try:
                    pub_count = self.count_publishers("/product_class")
                    sub_count = self.count_subscribers("/product_class")
                    self.get_logger().info(
                        f"📊 [DIAG] /product_class: {pub_count} publisher(s) | {sub_count} subscriber(s) | "
                        f"frames_acumulados={self.consecutive_frames} | last_class={self.last_class}")
                    if pub_count == 0:
                        self.get_logger().warn(
                            "⚠️ [DIAG] NENHUM publisher encontrado em /product_class! "
                            "Verifique: 1) cam_yolo_test.py está rodando? "
                            "2) RMW_IMPLEMENTATION é o mesmo no host e no container? "
                            "3) ROS_DOMAIN_ID=42 em ambos?")
                except Exception as e:
                    self.get_logger().warn(f"⚠️ [DIAG] Falha ao contar publishers: {e}")
            
            # Verifica timeout de inatividade de 5.0 segundos
            if (now - self.last_valid_time) > 5.0:
                if self.consecutive_frames > 0:
                    self.consecutive_frames = 0
                    self.last_class = None
                    self.get_logger().info("🔄 Timeout de inatividade atingido. Acumulador resetado na pose SCAN.")
                self.last_valid_time = now  # Evita repetição em loop imediato
                
        return self.triggered_class

def prompt_execution_mode(cli_mode):
    if cli_mode in ["auto", "manual"]:
        return cli_mode
    print("\n" + "="*50)
    print("      SELEÇÃO DE MODO DE EXECUÇÃO PICK & PLACE      ")
    print("="*50)
    print("  1. AUTOMÁTICO  - Execução contínua com pausa de 5s pós-coleta")
    print("  2. MANUAL      - Solicita confirmação [ENTER] para cada novo scan")
    print("="*50)
    try:
        choice = input("Escolha o modo (1-2) [Padrão: 1]: ").strip()
        if choice == "2":
            return "manual"
    except (KeyboardInterrupt, EOFError):
        pass
    return "auto"

def main():
    parser = argparse.ArgumentParser(description="Máquina de estados de Pick & Place")
    parser.add_argument("--mock", action="store_true", help="Ativa modo simulação (sem hardware real/MoveIt)")
    parser.add_argument("--mode", choices=["auto", "manual"], default=None, help="Modo de execução: 'auto' ou 'manual'")
    parser.add_argument("--cooldown", type=float, default=5.0, help="Tempo de espera/cooldown entre ciclos em segundos (Padrão: 5.0s)")
    args = parser.parse_args()

    exec_mode = prompt_execution_mode(args.mode)
    cooldown_sec = max(0.0, args.cooldown)

    rclpy.init()
    sm = PickAndPlaceStateMachine(mock=args.mock)
    
    try:
        sm.wait_ready()
        
        print("\n" + "="*50)
        print(f"      INICIANDO MÁQUINA DE ESTADOS PICK & PLACE      ")
        print(f"      Modo Hardware: {'SIMULAÇÃO (MOCK)' if sm.mock else 'ROBÔ REAL'} ")
        print(f"      Modo Operação: {exec_mode.upper()} ")
        print(f"      Cooldown Scan: {cooldown_sec:.1f}s ")
        print("="*50)

        # 1. Posição Inicial de Segurança: Mover para HOME (bem no alto)
        sm.reload_poses()
        sm.get_logger().info("Iniciando na pose inicial de segurança: HOME (alta altitude)...")
        sm.goto("home", sm.poses["home"])

        while rclpy.ok():
            # 2. Confirmação no modo MANUAL antes de abrir o scan
            if exec_mode == "manual":
                print("\n" + "-"*50)
                print("👉 [MODO MANUAL] O robô está aguardando permissão para realizar o SCAN.")
                ans = input("Pressione [ENTER] para iniciar o SCAN e detecção (ou 'q' para sair): ").strip().lower()
                if ans == "q":
                    sm.get_logger().info("Encerrando execução solicitada pelo usuário no modo manual.")
                    break

            # 3. Recarrega as poses dinamicamente no início de cada ciclo e move para SCAN
            sm.reload_poses()
            sm.goto("scan", sm.poses["scan"])
            
            # 4. Aguarda sinal do YOLO (3 frames seguidos de detecção válida)
            detected_class = sm.wait_for_target()
            sm.get_logger().info(f"Iniciando ciclo de pick & place para o objeto: {detected_class}")
            
            # 5. Sequência de Movimento Elevada & Bomba
            # 5.1. Aprox. e coleta: scan -> pick_approach -> pick
            sm.goto("pick_approach", sm.poses["pick_approach"])
            sm.goto("pick", sm.poses["pick"])
            
            # 5.2. Ligar bomba (Sucção ativa na pose de pick)
            sm.set_pump(True)
            
            # 5.3. Selagem do vácuo
            sm.get_logger().info("Selando vácuo / coletando objeto (1s)...")
            time.sleep(1.0)
            
            # 5.4. Subida Vertical pós-coleta para afastar da mesa
            sm.get_logger().info("Subindo verticalmente com o objeto (pick_approach)...")
            sm.goto("pick_approach", sm.poses["pick_approach"])
            
            # 5.5. Elevação para altitude segura (HOME) durante a transição horizontal
            sm.get_logger().info("Elevando braço para altitude de segurança (HOME) para evitar colisões na mesa...")
            sm.goto("home", sm.poses["home"])
            
            # 5.6. Mover para a posição de entrega: HOME -> place_approach -> place
            sm.goto("place_approach", sm.poses["place_approach"])
            sm.goto("place", sm.poses["place"])
            
            # 5.7. Desligar bomba
            sm.set_pump(False)
            
            # 5.8. Soltura do objeto
            sm.get_logger().info("Liberando objeto na mesa (1s)...")
            time.sleep(1.0)
            
            # 5.9. Subida Vertical pós-soltura
            sm.get_logger().info("Subindo verticalmente após soltura (place_approach)...")
            sm.goto("place_approach", sm.poses["place_approach"])
            
            # 5.10. Retorno seguro pelo alto para HOME
            sm.get_logger().info("Retornando pelo alto para HOME...")
            sm.goto("home", sm.poses["home"])
            
            # 6. Reseta acumuladores do YOLO e executa Cooldown pós-coleta
            sm.triggered_class = None
            sm.consecutive_frames = 0
            sm.last_class = None
            
            if exec_mode == "auto":
                sm.get_logger().info(f"⏳ Cooldown pós-coleta de {cooldown_sec:.1f}s ativado para evitar acúmulo de objetos na mesa...")
                time.sleep(cooldown_sec)
                
            sm.get_logger().info("✓ Ciclo concluído com sucesso!")
            print("\n" + "="*50 + "\n")

        # Ao finalizar o loop, retorna à posição HOME por segurança
        if rclpy.ok():
            sm.get_logger().info("Retornando o robô para a posição de repouso HOME...")
            sm.goto("home", sm.poses["home"])

    except KeyboardInterrupt:
        sm.get_logger().info("Interrompido pelo usuário. Desligando...")
    except Exception as e:
        sm.get_logger().error(f"Erro na máquina de estados: {e}")
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
