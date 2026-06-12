import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from pymycobot.mycobot280 import MyCobot280
import math
import time

# Jetson.GPIO só existe no Nano — import lazy para permitir
# rodar em mock no PC sem a biblioteca instalada.
try:
    import Jetson.GPIO as GPIO
except ImportError:
    GPIO = None

# Suction Pump V2.0 — fiação oficial 280 JN (BCM):
#   G5 → pino 20: válvula solenoide (LOW = sucção ON, HIGH = fechada)
#   G2 → pino 21: válvula de deflação (pulso LOW de ~1s solta o objeto)
PUMP_SOLENOID_PIN = 20
PUMP_VALVE_PIN = 21


class MyCobotBridge(Node):
    def __init__(self):
        super().__init__('mycobot_bridge')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('mock', False)
        # Velocidade para comandos diretos (face tracking).
        # 20-40 = suave para visual servoing; 70-100 = rápido mas brusco.
        self.declare_parameter('tracking_speed', 30)
        # Velocidade para trajectórias MoveIt (plan+execute).
        self.declare_parameter('moveit_speed', 60)

        self.port           = self.get_parameter('port').get_parameter_value().string_value
        self.baud           = self.get_parameter('baud').get_parameter_value().integer_value
        self.mock           = self.get_parameter('mock').get_parameter_value().bool_value
        self.tracking_speed = self.get_parameter('tracking_speed').value
        self.moveit_speed   = self.get_parameter('moveit_speed').value
        self._mock_angles_deg = [0.0] * 6

        if not self.mock:
            self.mc = MyCobot280(self.port, self.baud)
            time.sleep(1.0)
            # Fresh mode: sempre executa o comando mais recente,
            # descartando comandos antigos na fila — essencial para
            # visual servoing contínuo (face tracking).
            try:
                self.mc.set_fresh_mode(1)
                time.sleep(0.1)
                self.get_logger().info('Fresh mode ATIVO (comandos mais recentes têm prioridade)')
            except Exception as e:
                self.get_logger().warn(f'set_fresh_mode não disponível: {e}')

        # Nomes oficiais do driver elephantrobotics
        self.joint_names = [
            "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
            "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"
        ]
        self._last_valid_angles_deg = list(self._mock_angles_deg)

        # Publica raw para o relay no PC re-carimbar com clock local
        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Controle direto — face tracking (fire-and-forget, fresh mode)
        self.cmd_sub = self.create_subscription(
            JointState, 'joint_states_commands', self.command_callback, 10)

        # Action server — MoveIt plan+execute
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            self.execute_callback)

        # ── Suction Pump V2.0 (Jetson.GPIO — não há placa Basic no JN,
        #    portanto set_basic_output NÃO funciona neste modelo) ──────
        self._pump_active = False
        self._valve_timer = None
        self._gpio_ready = False
        if not self.mock:
            if GPIO is None:
                self.get_logger().warn(
                    'Jetson.GPIO indisponível — serviços da pump em modo degradado')
            else:
                try:
                    GPIO.setmode(GPIO.BCM)
                    # HIGH = válvulas fechadas (pump desligada) no arranque
                    GPIO.setup(PUMP_SOLENOID_PIN, GPIO.OUT, initial=GPIO.HIGH)
                    GPIO.setup(PUMP_VALVE_PIN, GPIO.OUT, initial=GPIO.HIGH)
                    self._gpio_ready = True
                except Exception as e:
                    self.get_logger().error(f'Falha ao inicializar GPIO da pump: {e}')
        self.create_service(Trigger, 'pump_on',  self._pump_on_cb)
        self.create_service(Trigger, 'pump_off', self._pump_off_cb)
        self._pump_state_pub = self.create_publisher(Bool, 'pump_state', 10)
        self.create_timer(0.5, self._publish_pump_state)

        self.get_logger().info(
            f'MyCobot Bridge | mock={self.mock} | port={self.port} | '
            f'tracking_speed={self.tracking_speed} | moveit_speed={self.moveit_speed} | '
            f'pump services: pump_on / pump_off')

    def command_callback(self, msg):
        """Controle direto para face tracking — executa imediatamente."""
        angles = [math.degrees(x) for x in msg.position]
        if len(angles) < 6:
            return
        if self.mock:
            self._mock_angles_deg = angles[:6]
            return
        self.mc.send_angles(angles[:6], self.tracking_speed)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mycobot_base_link'
        msg.name = self.joint_names
        if self.mock:
            msg.position = [math.radians(x) for x in self._mock_angles_deg]
        else:
            try:
                angles = self.mc.get_angles()
            except Exception as e:
                self.get_logger().warn(f'Falha ao ler ângulos do MyCobot: {e}', throttle_duration_sec=5.0)
                angles = None

            if isinstance(angles, list) and len(angles) == 6:
                self._last_valid_angles_deg = [float(x) for x in angles]
            else:
                self.get_logger().warn(f'Leitura inválida de ângulos: {angles}', throttle_duration_sec=5.0)

            msg.position = [math.radians(x) for x in self._last_valid_angles_deg]
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        self.joint_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        """Executa trajectória do MoveIt (plan+execute do RViz).

        Respeita o time_from_start de cada ponto (time parameterization
        do MoveIt) em vez de tocar tudo a 20 pontos/s — sem isso o
        movimento vira 'teleporte' no RViz e fica brusco no robô real.
        """
        self.get_logger().info('Trajetória MoveIt recebida')
        trajectory = goal_handle.request.trajectory
        prev_t = 0.0
        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            angles_deg = [math.degrees(p) for p in point.positions]
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = t - prev_t
            prev_t = t
            if dt <= 0.0:
                dt = 0.1   # fallback: trajetória sem time parameterization
            dt = min(dt, 2.0)
            if self.mock:
                # interpola entre pontos p/ animação suave no RViz
                start = list(self._mock_angles_deg)
                steps = max(int(dt / 0.05), 1)
                for i in range(1, steps + 1):
                    a = i / steps
                    self._mock_angles_deg = [
                        s + (g - s) * a for s, g in zip(start, angles_deg[:6])]
                    time.sleep(dt / steps)
            else:
                self.mc.send_angles(angles_deg, self.moveit_speed)
                time.sleep(dt)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()


    # ── Pump helpers ───────────────────────────────────────────────────

    def _cancel_valve_timer(self):
        if self._valve_timer is not None:
            self._valve_timer.cancel()
            self._valve_timer = None

    def _close_valve(self):
        """One-shot: fecha a válvula de deflação após o pulso de 1s."""
        self._cancel_valve_timer()
        if self._gpio_ready:
            GPIO.output(PUMP_VALVE_PIN, GPIO.HIGH)

    def _pump_on_cb(self, request, response):
        if self._pump_active:
            response.success = True
            response.message = 'Pump already active'
            return response
        try:
            if not self.mock:
                if not self._gpio_ready:
                    response.success = False
                    response.message = 'GPIO indisponível (Jetson.GPIO não inicializado)'
                    return response
                self._close_valve()                            # garante deflação fechada
                GPIO.output(PUMP_SOLENOID_PIN, GPIO.LOW)       # abre solenoide → sucção
                time.sleep(0.05)
            self._pump_active = True
            self.get_logger().info('Pump ON')
            response.success = True
            response.message = 'Pump activated'
        except Exception as e:
            response.success = False
            response.message = f'pump_on failed: {e}'
            self.get_logger().error(str(e))
        return response

    def _pump_off_cb(self, request, response):
        try:
            if not self.mock:
                if not self._gpio_ready:
                    response.success = False
                    response.message = 'GPIO indisponível (Jetson.GPIO não inicializado)'
                    return response
                GPIO.output(PUMP_SOLENOID_PIN, GPIO.HIGH)      # fecha solenoide
                time.sleep(0.05)
                GPIO.output(PUMP_VALVE_PIN, GPIO.LOW)          # abre deflação → solta objeto
                # Pulso de 1s fechado por timer para não bloquear o
                # executor (joint_states continuam a 10Hz durante o pulso)
                self._cancel_valve_timer()
                self._valve_timer = self.create_timer(1.0, self._close_valve)
            self._pump_active = False
            self.get_logger().info('Pump OFF')
            response.success = True
            response.message = 'Pump deactivated'
        except Exception as e:
            response.success = False
            response.message = f'pump_off failed: {e}'
            self.get_logger().error(str(e))
        return response

    def _publish_pump_state(self):
        msg = Bool()
        msg.data = self._pump_active
        self._pump_state_pub.publish(msg)

    def destroy_node(self):
        if self._gpio_ready:
            try:
                GPIO.output(PUMP_SOLENOID_PIN, GPIO.HIGH)
                GPIO.output(PUMP_VALVE_PIN, GPIO.HIGH)
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()   # fecha válvulas + GPIO.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
