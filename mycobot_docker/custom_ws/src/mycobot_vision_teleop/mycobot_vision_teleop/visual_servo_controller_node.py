#!/usr/bin/env python3
"""
visual_servo_controller_node.py
================================
Controlador de visual servoing para eye-in-hand camera no myCobot 280 JN.

Arquitetura:
───────────────────────────────────────────────────────────────────────
  face_detector_node publishes → /face_detection/center  (PointStamped)
                                  /face_detection/detected (Bool)

  Este nó implementa:
    1. State machine: WAITING_FACE → TRACKING ↔ TEMPORARY_LOST → WAITING_FACE
    2. EMA (exponential moving average) para suavizar erros ruidosos
    3. Deadband: ignora movimento quando rosto já está centrado
    4. Lei de controle IBVS (Image-Based Visual Servo):
         ex = center_x - 0.5            (erro horizontal normalizado)
         ey = center_y - 0.5            (erro vertical normalizado)
         ex_f = α·ex + (1-α)·ex_prev   (filtro EMA)
         delta_j1 = sign_x · Kx · ex_f (pan  → joint1)
         delta_j2 = sign_y · Ky · ey_f (tilt → joint2)
    5. Integrador sobre posição COMANDADA (não lida do hardware cada tick)
       → resolve jitter causado por joint_states publicando a 10 Hz
       enquanto o controlador roda a 30 Hz.

Frames (olho-na-mão, câmera no punho link6):
    ex > 0 → rosto à DIREITA na imagem
    ey > 0 → rosto ABAIXO do centro
    sign_x, sign_y ajustam a convenção real do robô.

State machine:
───────────────────────────────────────────────────────────────────────
  WAITING_FACE:
    - Nenhum comando enviado.
    - Aguarda N detecções consecutivas com confiança ≥ threshold.
    - Entra em TRACKING quando condição satisfeita.

  TRACKING:
    - Computa erro filtrado, aplica deadband, envia delta de posição.
    - Se rosto perdido por > lost_frames_threshold ticks → TEMPORARY_LOST.

  TEMPORARY_LOST:
    - Para o robô (não envia comandos).
    - Mantém integrador no último alvo.
    - Se rosto reaparece → volta a TRACKING imediatamente.
    - Se timeout > lost_timeout_s → zera EMA e vai para WAITING_FACE.

Nota sobre MoveIt Servo:
───────────────────────────────────────────────────────────────────────
  O pacote ros-galactic-moveit-servo não está disponível neste Docker.
  Este controlador implementa comportamento equivalente publicando
  JointState incrementais em /joint_states_commands. O bridge no Nano
  usa set_fresh_mode(1): sempre executa o comando mais recente, dando
  efeito de velocity control contínuo.

  Para migrar para MoveIt Servo quando disponível:
    - Substituir _send_joints() por publish em /servo_server/delta_twist_cmds
    - Converter delta_j1 → wz (rad/s): wz = delta_j1 * rate_hz
    - Setar frame_id = 'link6' ou 'camera_optical_frame'
"""

import math
import enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# ── Enumeração de estados ─────────────────────────────────────────────

class State(enum.Enum):
    WAITING_FACE   = 0
    TRACKING       = 1
    TEMPORARY_LOST = 2


# ── Nomes de juntas — devem bater com mycobot_bridge.joint_names ──────
# bridge publica: joint2_to_joint1, joint3_to_joint2, joint4_to_joint3,
#                 joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6
J1 = 'joint2_to_joint1'
J2 = 'joint3_to_joint2'
J3 = 'joint4_to_joint3'
J4 = 'joint5_to_joint4'
J5 = 'joint6_to_joint5'
J6 = 'joint6output_to_joint6'

JOINT_NAMES = [J1, J2, J3, J4, J5, J6]

JOINT_LIMITS = {
    J1: (-2.0,  2.0),   # ±114.6°
    J2: (-1.5,  1.5),   # ±85.9°
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ── Nó principal ──────────────────────────────────────────────────────

class VisualServoControllerNode(Node):

    def __init__(self):
        super().__init__('visual_servo_controller')

        # ── Parâmetros de controle ────────────────────────────────────
        self.declare_parameter('rate_hz',               30.0)
        self.declare_parameter('kx',                     0.5)   # ganho horizontal
        self.declare_parameter('ky',                     0.3)   # ganho vertical
        self.declare_parameter('alpha',                  0.35)  # EMA: 0=congelado 1=bruto
        self.declare_parameter('deadband_x',             0.06)  # zona morta horizontal
        self.declare_parameter('deadband_y',             0.06)  # zona morta vertical
        self.declare_parameter('max_delta_rad',          0.08)  # passo máximo por tick
        # Sinais: ajuste +1/-1 se o robô se mover na direção errada
        self.declare_parameter('sign_x',                -1.0)   # rosto dir → gira CW
        self.declare_parameter('sign_y',                 1.0)   # rosto baixo → inclina
        self.declare_parameter('j2_offset',              0.4)   # posição de repouso j2 [rad]

        # ── Parâmetros da state machine ───────────────────────────────
        self.declare_parameter('required_consecutive',   3)     # detecções p/ entrar em TRACKING
        self.declare_parameter('lost_frames_threshold',  5)     # frames p/ TEMPORARY_LOST
        self.declare_parameter('lost_timeout_s',         2.0)   # s em TEMPORARY_LOST → WAITING_FACE

        rate      = self.get_parameter('rate_hz').value
        self._dt  = 1.0 / rate

        # ── Estado da state machine ───────────────────────────────────
        self.state               = State.WAITING_FACE
        self._enabled            = False
        self._consecutive_det    = 0    # detecções consecutivas (WAITING_FACE)
        self._consecutive_lost   = 0    # frames consecutivos sem rosto (TRACKING)
        self._lost_timer         = 0.0  # segundos em TEMPORARY_LOST
        self._home_tick          = 0    # throttle do home return (1/3 dos ticks)

        # ── Dados de entrada ──────────────────────────────────────────
        self._face_center      = None   # último PointStamped recebido
        self._face_detected    = False  # último Bool de /face_detection/detected
        self._joint_positions  = {}     # posições reais (10 Hz do bridge)

        # ── Integrador interno (posição COMANDADA) ────────────────────
        # CRÍTICO: joint_states só atualiza a 10 Hz mas loop roda a 30 Hz.
        # Ler j1_now do hardware entre updates dá o mesmo valor por 3 ticks
        # consecutivos → deltas idênticos → alvo não avança → jitter.
        # Solução: integrar sobre _j1_cmd (interno), sincronizado do hardware
        # apenas na ativação.
        self._j1_cmd = None
        self._j2_cmd = None

        # ── Filtro EMA ────────────────────────────────────────────────
        self._ex_f = 0.0
        self._ey_f = 0.0

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(
            PointStamped, '/face_detection/center',   self._center_cb,   10)
        self.create_subscription(
            Bool,         '/face_detection/detected', self._detected_cb, 10)
        self.create_subscription(
            JointState,   '/joint_states',            self._js_cb,       10)
        self.create_subscription(
            Bool,         '/visual_servo/enabled',    self._enable_cb,   10)

        # ── Publishers ────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(JointState, '/joint_states_commands', 10)
        self._state_pub  = self.create_publisher(Bool,       '/visual_servo/active',   10)

        # ── Timer de controle ─────────────────────────────────────────
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'VisualServoController | {rate:.0f} Hz | '
            f'Kx={self.get_parameter("kx").value} Ky={self.get_parameter("ky").value} | '
            f'alpha={self.get_parameter("alpha").value} | '
            f'max_delta={math.degrees(self.get_parameter("max_delta_rad").value):.1f}°\n'
            '  Ativar:    ros2 topic pub --once /visual_servo/enabled std_msgs/Bool "data: true"\n'
            '  Desativar: ros2 topic pub --once /visual_servo/enabled std_msgs/Bool "data: false"'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks de entrada
    # ──────────────────────────────────────────────────────────────────

    def _center_cb(self, msg: PointStamped):
        self._face_center = msg

    def _detected_cb(self, msg: Bool):
        self._face_detected = msg.data

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_positions[name] = pos

    def _enable_cb(self, msg: Bool):
        if msg.data and not self._enabled:
            self.get_logger().info('Visual servo ATIVADO — aguardando rosto...')
            self._reset()
        elif not msg.data and self._enabled:
            self.get_logger().info('Visual servo DESATIVADO')
            self.state = State.WAITING_FACE
        self._enabled = msg.data

    # ──────────────────────────────────────────────────────────────────
    # Loop de controle principal
    # ──────────────────────────────────────────────────────────────────

    def _control_loop(self):
        active = self._enabled and self.state == State.TRACKING
        self._state_pub.publish(Bool(data=active))

        if not self._enabled:
            return

        det = self._face_detected

        # WAITING_FACE: só precisa de detecção — não precisa de joint_states
        if self.state == State.WAITING_FACE:
            self._tick_waiting(det)
            return

        # Sincroniza integrador: usa posição real se disponível, senão usa defaults seguros
        if self._j1_cmd is None:
            j2_off       = self.get_parameter('j2_offset').value
            self._j1_cmd = self._joint_positions.get(J1, 0.0)
            self._j2_cmd = self._joint_positions.get(J2, j2_off)
            if J1 not in self._joint_positions:
                self.get_logger().warn(
                    f'joint_states sem "{J1}" — iniciando integrador em defaults. '
                    'Verifique bridge no Nano.',
                    throttle_duration_sec=10.0
                )

        if self.state == State.TRACKING:
            self._tick_tracking(det)
        elif self.state == State.TEMPORARY_LOST:
            self._tick_lost(det)

    # ──────────────────────────────────────────────────────────────────
    # WAITING_FACE
    # ──────────────────────────────────────────────────────────────────

    def _tick_waiting(self, detected: bool):
        req = self.get_parameter('required_consecutive').value
        self.get_logger().info(
            f'[WAITING] det={detected} consecutive={self._consecutive_det}/{req}',
            throttle_duration_sec=1.0
        )

        # Retorno lento à posição de repouso (10 Hz — cada 3 ticks a 30 Hz).
        # Evita que o braço fique travado numa posição extrema após face perdida.
        if self._j1_cmd is not None:
            self._home_tick = (self._home_tick + 1) % 3
            if self._home_tick == 0:
                j2_off    = self.get_parameter('j2_offset').value
                home_step = 0.008   # ~0.46°/tick → ~4.6°/s de retorno a 10 Hz
                j1_err = 0.0   - self._j1_cmd
                j2_err = j2_off - self._j2_cmd
                moved  = False
                if abs(j1_err) > 0.01:
                    self._j1_cmd = clamp(self._j1_cmd + clamp(j1_err, -home_step, home_step),
                                         *JOINT_LIMITS[J1])
                    moved = True
                if abs(j2_err) > 0.01:
                    self._j2_cmd = clamp(self._j2_cmd + clamp(j2_err, -home_step, home_step),
                                         *JOINT_LIMITS[J2])
                    moved = True
                if moved:
                    self._send_joints()

        if detected:
            self._consecutive_det += 1
            if self._consecutive_det >= req:
                self.get_logger().info(
                    f'[WAITING_FACE → TRACKING] '
                    f'{req} detecções consecutivas confirmadas.'
                )
                self.state             = State.TRACKING
                self._consecutive_lost = 0
                self._lost_timer       = 0.0
                # Resetar EMA para entrada suave
                if self._face_center is not None:
                    self._ex_f = self._face_center.point.x - 0.5
                    self._ey_f = self._face_center.point.y - 0.5
                else:
                    self._ex_f = 0.0
                    self._ey_f = 0.0
        else:
            self._consecutive_det = 0

    # ──────────────────────────────────────────────────────────────────
    # TRACKING
    # ──────────────────────────────────────────────────────────────────

    def _tick_tracking(self, detected: bool):
        lost_thresh = self.get_parameter('lost_frames_threshold').value

        if detected and self._face_center is not None:
            self._consecutive_lost = 0
            self._lost_timer       = 0.0
            self._do_control()
        else:
            self._consecutive_lost += 1
            if self._consecutive_lost >= lost_thresh:
                self.get_logger().info(
                    f'[TRACKING → TEMPORARY_LOST] '
                    f'Rosto perdido por {lost_thresh} frames consecutivos.'
                )
                self.state       = State.TEMPORARY_LOST
                self._lost_timer = 0.0

    # ──────────────────────────────────────────────────────────────────
    # TEMPORARY_LOST
    # ──────────────────────────────────────────────────────────────────

    def _tick_lost(self, detected: bool):
        timeout = self.get_parameter('lost_timeout_s').value

        if detected and self._face_center is not None:
            self.get_logger().info('[TEMPORARY_LOST → TRACKING] Rosto reapareceu.')
            self.state             = State.TRACKING
            self._consecutive_lost = 0
            self._lost_timer       = 0.0
            # Reset EMA com posição atual — evita movimento brusco causado por
            # erro filtrado obsoleto quando o rosto reaparece em posição diferente
            self._ex_f = self._face_center.point.x - 0.5
            self._ey_f = self._face_center.point.y - 0.5
        else:
            self._lost_timer += self._dt
            if self._lost_timer >= timeout:
                self.get_logger().info(
                    f'[TEMPORARY_LOST → WAITING_FACE] '
                    f'Timeout {timeout:.1f}s atingido.'
                )
                self.state            = State.WAITING_FACE
                self._consecutive_det = 0
                self._lost_timer      = 0.0
                # Decai EMA — reset suave
                self._ex_f = 0.0
                self._ey_f = 0.0
            # TEMPORARY_LOST: não envia comandos — mantém última posição

    # ──────────────────────────────────────────────────────────────────
    # Lei de controle IBVS
    # ──────────────────────────────────────────────────────────────────

    def _do_control(self):
        alpha      = self.get_parameter('alpha').value
        db_x       = self.get_parameter('deadband_x').value
        db_y       = self.get_parameter('deadband_y').value
        kx         = self.get_parameter('kx').value
        ky         = self.get_parameter('ky').value
        max_d      = self.get_parameter('max_delta_rad').value
        sign_x     = self.get_parameter('sign_x').value
        sign_y     = self.get_parameter('sign_y').value

        pt = self._face_center

        # Erro normalizado: centro da imagem = 0, bordas = ±0.5
        ex = pt.point.x - 0.5
        ey = pt.point.y - 0.5

        # EMA — suaviza ruído de detecção frame-a-frame
        self._ex_f = alpha * ex + (1.0 - alpha) * self._ex_f
        self._ey_f = alpha * ey + (1.0 - alpha) * self._ey_f

        # Deadband: dentro da zona central → sem movimento
        ex_c = self._ex_f if abs(self._ex_f) >= db_x else 0.0
        ey_c = self._ey_f if abs(self._ey_f) >= db_y else 0.0

        if ex_c == 0.0 and ey_c == 0.0:
            return  # rosto centrado — mantém posição

        # Mapeamento erro → delta de posição
        # Normalização: ex_c ∈ [-0.5, 0.5]; divisão por 0.5 escala para [-1,1]
        # Depois multiplica pelo passo máximo e pelo ganho
        delta_j1 = clamp(sign_x * kx * (ex_c / 0.5) * max_d, -max_d, max_d)
        delta_j2 = clamp(sign_y * ky * (ey_c / 0.5) * max_d, -max_d, max_d)

        # Limiar = 5% do passo máximo (escala com max_delta_rad)
        min_step = max_d * 0.05
        if abs(delta_j1) < min_step and abs(delta_j2) < min_step:
            return  # passo efetivo mínimo — evita ruído de quantização

        # Calcula novo alvo e só avança integrador se o envio for bem-sucedido
        j1_new = clamp(self._j1_cmd + delta_j1, *JOINT_LIMITS[J1])
        j2_new = clamp(self._j2_cmd + delta_j2, *JOINT_LIMITS[J2])
        self._j1_cmd, self._j2_cmd = j1_new, j2_new
        if not self._send_joints():
            return  # joint_states incompleto — reverte não necessário (guarda idempotente)

        self.get_logger().info(
            f'[CTRL] ex={ex:+.3f} exf={self._ex_f:+.3f} | '
            f'dj1={math.degrees(delta_j1):+.2f}° dj2={math.degrees(delta_j2):+.2f}° | '
            f'j1={math.degrees(self._j1_cmd):.1f}° j2={math.degrees(self._j2_cmd):.1f}°',
            throttle_duration_sec=0.5
        )

    # ──────────────────────────────────────────────────────────────────
    # Envio de comando (fire-and-forget; bridge usa fresh_mode=1)
    # ──────────────────────────────────────────────────────────────────

    def _send_joints(self):
        # Não envia se j3-j6 ainda desconhecidos (evita mover juntas para 0° inadvertidamente)
        for name in (J3, J4, J5, J6):
            if name not in self._joint_positions:
                self.get_logger().warn(
                    f'joint_positions sem "{name}" — aguardando bridge',
                    throttle_duration_sec=3.0)
                return False

        positions = []
        for name in JOINT_NAMES:
            if name == J1:
                positions.append(self._j1_cmd)
            elif name == J2:
                positions.append(self._j2_cmd)
            else:
                positions.append(self._joint_positions[name])

        msg          = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = positions
        self._cmd_pub.publish(msg)
        return True

    # ──────────────────────────────────────────────────────────────────
    # Auxiliares
    # ──────────────────────────────────────────────────────────────────

    def _reset(self):
        """Reseta toda a state machine e integrador."""
        self.state            = State.WAITING_FACE
        self._consecutive_det = 0
        self._consecutive_lost = 0
        self._lost_timer      = 0.0
        self._ex_f            = 0.0
        self._ey_f            = 0.0
        self._j1_cmd          = None   # re-sinc com hardware no próximo tick
        self._j2_cmd          = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisualServoControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[visual_servo_controller] {e}')
        import traceback; traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
