#!/usr/bin/env python3
"""
face_follower_node.py — IBVS para joint1/joint2 do myCobot.

Como face → juntas (câmera no punho, apontando para a pessoa):
──────────────────────────────────────────────────────────────
  ex = face.x − 0.5   (>0 = rosto à DIREITA na imagem)
  ey = face.y − 0.5   (>0 = rosto ABAIXO do centro)

  joint1 (pan horizontal):
    delta_j1 = −kp_x · ease(ex)
    Rosto à direita → ex>0 → delta negativo → joint1 gira CW → câmera segue

  joint2 (tilt vertical):
    delta_j2 = +kp_y · ease(ey)
    Rosto abaixo  → ey>0 → delta positivo → joint2 inclina para baixo

  Mapping é ângulo pequeno: 1 px deslocado ≈ fov/res rad na junta.
  Ganhos kp_x/kp_y afinam esse fator sem precisar modelar a câmera.

Por que "passitos travados":
──────────────────────────────────────────────────────────────
  joint_states (posição real) atualiza a 10 Hz no bridge.
  Controlador rodava a 30 Hz e lia j1_now do hardware a cada tick.
  → 3 ticks consecutivos com mesma base → deltas idênticos → alvo não avança
    entre updates → movimento em degraus sincronizados com 10 Hz.

  FIX: integrar sobre _j1_cmd (posição COMANDADA interna), não lida do HW.
  Inicializa de joint_states apenas uma vez (ao habilitar).

Protocolo de busca:
──────────────────────────────────────────────────────────────
  Quando tracking_ok=False após ter rastreado, extrapola na última
  direção conhecida de ex (pan). Continua até re-detectar o rosto ou
  até search_timeout_s. Sem varredura vertical (rosto fica em altura similar).
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


JOINT_NAMES = [
    'cobot_joint_1', 'cobot_joint_2', 'cobot_joint_3',
    'cobot_joint_4', 'cobot_joint_5', 'cobot_joint_6',
]

JOINT_LIMITS = {
    'cobot_joint_1': (-2.0,  2.0),
    'cobot_joint_2': (-1.5,  1.5),
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _ease_inout_cubic(t: float) -> float:
    """S-curve: suave perto do centro, agressivo longe."""
    if t <= 0.5:
        return 4.0 * t ** 3
    return 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0


def ease_error(error: float, deadband: float, max_error: float = 0.42) -> float:
    """Mapeia erro → saída eased em [-1,1], zero dentro do deadband.
    max_error alinhado com outer_zone do vision_node (0.42).
    """
    if abs(error) <= deadband:
        return 0.0
    t = min(1.0, (abs(error) - deadband) / (max_error - deadband))
    return math.copysign(_ease_inout_cubic(t), error)


class FaceFollowerNode(Node):
    def __init__(self):
        super().__init__('face_follower')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('kp_x',             0.5)
        self.declare_parameter('kp_y',             0.3)
        self.declare_parameter('deadband',          0.06)
        self.declare_parameter('max_delta_rad',     0.08)
        self.declare_parameter('rate_hz',           20.0)
        self.declare_parameter('j2_offset',         0.4)
        self.declare_parameter('invert_x',          False)
        # Busca
        self.declare_parameter('search_delta_rad',  0.04)  # rad/tick na busca (lento)
        self.declare_parameter('search_timeout_s',  3.0)   # seg antes de desistir

        rate = self.get_parameter('rate_hz').value

        # ── Estado de rastreio ────────────────────────────────────────
        self.face_center    = None
        self.tracking_ok    = False
        self.joint_positions = {}
        self.enabled        = False

        # Integrador interno: posição COMANDADA (não lida do HW a cada tick)
        self._j1_cmd  = None   # inicializado de joint_states ao habilitar
        self._j2_cmd  = None

        # ── Estado de busca ───────────────────────────────────────────
        self._search_dir_x   = 1.0   # último sinal de ex: +1 direita / -1 esquerda
        self._search_ticks   = 0     # ticks consecutivos em modo busca
        self._was_tracking   = False  # detecta transição rastreando → perdido

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Point,      '/human/face_center',     self._face_cb,     10)
        self.create_subscription(Bool,       '/human/tracking_ok',     self._tracking_cb, 10)
        self.create_subscription(JointState, '/joint_states',          self._js_cb,       10)
        self.create_subscription(Bool,       '/face_follower/enabled', self._enable_cb,   10)

        # ── Publishers ────────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(JointState, '/joint_states_commands', 10)
        self._pub_status = self.create_publisher(Bool, '/face_follower/active', 10)

        # ── Timer de controle ─────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'FaceFollower — {rate:.0f} Hz | '
            f'deadband={self.get_parameter("deadband").value:.0%} | '
            f'max_delta={math.degrees(self.get_parameter("max_delta_rad").value):.1f}° | '
            f'busca={self.get_parameter("search_timeout_s").value:.0f}s\n'
            '  Habilitar: ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def _face_cb(self, msg: Point):
        self.face_center = msg

    def _tracking_cb(self, msg: Bool):
        self.tracking_ok = msg.data

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def _enable_cb(self, msg: Bool):
        if msg.data and not self.enabled:
            self.get_logger().info('Face follower HABILITADO')
            # Sincroniza integrador com posição real ao habilitar
            self._j1_cmd = None
            self._j2_cmd = None
            self._search_ticks = 0
            self._was_tracking = False
        elif not msg.data and self.enabled:
            self.get_logger().info('Face follower DESABILITADO')
        self.enabled = msg.data

    # ──────────────────────────────────────────────────────────────────
    # Loop de controle
    # ──────────────────────────────────────────────────────────────────

    def _control_loop(self):
        self._pub_status.publish(Bool(data=self.enabled and self.tracking_ok))

        if not self.enabled or not self.joint_positions:
            return

        # Inicializa integrador da posição real (apenas uma vez por sessão)
        if self._j1_cmd is None:
            j2_off = self.get_parameter('j2_offset').value
            self._j1_cmd = self.joint_positions.get('cobot_joint_1', 0.0)
            self._j2_cmd = self.joint_positions.get('cobot_joint_2', j2_off)

        rate      = self.get_parameter('rate_hz').value
        max_d     = self.get_parameter('max_delta_rad').value
        search_d  = self.get_parameter('search_delta_rad').value
        search_to = self.get_parameter('search_timeout_s').value
        search_max_ticks = int(search_to * rate)

        if self.tracking_ok and self.face_center is not None:
            # ── MODO RASTREIO ─────────────────────────────────────────
            self._search_ticks = 0
            self._was_tracking = True
            self._do_track(max_d)

        elif self._was_tracking and self._search_ticks < search_max_ticks:
            # ── MODO BUSCA ────────────────────────────────────────────
            self._search_ticks += 1
            self._do_search(search_d, search_max_ticks)

        else:
            # ── IDLE ──────────────────────────────────────────────────
            if self._was_tracking and self._search_ticks >= search_max_ticks:
                self.get_logger().warn(
                    'Busca expirou — parado. Aguardando rosto.',
                    throttle_duration_sec=3.0
                )
                self._was_tracking = False

    def _do_track(self, max_d: float):
        kp_x     = self.get_parameter('kp_x').value
        kp_y     = self.get_parameter('kp_y').value
        deadband = self.get_parameter('deadband').value
        inv_x    = self.get_parameter('invert_x').value

        ex = self.face_center.x - 0.5
        ey = self.face_center.y - 0.5
        if inv_x:
            ex = -ex

        # Guarda direção para protocolo de busca
        if abs(ex) > deadband:
            self._search_dir_x = math.copysign(1.0, ex)

        ex_eased = ease_error(ex, deadband)
        ey_eased = ease_error(ey, deadband)

        # Deltas: integrados sobre posição COMANDADA (não posição real)
        delta_j1 = clamp(-kp_x * ex_eased, -max_d, max_d)
        delta_j2 = clamp( kp_y * ey_eased, -max_d, max_d)

        if abs(delta_j1) < 0.005 and abs(delta_j2) < 0.005:
            return

        self._j1_cmd = clamp(self._j1_cmd + delta_j1, *JOINT_LIMITS['cobot_joint_1'])
        self._j2_cmd = clamp(self._j2_cmd + delta_j2, *JOINT_LIMITS['cobot_joint_2'])
        self._send_joints()

        self.get_logger().debug(
            f'TRACK ex={ex:+.3f} ey={ey:+.3f} | '
            f'j1_cmd={math.degrees(self._j1_cmd):.1f}° '
            f'j2_cmd={math.degrees(self._j2_cmd):.1f}°'
        )

    def _do_search(self, search_d: float, search_max_ticks: int):
        """Extrapola na última direção de ex para tentar re-encontrar o rosto."""
        # Rotaciona lentamente na última direção conhecida
        delta_j1 = -self._search_dir_x * search_d
        self._j1_cmd = clamp(self._j1_cmd + delta_j1, *JOINT_LIMITS['cobot_joint_1'])
        # Mantém j2 (altura) — não varre verticalmente
        self._send_joints()

        if self._search_ticks % 10 == 1:
            self.get_logger().info(
                f'BUSCA [{self._search_ticks}/{search_max_ticks}] '
                f'dir={self._search_dir_x:+.0f} '
                f'j1={math.degrees(self._j1_cmd):.1f}°'
            )

    # ──────────────────────────────────────────────────────────────────
    # Envio (fire-and-forget; bridge usa fresh_mode=1 no Nano)
    # ──────────────────────────────────────────────────────────────────

    def _send_joints(self):
        positions = []
        for name in JOINT_NAMES:
            if name == 'cobot_joint_1':
                positions.append(self._j1_cmd)
            elif name == 'cobot_joint_2':
                positions.append(self._j2_cmd)
            else:
                positions.append(self.joint_positions.get(name, 0.0))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = JOINT_NAMES
        msg.position = positions
        self._cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaceFollowerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[face_follower] Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
