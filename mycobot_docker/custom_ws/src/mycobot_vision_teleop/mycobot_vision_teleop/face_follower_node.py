#!/usr/bin/env python3
"""
face_follower_node.py
=====================
Controlador proporcional com easing cúbico: detecta posição do rosto e move
joint1/joint2 do myCobot para que o braço "aponte" na direção do rosto.

Arquitetura de controle (modo direto — sem overhead de action):
  Publica JointState em /joint_states_commands (topic, fire-and-forget).
  A bridge no Nano (set_fresh_mode=1) sempre executa o comando mais recente,
  descartando comandos antigos — efeito de "velocity control" contínuo.

Easing cúbico In/Out:
  Movimentos suaves perto do centro (baixo erro), agressivos longe (alto erro).
  Implementação baseada em: DOI 10.3390/s22020572 (eq. 25).

Ativação:
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: false"

Parâmetros ROS:
  kp_x          (float, default 0.5)   ganho horizontal → joint1
  kp_y          (float, default 0.3)   ganho vertical   → joint2
  deadband      (float, default 0.06)  zona morta (6% da imagem)
  max_delta_rad (float, default 0.08)  movimento máximo por step [rad]
  rate_hz       (float, default 20.0)  frequência do controlador
  j2_offset     (float, default 0.4)   inclinação base do joint2 [rad]
  invert_x      (bool,  default False) inverte sentido horizontal
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
    """Smooth S-curve: t∈[0,1] → [0,1]. Gentle at edges, fast in middle."""
    if t <= 0.5:
        return 4.0 * t ** 3
    return 1.0 - (-2.0 * t + 2.0) ** 3 / 2.0


def ease_error(error: float, deadband: float, max_error: float = 0.5) -> float:
    """Map raw error to eased output in [-1, 1], zero inside deadband."""
    if abs(error) <= deadband:
        return 0.0
    t = min(1.0, (abs(error) - deadband) / (max_error - deadband))
    return math.copysign(_ease_inout_cubic(t), error)


class FaceFollowerNode(Node):
    def __init__(self):
        super().__init__('face_follower')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('kp_x',          0.5)
        self.declare_parameter('kp_y',          0.3)
        self.declare_parameter('deadband',       0.06)
        self.declare_parameter('max_delta_rad',  0.08)
        self.declare_parameter('rate_hz',        20.0)
        self.declare_parameter('j2_offset',      0.4)
        self.declare_parameter('invert_x',       False)

        rate = self.get_parameter('rate_hz').value

        # ── Estado interno ────────────────────────────────────────────
        self.face_center = None
        self.tracking_ok = False
        self.joint_positions = {}
        self.enabled = False

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Point,      '/human/face_center',     self._face_cb,     10)
        self.create_subscription(Bool,       '/human/tracking_ok',     self._tracking_cb, 10)
        self.create_subscription(JointState, '/joint_states',          self._js_cb,       10)
        self.create_subscription(Bool,       '/face_follower/enabled', self._enable_cb,   10)

        # ── Publicadores ─────────────────────────────────────────────
        # Controle direto: bridge executa imediatamente (fresh_mode=1 no Nano)
        self._cmd_pub   = self.create_publisher(JointState, '/joint_states_commands', 10)
        self._pub_status = self.create_publisher(Bool, '/face_follower/active', 10)

        # ── Timer de controle ─────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'FaceFollower pronto — {rate:.0f} Hz | '
            f'deadband={self.get_parameter("deadband").value:.0%} | '
            f'max_delta={math.degrees(self.get_parameter("max_delta_rad").value):.1f}°\n'
            '  Habilitar:   ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"\n'
            '  Desabilitar: ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: false"'
        )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────

    def _face_cb(self, msg: Point):
        self.face_center = msg

    def _tracking_cb(self, msg: Bool):
        self.tracking_ok = msg.data
        if not msg.data and self.enabled:
            self.get_logger().warn('Tracking perdido — aguardando rosto', throttle_duration_sec=2.0)

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.joint_positions[name] = pos

    def _enable_cb(self, msg: Bool):
        if msg.data and not self.enabled:
            self.get_logger().info('Face follower HABILITADO')
        elif not msg.data and self.enabled:
            self.get_logger().info('Face follower DESABILITADO')
        self.enabled = msg.data

    # ──────────────────────────────────────────────────────────────────
    # Loop de controle
    # ──────────────────────────────────────────────────────────────────

    def _control_loop(self):
        active = self.enabled and self.tracking_ok
        self._pub_status.publish(Bool(data=active))

        if not active:
            return
        if self.face_center is None or not self.joint_positions:
            return

        kp_x     = self.get_parameter('kp_x').value
        kp_y     = self.get_parameter('kp_y').value
        deadband = self.get_parameter('deadband').value
        max_d    = self.get_parameter('max_delta_rad').value
        j2_off   = self.get_parameter('j2_offset').value
        inv_x    = self.get_parameter('invert_x').value

        ex = self.face_center.x - 0.5
        ey = self.face_center.y - 0.5
        if inv_x:
            ex = -ex

        # Easing cúbico: suave perto do centro, agressivo longe
        ex_eased = ease_error(ex, deadband)
        ey_eased = ease_error(ey, deadband)
        delta_j1 = clamp(-kp_x * ex_eased, -max_d, max_d)
        delta_j2 = clamp( kp_y * ey_eased, -max_d, max_d)

        if abs(delta_j1) < 0.005 and abs(delta_j2) < 0.005:
            return

        j1_now = self.joint_positions.get('cobot_joint_1', 0.0)
        j2_now = self.joint_positions.get('cobot_joint_2', j2_off)

        j1_target = clamp(j1_now + delta_j1, *JOINT_LIMITS['cobot_joint_1'])
        j2_target = clamp(j2_now + delta_j2, *JOINT_LIMITS['cobot_joint_2'])

        self._send_joints(j1_target, j2_target)

        self.get_logger().debug(
            f'ex={ex:.3f} ey={ey:.3f} | '
            f'j1={math.degrees(j1_target):.1f}° j2={math.degrees(j2_target):.1f}°'
        )

    # ──────────────────────────────────────────────────────────────────
    # Envio direto (sem action, sem fila)
    # A bridge no Nano usa fresh_mode=1: sempre executa o mais recente.
    # ──────────────────────────────────────────────────────────────────

    def _send_joints(self, j1_rad: float, j2_rad: float):
        positions = []
        for name in JOINT_NAMES:
            if name == 'cobot_joint_1':
                positions.append(j1_rad)
            elif name == 'cobot_joint_2':
                positions.append(j2_rad)
            else:
                positions.append(self.joint_positions.get(name, 0.0))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
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
