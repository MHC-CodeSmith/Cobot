#!/usr/bin/env python3
"""
face_follower_node.py
=====================
Lê a posição do rosto detectada pelo vision_node e move as duas
primeiras juntas do myCobot para que o braço "aponte" na direção do rosto.

Arquitetura:
  /human/face_center  (Point, x/y ∈ [0,1] coords normalizadas da câmera)
  /human/tracking_ok  (Bool)
  /joint_states       (JointState — estado atual do robô)
        │
        ▼
  FaceFollowerNode (controlador proporcional simples)
        │
        ▼
  /mycobot_arm_controller/follow_joint_trajectory   ← direto ao bridge do Nano
  (bypassa planejamento MoveIt — OK pois só mexe joint1/joint2)

Ativação:
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: false"

Parâmetros ROS:
  kp_x          (float, default 0.5)  ganho horizontal → joint1 [rad/unit]
  kp_y          (float, default 0.3)  ganho vertical   → joint2 [rad/unit]
  deadband      (float, default 0.07) zona morta (7% da largura/altura da imagem)
  max_delta_rad (float, default 0.25) movimento máximo por step [rad]
  rate_hz       (float, default 3.0)  frequência do controlador
  j2_offset     (float, default 0.4)  ângulo base do joint2 (resting look-up) [rad]
  invert_x      (bool,  default False) inverte sinal horizontal (depende da câmera)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# Nomes canônicos — devem ser idênticos ao SRDF e ao bridge
JOINT_NAMES = [
    'cobot_joint_1', 'cobot_joint_2', 'cobot_joint_3',
    'cobot_joint_4', 'cobot_joint_5', 'cobot_joint_6',
]

# Limites seguros para face tracking (conservadores)
JOINT_LIMITS = {
    'cobot_joint_1': (-2.0,  2.0),   # yaw base ±~115°
    'cobot_joint_2': (-1.5,  1.5),   # shoulder ±~85°
}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class FaceFollowerNode(Node):
    def __init__(self):
        super().__init__('face_follower')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('kp_x',          0.5)
        self.declare_parameter('kp_y',          0.3)
        self.declare_parameter('deadband',       0.07)
        self.declare_parameter('max_delta_rad',  0.25)
        self.declare_parameter('rate_hz',        3.0)
        self.declare_parameter('j2_offset',      0.4)   # joint2 "olha para cima" um pouco
        self.declare_parameter('invert_x',       False)

        rate = self.get_parameter('rate_hz').value

        # ── Estado interno ────────────────────────────────────────────
        self.face_center: Point | None = None
        self.tracking_ok: bool = False
        self.joint_positions: dict[str, float] = {}
        self.enabled: bool = False
        self._goal_active: bool = False
        self._last_target: tuple[float, float] = (0.0, 0.0)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Point,      '/human/face_center',     self._face_cb,     10)
        self.create_subscription(Bool,       '/human/tracking_ok',     self._tracking_cb, 10)
        self.create_subscription(JointState, '/joint_states',          self._js_cb,       10)
        self.create_subscription(Bool,       '/face_follower/enabled', self._enable_cb,   10)

        # ── Action client (direto ao bridge, sem MoveIt planning) ─────
        self._ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/mycobot_arm_controller/follow_joint_trajectory',
        )

        # ── Publisher de status ───────────────────────────────────────
        self._pub_status = self.create_publisher(Bool, '/face_follower/active', 10)

        # ── Timer de controle ─────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'FaceFollower pronto — deadband={self.get_parameter("deadband").value:.0%} '
            f'kp_x={self.get_parameter("kp_x").value} '
            f'kp_y={self.get_parameter("kp_y").value}\n'
            '  Habilitar:  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"\n'
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
            self.get_logger().warn('Tracking perdido — parando', throttle_duration_sec=2.0)

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
        if self.face_center is None:
            return
        if not self.joint_positions:
            return
        if self._goal_active:
            return  # aguarda goal anterior terminar

        # Parâmetros
        kp_x     = self.get_parameter('kp_x').value
        kp_y     = self.get_parameter('kp_y').value
        deadband = self.get_parameter('deadband').value
        max_d    = self.get_parameter('max_delta_rad').value
        j2_off   = self.get_parameter('j2_offset').value
        inv_x    = self.get_parameter('invert_x').value

        # Erro em relação ao centro da imagem (0.5, 0.5)
        ex = self.face_center.x - 0.5   # positivo = rosto à DIREITA da imagem
        ey = self.face_center.y - 0.5   # positivo = rosto ABAIXO do centro

        if inv_x:
            ex = -ex

        # Zona morta
        delta_j1 = 0.0
        delta_j2 = 0.0

        if abs(ex) > deadband:
            # Rosto à direita da imagem → rotaciona joint1 no sentido horário
            # O sinal correto depende da orientação da câmera:
            #   se câmera olha para o usuário de frente ao robô, ex>0 → joint1 negativo
            delta_j1 = clamp(-kp_x * ex, -max_d, max_d)

        if abs(ey) > deadband:
            # Rosto abaixo → joint2 positivo (inclina braço para cima)
            delta_j2 = clamp(kp_y * ey, -max_d, max_d)

        # Sem mudança significativa → não envia
        if abs(delta_j1) < 0.015 and abs(delta_j2) < 0.015:
            return

        # Posições atuais
        j1_now = self.joint_positions.get('cobot_joint_1', 0.0)
        j2_now = self.joint_positions.get('cobot_joint_2', j2_off)

        # Targets com clamp
        j1_target = clamp(j1_now + delta_j1, *JOINT_LIMITS['cobot_joint_1'])
        j2_target = clamp(j2_now + delta_j2, *JOINT_LIMITS['cobot_joint_2'])

        # Evita enviar se mudança muito pequena (oscilação)
        if (abs(j1_target - self._last_target[0]) < 0.02 and
                abs(j2_target - self._last_target[1]) < 0.02):
            return

        self._last_target = (j1_target, j2_target)
        self._send_joints(j1_target, j2_target)

    # ──────────────────────────────────────────────────────────────────
    # Envio da trajetória
    # ──────────────────────────────────────────────────────────────────

    def _send_joints(self, j1_rad: float, j2_rad: float):
        """Envia uma trajetória de 1 ponto para joint1 e joint2.
        Mantém joints 3-6 nos valores atuais.
        """
        if not self._ac.server_is_ready():
            self.get_logger().warn('Bridge não disponível', throttle_duration_sec=5.0)
            return

        # Monta posição completa (6 juntas)
        positions = []
        for name in JOINT_NAMES:
            if name == 'cobot_joint_1':
                positions.append(j1_rad)
            elif name == 'cobot_joint_2':
                positions.append(j2_rad)
            else:
                positions.append(self.joint_positions.get(name, 0.0))

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=int(0.35e9))  # 350ms para chegar
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._goal_active = True
        future = self._ac.send_goal_async(goal)
        future.add_done_callback(self._goal_sent_cb)

        self.get_logger().debug(
            f'joint1={math.degrees(j1_rad):.1f}° '
            f'joint2={math.degrees(j2_rad):.1f}°'
        )

    def _goal_sent_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejeitado pelo bridge')
            self._goal_active = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._goal_active = False


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
