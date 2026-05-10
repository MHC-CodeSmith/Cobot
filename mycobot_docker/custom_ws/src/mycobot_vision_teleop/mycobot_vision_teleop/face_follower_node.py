#!/usr/bin/env python3
"""
face_follower_node.py
=====================
Controlador proporcional: lê a posição do rosto e move joint1/joint2 do
myCobot para que o braço "aponte" na direção do rosto — como uma cobra
seguindo a presa.

Modo fluido (10 Hz + cancelamento de goal):
  A cada ciclo, cancela o goal anterior e envia um novo, com
  time_from_start curto (120 ms). Isso cria movimento contínuo
  sem esperar o goal anterior terminar.

Ativação:
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: false"

Parâmetros ROS:
  kp_x          (float, default 0.5)   ganho horizontal → joint1 [rad/unit]
  kp_y          (float, default 0.3)   ganho vertical   → joint2 [rad/unit]
  deadband      (float, default 0.06)  zona morta (6% da imagem)
  max_delta_rad (float, default 0.12)  movimento máximo por step [rad]
  rate_hz       (float, default 20.0)  frequência do controlador
  traj_ms       (int,   default 300)   duração de cada trajectory [ms] — deve ser > 1/rate_hz
  j2_offset     (float, default 0.4)   inclinação base do joint2 [rad]
  invert_x      (bool,  default False) inverte sentido horizontal

Modo streaming: trajetórias de 300 ms enviadas a 20 Hz (sobrepostas).
O servidor FollowJointTrajectory faz preempt automático do goal anterior,
criando movimento contínuo sem stop-start.
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


class FaceFollowerNode(Node):
    def __init__(self):
        super().__init__('face_follower')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('kp_x',          0.5)
        self.declare_parameter('kp_y',          0.3)
        self.declare_parameter('deadband',       0.06)
        self.declare_parameter('max_delta_rad',  0.12)
        self.declare_parameter('rate_hz',        20.0)
        self.declare_parameter('traj_ms',        300)
        self.declare_parameter('j2_offset',      0.4)
        self.declare_parameter('invert_x',       False)

        rate = self.get_parameter('rate_hz').value

        # ── Estado interno ────────────────────────────────────────────
        self.face_center = None
        self.tracking_ok = False
        self.joint_positions = {}
        self.enabled = False
        self._last_target = (0.0, 0.0)
        self._pending = False   # evita flood se action server estiver lento

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(Point,      '/human/face_center',     self._face_cb,     10)
        self.create_subscription(Bool,       '/human/tracking_ok',     self._tracking_cb, 10)
        self.create_subscription(JointState, '/joint_states',          self._js_cb,       10)
        self.create_subscription(Bool,       '/face_follower/enabled', self._enable_cb,   10)

        # ── Action client ─────────────────────────────────────────────
        self._ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/mycobot_arm_controller/follow_joint_trajectory',
        )

        # ── Status ────────────────────────────────────────────────────
        self._pub_status = self.create_publisher(Bool, '/face_follower/active', 10)

        # ── Timer de controle ─────────────────────────────────────────
        self.create_timer(1.0 / rate, self._control_loop)

        self.get_logger().info(
            f'FaceFollower pronto — {rate:.0f} Hz streaming | '
            f'traj={self.get_parameter("traj_ms").value} ms | '
            f'deadband={self.get_parameter("deadband").value:.0%}\n'
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
            self.get_logger().warn('Tracking perdido', throttle_duration_sec=2.0)

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
    # Loop de controle (10 Hz)
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

        delta_j1 = clamp(-kp_x * ex, -max_d, max_d) if abs(ex) > deadband else 0.0
        delta_j2 = clamp( kp_y * ey, -max_d, max_d) if abs(ey) > deadband else 0.0

        if abs(delta_j1) < 0.008 and abs(delta_j2) < 0.008:
            return

        j1_now = self.joint_positions.get('cobot_joint_1', 0.0)
        j2_now = self.joint_positions.get('cobot_joint_2', j2_off)

        j1_target = clamp(j1_now + delta_j1, *JOINT_LIMITS['cobot_joint_1'])
        j2_target = clamp(j2_now + delta_j2, *JOINT_LIMITS['cobot_joint_2'])

        if (abs(j1_target - self._last_target[0]) < 0.01 and
                abs(j2_target - self._last_target[1]) < 0.01):
            return

        if self._pending:
            return  # já tem um goal no ar, aguarda o ack

        self._last_target = (j1_target, j2_target)
        self._send_joints(j1_target, j2_target)

    # ──────────────────────────────────────────────────────────────────
    # Envio da trajetória — modo streaming (sem cancelamento explícito)
    # O servidor FollowJointTrajectory faz preempt automático.
    # Trajetória de 300 ms enviada a 20 Hz → robot "persegue" o target
    # de forma contínua sem stop-start.
    # ──────────────────────────────────────────────────────────────────

    def _send_joints(self, j1_rad: float, j2_rad: float):
        if not self._ac.server_is_ready():
            self.get_logger().warn('Bridge não disponível', throttle_duration_sec=5.0)
            return

        positions = []
        for name in JOINT_NAMES:
            if name == 'cobot_joint_1':
                positions.append(j1_rad)
            elif name == 'cobot_joint_2':
                positions.append(j2_rad)
            else:
                positions.append(self.joint_positions.get(name, 0.0))

        traj_ms = self.get_parameter('traj_ms').value

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = Duration(sec=0, nanosec=traj_ms * 1_000_000)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._pending = True
        future = self._ac.send_goal_async(goal)
        future.add_done_callback(self._goal_sent_cb)

        self.get_logger().debug(
            f'j1={math.degrees(j1_rad):.1f}° j2={math.degrees(j2_rad):.1f}°'
        )

    def _goal_sent_cb(self, future):
        # Goal aceito/rejeitado pelo bridge — libera para próximo envio
        self._pending = False

    def _result_cb(self, future):
        pass  # não usado no modo streaming


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
