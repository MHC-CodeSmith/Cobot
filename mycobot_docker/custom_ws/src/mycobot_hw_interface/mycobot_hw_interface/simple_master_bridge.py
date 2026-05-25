#!/usr/bin/env python3
"""
simple_master_bridge.py — Bridge Mestre Unificada para MyCobot 280 JN
======================================================================
Roda no Jetson Nano. Conecta diretamente ao hardware via serial e
expõe a interface ROS 2 completa para o MoveIt no PC:

  - Publica /joint_states com nomes cobot_joint_1..6 (em radianos)
  - Serve uma Action FollowJointTrajectory para execução de trajetórias
  - Timestamp local do Nano (re-estampado pelo joint_state_relay no PC)

Requisitos: pymycobot, rclpy, control_msgs
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
import math
import time

# Suporte a versões diferentes do pymycobot
try:
    from pymycobot.mycobot280 import MyCobot280 as _HW
except ImportError:
    try:
        from pymycobot.mycobot import MyCobot as _HW
    except ImportError:
        _HW = None


# Nomes canônicos — devem ser idênticos ao XACRO e ao SRDF
JOINT_NAMES = [
    "cobot_joint_1",
    "cobot_joint_2",
    "cobot_joint_3",
    "cobot_joint_4",
    "cobot_joint_5",
    "cobot_joint_6",
]

# Velocidade padrão de envio ao hardware (0–100)
DEFAULT_SPEED = 80

# Tolerância de graus para considerar um ponto da trajetória como chegado
ARRIVE_THRESHOLD_DEG = 2.0


class SimpleMasterBridge(Node):
    def __init__(self):
        super().__init__('simple_master_bridge')

        # --- Parâmetros ---
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('mock', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('speed', DEFAULT_SPEED)

        port      = self.get_parameter('port').get_parameter_value().string_value
        baud      = self.get_parameter('baud').get_parameter_value().integer_value
        mock      = self.get_parameter('mock').get_parameter_value().bool_value
        rate_hz   = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().integer_value

        self.mock = mock
        self.mc   = None

        if mock:
            self.get_logger().warn('=== MODO MOCK ATIVO — sem conexão com hardware ===')
            self._mock_angles = [0.0] * 6
        else:
            if _HW is None:
                self.get_logger().fatal('pymycobot não está instalado! Instale com: pip3 install pymycobot')
                raise SystemExit(1)
            self.get_logger().info(f'Conectando ao MyCobot em {port} @ {baud} baud...')
            try:
                self.mc = _HW(port, baud)
                time.sleep(1.5)      # aguarda o hardware acordar
                self.get_logger().info('Conexão com MyCobot estabelecida!')
            except Exception as e:
                self.get_logger().fatal(f'ERRO CRÍTICO de conexão serial: {e}')
                raise SystemExit(1)

        # --- Publisher de estado das juntas ---
        # Publica em joint_states_raw; o joint_state_relay no PC re-estampa
        # com o clock local e republica em joint_states para o MoveIt.
        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        self.create_timer(1.0 / rate_hz, self._publish_joint_states)

        # --- Action Server para execução de trajetórias ---
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            f'SimpleMasterBridge pronto | mock={mock} | juntas={JOINT_NAMES}'
        )

    # ------------------------------------------------------------------ #
    #  Publicação de estado
    # ------------------------------------------------------------------ #

    def _publish_joint_states(self):
        angles_deg = self._read_angles()
        if angles_deg is None:
            return  # hardware não respondeu, não publica lixo

        msg = JointState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mycobot_base_link'
        msg.name            = JOINT_NAMES
        msg.position        = [math.radians(a) for a in angles_deg]
        msg.velocity        = [0.0] * 6
        msg.effort          = [0.0] * 6
        self.joint_pub.publish(msg)

    def _read_angles(self):
        """Retorna lista de 6 floats (graus) ou None se falhar."""
        if self.mock:
            return list(self._mock_angles)
        try:
            angles = self.mc.get_angles()
            if angles and len(angles) == 6:
                return [float(a) for a in angles]
        except Exception as e:
            self.get_logger().warn(f'Falha ao ler ângulos: {e}', throttle_duration_sec=5.0)
        return None

    # ------------------------------------------------------------------ #
    #  Action Server — callbacks de ciclo de vida
    # ------------------------------------------------------------------ #

    def _goal_callback(self, goal_request):
        self.get_logger().info('Nova trajetória recebida do MoveIt.')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancelamento solicitado.')
        return CancelResponse.ACCEPT

    # ------------------------------------------------------------------ #
    #  Execução de trajetória
    # ------------------------------------------------------------------ #

    async def _execute_trajectory(self, goal_handle):
        self.get_logger().info('Executando trajetória...')
        trajectory = goal_handle.request.trajectory

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = JOINT_NAMES

        # Reordena pontos para casar com JOINT_NAMES (MoveIt pode reordenar)
        incoming_names  = list(trajectory.joint_names)
        reorder_indices = self._build_reorder_map(incoming_names)

        for i, point in enumerate(trajectory.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Trajetória cancelada.')
                return FollowJointTrajectory.Result()

            # Reordena posições para a ordem canônica
            positions_rad = self._reorder(list(point.positions), reorder_indices)
            angles_deg    = [math.degrees(p) for p in positions_rad]

            self.get_logger().debug(
                f'Ponto {i+1}/{len(trajectory.points)}: {[f"{a:.1f}" for a in angles_deg]}°'
            )

            if self.mock:
                self._mock_angles = angles_deg
            else:
                try:
                    self.mc.send_angles(angles_deg, self.speed)
                except Exception as e:
                    self.get_logger().error(f'Erro ao enviar ângulos: {e}')
                    goal_handle.abort()
                    return FollowJointTrajectory.Result()

            # Feedback para o MoveIt
            feedback.actual.positions = positions_rad
            goal_handle.publish_feedback(feedback)

            # Aguarda o tempo previsto do ponto (sem bloquear o executor)
            if i < len(trajectory.points) - 1:
                next_point   = trajectory.points[i + 1]
                current_time = point.time_from_start
                next_time    = next_point.time_from_start
                dt_sec = (
                    (next_time.sec - current_time.sec)
                    + (next_time.nanosec - current_time.nanosec) * 1e-9
                )
                if dt_sec > 0:
                    await self._async_sleep(dt_sec)

        # Aguarda o robô chegar na posição final
        if not self.mock:
            await self._wait_for_arrival(angles_deg)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        self.get_logger().info('Trajetória concluída com sucesso.')
        return result

    # ------------------------------------------------------------------ #
    #  Utilitários internos
    # ------------------------------------------------------------------ #

    def _build_reorder_map(self, incoming_names):
        """Mapeia índice de incoming_names → índice em JOINT_NAMES."""
        indices = []
        for name in JOINT_NAMES:
            try:
                indices.append(incoming_names.index(name))
            except ValueError:
                self.get_logger().warn(f'Junta "{name}" ausente na trajetória!')
                indices.append(None)
        return indices

    def _reorder(self, values, indices):
        result = []
        for idx in indices:
            result.append(values[idx] if idx is not None else 0.0)
        return result

    async def _async_sleep(self, seconds):
        """Sleep não-bloqueante compatível com o executor rclpy."""
        import asyncio
        await asyncio.sleep(seconds)

    async def _wait_for_arrival(self, target_deg, timeout=5.0):
        """Aguarda até o robô atingir o alvo ou esgotar o timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            current = self._read_angles()
            if current is not None:
                errors = [abs(current[i] - target_deg[i]) for i in range(6)]
                if max(errors) < ARRIVE_THRESHOLD_DEG:
                    return
            await self._async_sleep(0.05)
        self.get_logger().warn('Timeout aguardando chegada — continuando mesmo assim.')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMasterBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
