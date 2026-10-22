import math

import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from rclpy.time import Time
from cinematica_interfaces.msg import WheelsVelocities


# MINHAS CONVENÇÕES
# x+ para frente
# y+ para a esquerda
# rotação positiva: anti-horário, vista de cima


class DiferencialKinematics(Node):

    # VERIFICAR SE OS VALORES SÃO VÁLIDOS!
    def verify_valid_values(
        self,
        WHEEL_RADIUS,
        WHEEL_SEPARATION,
        TIMEOUT,
        PUBLISH_PER_SECOND
    ):

        if WHEEL_RADIUS <= 0 or not math.isfinite(WHEEL_RADIUS):
            self.get_logger().warn(
                'Raio deve ser finito e maior que zero!'
            )
            rclpy.shutdown()
            return

        if WHEEL_SEPARATION <= 0 or not math.isfinite(WHEEL_SEPARATION):
            self.get_logger().warn(
                'Distância entre as rodas deve ser finita e maior que zero!'
            )
            rclpy.shutdown()
            return

        if TIMEOUT <= 0 or not math.isfinite(TIMEOUT):
            self.get_logger().warn(
                'Timeout deve ser finito e maior que zero!'
            )
            rclpy.shutdown()
            return

        if PUBLISH_PER_SECOND <= 0 or not math.isfinite(PUBLISH_PER_SECOND):
            self.get_logger().warn(
                'Frequência de publicação deve ser finita e maior que zero!'
            )
            rclpy.shutdown()
            return


    def __init__(self):
        super().__init__("cinematica_diferencial_node")

        # PARÂMETROS: os valores vêm do YAML.
        self.declare_parameter('wheel_radius', Parameter.Type.DOUBLE)
        self.declare_parameter('wheel_separation', Parameter.Type.DOUBLE)
        self.declare_parameter('timeout', Parameter.Type.DOUBLE)
        self.declare_parameter('publish_per_second', Parameter.Type.DOUBLE)

        # VALORES IMPORTADOS DOS PARÂMETROS
        WHEEL_RADIUS = self.get_parameter('wheel_radius').value
        WHEEL_SEPARATION = self.get_parameter('wheel_separation').value
        TIMEOUT = self.get_parameter('timeout').value
        PUBLISH_PER_SECOND = self.get_parameter('publish_per_second').value

        self.verify_valid_values(
            WHEEL_RADIUS,
            WHEEL_SEPARATION,
            TIMEOUT,
            PUBLISH_PER_SECOND
        )
        
        # VARIÁVEIS
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.command_stamp = None
        self.is_command_recepted = False

        self.angular_velocity_wheel_left = 0.0
        self.angular_volocity_wheel_right = 0.0

        # PUBLISHERS
        self.wheel_publisher = self.create_publisher(
            WheelsVelocities,
            'wheel_velocities',
            10
        )

        # SUBSCRIBERS
        self.command_subscription = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.command_callback,
            10
        )

        # TIMER: o primeiro argumento é o período em segundos.
        self.timer = self.create_timer(
            1.0 / PUBLISH_PER_SECOND,
            self.timer_callback
        )

        self.get_logger().info(
            "Nó da cinemática diferencial iniciado!"
        )


    # CALLBACKS
    def command_callback(self, msg: TwistStamped):

        # Verificar se as velocidades são válidas.
        if not math.isfinite(msg.twist.linear.x):
            self.is_command_recepted = False
            return

        if not math.isfinite(msg.twist.angular.z):
            self.is_command_recepted = False
            return

        # Rejeitar stamp zerado.
        if (
            msg.header.stamp.sec == 0
            and msg.header.stamp.nanosec == 0
        ):
            self.is_command_recepted = False
            return

        if (
            msg.header.stamp.sec < 0
            or msg.header.stamp.nanosec >= 1000000000
        ):
            self.is_command_recepted = False
            return

        # Guardar as velocidades depois da validação.
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z

        # Guardar o instante de emissão do comando.
        self.command_stamp = Time.from_msg(
            msg.header.stamp,
            clock_type=self.get_clock().clock_type
        )

        self.is_command_recepted = True


    def timer_callback(self):

        WHEEL_RADIUS = self.get_parameter('wheel_radius').value
        WHEEL_SEPARATION = self.get_parameter('wheel_separation').value
        TIMEOUT = self.get_parameter('timeout').value
        PUBLISH_PER_SECOND = self.get_parameter('publish_per_second').value

        # Publicar zeros antes de encerrar se algum parâmetro ficar inválido.
        if (
            WHEEL_RADIUS <= 0
            or not math.isfinite(WHEEL_RADIUS)
            or WHEEL_SEPARATION <= 0
            or not math.isfinite(WHEEL_SEPARATION)
            or TIMEOUT <= 0
            or not math.isfinite(TIMEOUT)
            or PUBLISH_PER_SECOND <= 0
            or not math.isfinite(PUBLISH_PER_SECOND)
        ):
            self.angular_velocity_wheel_left = 0.0
            self.angular_volocity_wheel_right = 0.0

            msg = WheelsVelocities()

            msg.front_left_velocity = 0.0
            msg.front_right_velocity = 0.0
            msg.rear_left_velocity = 0.0
            msg.rear_right_velocity = 0.0

            self.wheel_publisher.publish(msg)

            self.verify_valid_values(
                WHEEL_RADIUS,
                WHEEL_SEPARATION,
                TIMEOUT,
                PUBLISH_PER_SECOND
            )

            return

        # manter as velocidades igual a zero caso nada recebido
        self.angular_velocity_wheel_left = 0.0
        self.angular_volocity_wheel_right = 0.0

        if self.is_command_recepted:

            command_age = (
                self.get_clock().now() - self.command_stamp
            ).nanoseconds / 1e9

            if command_age < 0.0 or command_age >= TIMEOUT:
                self.is_command_recepted = False

            else:
                self.angular_velocity_wheel_left = (
                    self.linear_velocity
                    - self.angular_velocity * WHEEL_SEPARATION / 2.0
                ) / WHEEL_RADIUS

                self.angular_volocity_wheel_right = (
                    self.linear_velocity
                    + self.angular_velocity * WHEEL_SEPARATION / 2.0
                ) / WHEEL_RADIUS

        if (
            not math.isfinite(self.angular_velocity_wheel_left)
            or not math.isfinite(self.angular_volocity_wheel_right)
        ):
            self.angular_velocity_wheel_left = 0.0
            self.angular_volocity_wheel_right = 0.0
            self.is_command_recepted = False


        #PUBLICANDO AS MENSAGENS
        #msg = WheelsVelocities, pois o autocomplete não estava funcionando
        
        msg = WheelsVelocities()

        msg.front_left_velocity = self.angular_velocity_wheel_left
        msg.rear_left_velocity = self.angular_velocity_wheel_left

        msg.front_right_velocity = self.angular_volocity_wheel_right
        msg.rear_right_velocity = self.angular_volocity_wheel_right

        self.wheel_publisher.publish(msg)


def main():
    
    
    rclpy.init()
    node = None

    #Tratamento de Erros por causa dos Shutdown's presentes
    #na verificação dos parâmetros
    
    try:
        node = DiferencialKinematics()

        if rclpy.ok():
            rclpy.spin(node)

    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()