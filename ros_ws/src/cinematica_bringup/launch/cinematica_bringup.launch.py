import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def select_node(context):

    kinematics = LaunchConfiguration('cinematica').perform(context)

    if kinematics == 'diferencial':
        package = 'cinematica_diferencial'
        executable = 'cinematica_diferencial_node'
        configuration = 'parametros_diferencial.yaml'

    elif kinematics == 'ackermann':
        package = 'cinematica_ackermann'
        executable = 'cinematica_ackermann_node'
        configuration = 'parametros_ackermann.yaml'

    else:
        raise RuntimeError("Escolha 'diferencial' ou 'ackermann' no argumento cinematica.")

    package_directory = get_package_share_directory(package)
    configuration_path = os.path.join(package_directory, 'config', configuration)

    # Apenas o nó escolhido é criado.
    node = Node(package=package, executable=executable, name=executable, parameters=[configuration_path], output='screen')

    return [node]


def generate_launch_description():

    argument = DeclareLaunchArgument('cinematica', default_value='diferencial', description='Cinemática utilizada: diferencial ou ackermann.')

    selection = OpaqueFunction(function=select_node)

    return LaunchDescription([argument, selection])