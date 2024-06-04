import os
from launch import LaunchDescription, launch_description_sources
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import launch
from colorama import Fore, Back, Style
from launch.substitutions import PythonExpression, EnvironmentVariable

from scipy.spatial.transform import Rotation as R

configurable_parameters = [
                           {'name': 'feature_names', 'default': '', 'description': 'The name of the feature'},
                           {'name': 'delay', 'default': '2.0', 'description': 'the delay before message is published'},
                           {'name': 'number_of_publish', 'default': '5', 'description': 'number of feature status list published before exiting'},
                          ]


def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def launch_nodes(context, *args, **kwargs):
    delay = eval(launch.substitutions.LaunchConfiguration('delay').perform(context))
    number_of_publish = eval(launch.substitutions.LaunchConfiguration('number_of_publish').perform(context))
    feature_names_str = launch.substitutions.LaunchConfiguration('feature_names').perform(context)

    feature_names_arr = [s.strip() for s in feature_names_str.split(',')]

    return [
            Node(
                package = 'am_utils',
                name = 'feature_status_list_publisher',
                executable = 'feature_status_list_publisher',
                parameters = [{
                    'feature_names': feature_names_arr,
                    'delay': delay,
                    'number_of_publish': number_of_publish,
                }],
                output='screen'
                ),

        ]
def generate_launch_description():

    ld = LaunchDescription()

    declared_args = declare_configurable_parameters(configurable_parameters)
    for arg in declared_args:
        ld.add_action(arg)
    
    of = OpaqueFunction(function=launch_nodes)
    ld.add_action(of)
    return ld


if __name__ == '__main__':
    generate_launch_description()
    
    
    
    
    