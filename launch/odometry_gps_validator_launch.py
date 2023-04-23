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
from launch.substitutions import EnvironmentVariable

import subprocess
import math
from vtk.numpy_interface.algorithms import condition


configurable_parameters = [{'name': 'bag_file_input_path', 'default': '', 'description': 'path to the input bag file'},
                           {'name': 'bag_file_output_path', 'default': '', 'description': 'path to the output bag file'},
                           {'name': 'gps_topic', 'default': '/mavros/gpsstatus/gps1/raw', 'description': 'topic for the gps data'},
                           {'name': 'odom_topic', 'default': '/visual_slam/tracking/odometry', 'description': 'topic for the odometry data'},
                           {'name': 'gps_reference_latitude', 'default': '38.012567524', 'description': 'reference latitude'},
                           {'name': 'gps_reference_longitude', 'default': '-122.52803672', 'description': 'reference longitude'},
                          ]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]


def generate_launch_description():
    
    declared_args = declare_configurable_parameters(configurable_parameters)
    
    
    
        
        
    node = Node(
                package = 'am_utils',
                name = 'odometry_gps_validator',
                executable = 'odometry_gps_validator',
                parameters = [set_configurable_parameters(configurable_parameters),
                              {
                                'use_sim_time': True

                            }
                              ],
                output='screen'
            )
            
    ld = LaunchDescription()
    for arg in declared_args:
        ld.add_action(arg)
    ld.add_action(node)

    return ld

if __name__ == '__main__':    
    generate_launch_description()
    
    
    
    
    
