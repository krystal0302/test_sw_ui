import os
import yaml
import sys
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_configuration():
    # ============================
    # GENERIC MASTER CONFIGURATION
    # ============================
    # Check the number of arguments
    if len(sys.argv) != 5:
        print('Wrong number of arguments. Configuration must be specified.')
        exit()
    # Check if the argument conf_file is specified
    conf_file = str(sys.argv[4]).split(':=')
    if conf_file[0] != 'conf_file':
        print('Configuration argument conf_file is not found.')
        exit()
    # Check that the conf.yaml file exists
    if conf_file[1] != './conf/conf.yaml':
        print('conf.yaml file is not found. A conf.yaml file needs to be included in far_amr_ws/conf/')
        exit()

    # ===================================
    # AUTO-GENERATE LAUNCH SPECIFIC FILES
    # ===================================
    print('Auto-generate configuration files')
    # navigation_path = get_package_share_directory('far_nav')
    # Delete auto-generated folder and create empty ones
    # try:
    #     shutil.rmtree(navigation_path+'/auto_generated')
    # except OSError:
    #     print('Auto-generated folder cannot be deleted')
    # try:
    #     os.mkdir(navigation_path+'/auto_generated', 0o755)
    # except OSError:
    #     print('Auto-generated folder cannot be created')

    # Auto-generate robot bringup configuration ARGS + parameters YAML
    conf = {}
    with open(conf_file[1], 'r') as master_yaml_stream:
        yaml_conf = yaml.safe_load(master_yaml_stream)
        for amr_name, amr_params in yaml_conf.items():
            # Configuration arguments (mainly used for Gazebo)
            conf = {'log_folder': str(amr_params['logging_folder'])}
            
    return conf


def generate_launch_description():
    # Auto-generate configuration
    conf = generate_launch_configuration()
    print(conf)

    return LaunchDescription([

        Node(
            package='logging_core',
            executable='logging_core',
            name='logging_core',
            output='screen',
            parameters=[
                {"logging_folder": conf['log_folder']},
            ]
         )

    ])