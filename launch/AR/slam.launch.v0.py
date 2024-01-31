import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml

import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file


launch_args=[
        DeclareLaunchArgument('use_sim_time',default_value='true',description='Use sim time if true'),
        # DeclareLaunchArgument('use_ros2_control',default_value='true',description='Use ros2_control if true'),
        DeclareLaunchArgument('namespace',default_value='/',description='Use ros2_control if true'),
        DeclareLaunchArgument('params_file',default_value='/app/ros2_ws/src/articubot_two/config/mapper_params_online_async.yaml',description='param file for slam')
        ]

def launch_setup(context):
        
        use_sim_time = LaunchConfiguration('use_sim_time')
        # use_ros2_control = LaunchConfiguration('use_ros2_control')
        
        namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param
        params_file=LaunchConfiguration('params_file').perform(context)

        namespaced_params_file = RewrittenYaml(
            source_file= params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True
            )
        
        print("SLAM PARAMS: ",namespace, params_file)
        slam_online_async = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'params_file':namespaced_params_file
                                    #    "namespace":namespace
                                       }.items()
    )
        return [slam_online_async]



def generate_launch_description():
    print("Starting SLAM")
    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
