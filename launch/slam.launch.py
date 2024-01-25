import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node,SetRemap
from launch.actions import IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml,ReplaceString

import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file


launch_args=[
        DeclareLaunchArgument('use_sim_time',default_value='true',description='Use sim time if true'),
        # DeclareLaunchArgument('use_ros2_control',default_value='true',description='Use ros2_control if true'),
        DeclareLaunchArgument('namespace',default_value='/',description='Use ros2_control if true'),
        DeclareLaunchArgument('params_file',default_value='/app/ros2_ws/src/articubot_two/config/mapper_params_online_async-1.yaml',description='param file for slam'),
        DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup the nav2 stack'),
        DeclareLaunchArgument('use_respawn', default_value='False',description='Whether to respawn if a node crashes. Applied when composition is disabled.')


        ]

def launch_setup(context):
        
        use_sim_time = LaunchConfiguration('use_sim_time')
        # use_ros2_control = LaunchConfiguration('use_ros2_control')
        autostart = LaunchConfiguration('autostart')
        use_respawn = LaunchConfiguration('use_respawn')

        namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param
        params_file=LaunchConfiguration('params_file').perform(context)


        param_substitutions = {
        'use_sim_time': use_sim_time,
        'odom_frame':namespace+"/odom",
        'map_frame' : namespace+"/map",
        'base_frame' : namespace+"/base_footprint",
        'scan_topic' : namespace+"/scan"
        }


        namespaced_params_file = RewrittenYaml(
            source_file= params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True
            )
        
        namespaced_params_file = ReplaceString(
             source_file=namespaced_params_file, 
             replacements={
                "<robot_namespace>":(namespace)      
             } 
        )
        
        lifecycle_nodes = ['map_saver']
        
        remappings=[
        ("/map", "map"),
        ("/map_metadata", "map_metadata"),
        ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
        ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ]

        print("SLAM PARAMS: ",namespace, params_file)



        start_map_saver_server_cmd = Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            # arguments=['--ros-args', '--log-level', log_level],
            parameters=[namespaced_params_file])

        start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            # arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

        slam_online_async = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("slam_toolbox"),'launch','online_async_launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time, 
                                       'slam_params_file':namespaced_params_file
                                    #    "namespace":namespace
                                       }.items()
                )

        slam_online_async_ga = GroupAction(
              actions=[
                        SetRemap(src="/map", dst="map"),
                        SetRemap(src="/map_metadata", dst="map_metadata"),
                        SetRemap(src="/slam_toolbox/scan_visualization", dst="slam_toolbox/scan_visualization"),
                        SetRemap(src="/slam_toolbox/graph_visualization", dst="slam_toolbox/graph_visualization"),
                        
                        
                        slam_online_async
                    
                       ]
        )

    
        return [
                # start_map_saver_server_cmd,
                # start_lifecycle_manager_cmd,
                slam_online_async_ga
                ]



def generate_launch_description():
    print("Starting SLAM")
    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
