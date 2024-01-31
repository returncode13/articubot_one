import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml,ReplaceString


import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file

PACKAGE_NAME="articubot_two"

launch_args=[
        DeclareLaunchArgument('use_sim_time',default_value='true',description='Use sim time if true'),
        DeclareLaunchArgument('use_namespace',default_value='true',description='Use sim time if true'),
        # DeclareLaunchArgument('use_ros2_control',default_value='true',description='Use ros2_control if true'),
        DeclareLaunchArgument('namespace',default_value='/',description='Use ros2_control if true'),
        DeclareLaunchArgument('params_file',default_value='/app/ros2_ws/src/articubot_two/config/nav2_params-2.yaml',description='param file for nav2'),
        DeclareLaunchArgument('autostart',default_value='false',description='param file for nav2'),
        # DeclareLaunchArgument('map',default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),description='Full path to map file to load')
        ]

def launch_setup(context):
        
        use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        # use_ros2_control = LaunchConfiguration('use_ros2_control')
        
        namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param
        params_file=LaunchConfiguration('params_file').perform(context)
        # namespaced_params_file = os.path.join(namespace, os.path.basename(params_file))
        autostart=LaunchConfiguration('autostart').perform(context)
        use_namespace=LaunchConfiguration('use_namespace')

        # namespaced_params_file = RewrittenYaml(
        #     source_file= params_file,
        #     root_key=namespace,
        #     param_rewrites={},
        #     convert_types=True
        #     )
        
        # namespaced_params_file = ReplaceString(
        #      source_file=namespaced_params_file, 
        #      replacements={
        #         "<robot_namespace>":(namespace)      
        #      } 
        # )



    #     print("NS_PARAMS: ns:",namespace," ns_params_file:", namespaced_params_file," autostart:",autostart,"use_sim_time: ",use_sim_time)

    #     nav2 = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(PACKAGE_NAME),'launch','navigation_launch.py'   #/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py
                   
    #             )]), launch_arguments={'use_sim_time': use_sim_time, 
    #                                    'params_file': namespaced_params_file,
    #                                    'autostart': autostart,
    #                                    "namespace": namespace
    #                                    }.items()
    # )
        
        

        bringup_cmd = IncludeLaunchDescription(
              PythonLaunchDescriptionSource(
                    os.path.join(
                                    get_package_share_directory(PACKAGE_NAME),'launch', 'ns_bringup.launch.py')),
                                    launch_arguments={  'namespace': namespace,
                                                        'use_namespace': 'True',
                                                        'slam': 'True',
                                                        # 'map': map_yaml_file,
                                                        'use_sim_time': use_sim_time,
                                                        'params_file': params_file,
                                                        'autostart': autostart,
                                                        'use_composition': 'False',
                                                        'use_respawn': 'True'}.items()
                                )


        # return [nav2]
        return [bringup_cmd]



def generate_launch_description():
    print("Starting NAV")
    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
