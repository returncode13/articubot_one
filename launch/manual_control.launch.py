import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node

import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file

PACKAGE_NAME="articubot_two"

launch_args=[
        DeclareLaunchArgument('use_sim_time',default_value='false',description='Use sim time if true'),
        ]

def launch_setup(context):
        
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param

    joy_params = os.path.join(get_package_share_directory('articubot_two'),'config','joystick.yaml')


    
    print("NAMESPACE: JOY: ",namespace)
    joy_node = Node(
            package='joy',
            namespace=namespace,
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            namespace=namespace,
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel','cmd_vel_joy')]
    )
    
    twist_mux_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('cmd_vel_out','cmd_vel')]
        )

    return [joy_node,teleop_node,twist_mux]



def generate_launch_description():

    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
