import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node

import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file

PACKAGE_DIR='articubot_two'

launch_args=[
        DeclareLaunchArgument('use_sim_time',default_value='false',description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control',default_value='true',description='Use ros2_control if true'),
        DeclareLaunchArgument('namespace',default_value='/',description='Use ros2_control if true'),
        ]

def launch_setup(context):
        
        use_sim_time = LaunchConfiguration('use_sim_time')
        use_ros2_control = LaunchConfiguration('use_ros2_control')
        
        namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param

        print("RSP: NS: ",namespace)
        # Process the URDF file
        pkg_path = os.path.join(get_package_share_directory('articubot_two'))
        xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
        # robot_description_config = xacro.process_file(xacro_file).toxml()
        robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time, ' robot_ns:=',namespace])
        remappings = [('/tf','tf'), ('/tf_static', 'tf_static')]

        urdf = os.path.join(get_package_share_directory(PACKAGE_DIR), 'description', 'turtlebot3_waffle.xacro')
        with open(urdf, 'r') as infp:
            robot_description = infp.read()

        print("RD: ",robot_description)
        start_robot_state_publisher_cmd = Node(
            # condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                        'robot_description': robot_description}],
            remappings=remappings)
        
        return [start_robot_state_publisher_cmd]



def generate_launch_description():

    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
