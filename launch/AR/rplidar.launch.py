import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch_ros.actions import Node

import xacro

#I need to get the namespace out when this launch file is called during runtime.
    #https://robotics.stackexchange.com/questions/104340/getting-the-value-of-launchargument-inside-python-launch-file


launch_args=[]
def launch_setup(context):
        
        print("NAMESPACE: RPLIDAR: ",namespace)
        namespace=LaunchConfiguration('namespace').perform(context)  #this gets the runtime value of the namespace param
        rplidar_node = Node(
            package='rplidar_ros',
            namespace=namespace,
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0',
                'frame_id': 'laser_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }]
        )
        return [rplidar_node]



def generate_launch_description():

    opfunc=OpaqueFunction(function=launch_setup)
    ld= LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
    
