import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import GroupAction,TimerAction
from launch_ros.actions import PushRosNamespace



PACKAGE_NAME="articubot_two"


def generate_global_launches(package_name=PACKAGE_NAME): 
    
    ld=LaunchDescription()
    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    #          )
    
    # ld.add_action(gazebo)
    return ld
    



def generate_launch_description_for_single(package_name=PACKAGE_NAME,namespace="/"):


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    namespace=namespace
    package_name=package_name #<--- CHANGE ME
    ld=LaunchDescription()
    actions=[]
    print("NS_NAV2 LAUNCH_FILE ",namespace)
    
    
    nav2_params_file='/app/ros2_ws/src/articubot_two/config/nav2_iron_all.v1.yaml'
    nav2=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','nav2.launch.py'
                )]), launch_arguments={
                    'use_sim_time': 'true',
                    'namespace': namespace,
                    'use_namespace': 'True',
                    'autostart': 'true',
                    'params_file':nav2_params_file
                    }.items()
    )

    
   

    timed_nav2=TimerAction(
        period=6.0,
        actions=[nav2]
    )

  
    ns_actions=[
        # PushRosNamespace(namespace),
        
        nav2
        # timed_slam,
        # timed_nav2
    ]
    
    ld_ns=GroupAction(
        actions=ns_actions
    )

    lld=LaunchDescription()
    # lld.add_action(gazebo)
    lld.add_action(ld_ns)
    return lld


def generate_launch_description():
    
    ld=LaunchDescription()
    ld.add_action(generate_global_launches())

    for x in range(0,1):
        ns="ns_eigen_"+str(x)
        # ns=""
        ld.add_action((generate_launch_description_for_single(namespace=ns)))
    
    return ld