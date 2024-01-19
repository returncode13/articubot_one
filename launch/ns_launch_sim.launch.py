import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    namespace="ns_eigen2"
    package_name='articubot_two' #<--- CHANGE ME
    ld=LaunchDescription()
    actions=[]
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.old.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    ld.add_action(rsp)

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.old.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    ld.add_action(joystick)

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    ld.add_action(twist_mux)

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["-c","/"+namespace+"/controller_manager","--namespace",namespace,"diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["-c","/"+namespace+"/controller_manager","--namespace",namespace,"joint_broad"],
    )

    ld.add_action(gazebo)
    ld.add_action(spawn_entity)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    # Code for delaying a node (I haven't tested how effective it is)
    # 
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    ld.add_action(PushRosNamespace("ns_eigen2"))

    # Launch them all!
    # return LaunchDescription([
    #     rsp,
    #     joystick,
    #     twist_mux,
    #     gazebo,
    #     spawn_entity,
    #     diff_drive_spawner,
    #     joint_broad_spawner
    # ])
    actions=[
        PushRosNamespace('ns_eigen2'),
        rsp,
        joystick,
        twist_mux,
        
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner
    ]
    
    ld_ns=GroupAction(
        actions=actions
    )

    lld=LaunchDescription()
    lld.add_action(gazebo)
    lld.add_action(ld_ns)
    return lld
