import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch.actions import GroupAction,TimerAction
from launch_ros.actions import PushRosNamespace



PACKAGE_NAME="articubot_two"


x_pos={'del_x':'0.0'}  # a parameter used to spawn robots in different positions along x

def generate_global_launches(package_name=PACKAGE_NAME): 
    
    ld=LaunchDescription()
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    ld.add_action(gazebo)
    return ld
    



def generate_launch_description_for_single(package_name=PACKAGE_NAME,namespace="/"):


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    namespace=namespace
    package_name=package_name #<--- CHANGE ME
    ld=LaunchDescription()
    actions=[]
    print("NS_LAUNCH_FILE ",namespace)
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true',"namespace":namespace}.items()
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

    # gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #                 launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    #          )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', namespace,
                                   '-x', x_pos['del_x'], '-y', '0.0',
                                    '-z', '0.01', '-Y', '0.0',
                                   ],
                        output='screen')

    remappings=[(namespace+"/diff_cont/odom","odom")]
   

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",                 # loads, configures and start a controller on startup.
        arguments=["-c","/"+namespace+"/controller_manager","--namespace",namespace,"diff_cont"],
        # remappings=remappings
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",                 # loads, configures and start a controller on startup.
        arguments=["-c","/"+namespace+"/controller_manager","--namespace",namespace,"joint_broad"],
    )


    ##SLAM for this bot
    slam_params_file='/app/ros2_ws/src/articubot_two/config/mapper_params_online_async.yaml'
    slam_async=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','slam.launch.py'
                )]), launch_arguments={'use_sim_time': 'true',
                                       'namespace':namespace,
                                       'autostart': 'true',
                                       'use_respawn': 'true',
                                       'params_file':slam_params_file
                                       }.items()
    )
    
    nav2_params_file='/app/ros2_ws/src/articubot_two/config/nav2_params.yaml'
    nav2=IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','nav2.launch.py'
                )]), launch_arguments={
                    'use_sim_time': 'true',
                    'namespace ': namespace,
                    'autostart': 'true',
                    'params_file':nav2_params_file
                    }.items()
    )

    timed_slam=TimerAction(
        period=3.0,
        actions=[slam_async]
    )

    timed_nav2=TimerAction(
        period=6.0,
        actions=[nav2]
    )

    timerAction=TimerAction(
        period=3.0,
        actions=[]
    )
    # ld.add_action(gazebo)
    # ld.add_action(spawn_entity)
    # ld.add_action(diff_drive_spawner)
    # ld.add_action(joint_broad_spawner)

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

    # ld.add_action(PushRosNamespace("ns_eigen2"))

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
    ns_actions=[
        PushRosNamespace(namespace),
        rsp,
        joystick,
        twist_mux,
        
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        # timerAction,
        slam_async
        # nav2
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
    del_x=0.0

    for x in range(0,1):
        ns="ns_eigen_"+str(x)
        # ns=""
        ld.add_action((generate_launch_description_for_single(namespace=ns)))

        #create next robot 1.0 m ahead of the last one.
        del_x=del_x+1.0
        x_pos['del_x']=str(del_x)
    
    return ld