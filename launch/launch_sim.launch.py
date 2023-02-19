import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import RegisterEventHandler,TimerAction
from launch.event_handlers import OnProcessStart
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--verbose --ros-args --params-file ' + gazebo_params_file}.items()
             )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'AGV'],
                        output='screen')
    delayed_spawn_entity = TimerAction(period=3.0, actions=[spawn_entity])
    diff_drive_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["diff_cont"],
                        )
    delayed_diff_drive_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=spawn_entity,
        on_start=[diff_drive_spawner],
        )
    )
    joint_broad_spawner = Node(
                        package="controller_manager",
                        executable="spawner.py",
                        arguments=["joint_broad"],
                )       
    delayed_joint_broad_spawner = RegisterEventHandler(
    event_handler=OnProcessStart(
        target_action=spawn_entity,
        on_start=[joint_broad_spawner],
        )
    )
    robot_localization_params =os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(robot_localization_params), {'use_sim_time': True}]
)



    # Launch them all!
    return LaunchDescription([
        rsp,
        gazebo,
        twist_mux,
        delayed_spawn_entity,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        # robot_localization_node,
        
    ])
