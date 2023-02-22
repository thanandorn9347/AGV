import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    # joystick = IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource([os.path.join(
    #                 get_package_share_directory(package_name),'launch','joystick.launch.py'
    #             )])
    # )

    

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    imu_params_config = os.path.join(get_package_share_directory(package_name), 'config', 'imu_params.yaml')   
    imu_node = Node(
            package='imu_pkg',
            executable='imu_node',
            name='imu_node',
            parameters = [imu_params_config]
        )
    imu_tools_madgwick = Node(
        package="imu_fusion_madgwick",
        executable="imu_fusion_madgwick_node",
        parameters = [imu_params_config]
    )
    imu_tools_tf = Node(
        package="imu_tf",
        executable="transform",
        parameters=[{
          'source_frame': 'imu_frame',
          'target_frame': 'chassis',
    }])
    range_sensor = Node(
        package="vl53l0x",
        executable="vl53l0x",
        parameters=[]
    )
    robot_localization_params =os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(robot_localization_params), {'use_sim_time': False}]
)
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )
   
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
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
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )


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



    # Launch them all!
    return LaunchDescription([
        rsp,
        twist_mux,
        imu_node,
        imu_tools_madgwick,
        # imu_tools_tf,
        range_sensor,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        robot_localization_node,
    ])
