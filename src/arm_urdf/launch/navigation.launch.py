from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    # 1. Start map_server immediately
    start_map_server = ExecuteProcess(
        cmd=[
            "ros2", "run", "nav2_map_server", "map_server",
            "--ros-args",
            "-p", "yaml_filename:=/home/jerome/project_karma/src/arm_urdf/config/map.yaml",
            "-p", "use_sim_time:=true",
            "-r", "/map:=/map2"
        ],
        output="screen"
    )

    # 2. Configure map_server after 2 seconds
    configure_map_server = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "configure"],
                output="screen"
            )
        ]
    )

    # 3. Activate map_server after 4 seconds
    activate_map_server = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "lifecycle", "set", "/map_server", "activate"],
                output="screen"
            )
        ]
    )

    # 4. Start Nav2 bringup after 6 seconds
    start_nav2 = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "launch", "nav2_bringup", "navigation_launch.py",
                    "map:=/home/jerome/project_karma/src/arm_urdf/config/map.yaml",
                    "params_file:=/home/jerome/project_karma/src/arm_urdf/config/nav2.yaml",
                    "use_sim_time:=true",
                    "autostart:=true"
                ],
                output="screen"
            )
        ]
    )

    # 5. Start SLAM after Nav2 → add 8 seconds
    start_slam = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "launch", "arm_urdf", "slam.launch.py"
                ],
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        start_map_server,
        configure_map_server,
        activate_map_server,
        start_slam,
        start_nav2
    ])
