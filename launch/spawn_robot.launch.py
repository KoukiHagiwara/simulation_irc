import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # パッケージのパスを取得
    pkg_path = get_package_share_directory('simulation_irc')

    # URDFファイル(xacro)のパス
    xacro_file = os.path.join(pkg_path, 'urdf', 'simple_robot.urdf.xacro')
    # xacroをURDF(xml)に変換
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # ワールドファイルのパス
    world_file = os.path.join(pkg_path, 'worlds', 'empty_world.world')

    # Gazeboの起動
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'),
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Robot State Publisher の起動
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )

    # ロボットをスポーン（出現）させる
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_robot'],
        output='screen'
    )

    # ---- ros2_control ----
    
    # コントローラーの起動
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            os.path.join(pkg_path, 'config', 'diff_drive_controller.yaml')
        ],
        output="screen",
    )

    # 差動二輪コントローラーのロード
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # 関節状態ブロードキャスターのロード
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        controller_manager, # humble以降は controller_manager を先に起動
        diff_drive_controller_spawner,
        joint_state_broadcaster_spawner,
    ])
