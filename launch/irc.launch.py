import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    pkg_sim = get_package_share_directory('simulation_irc')
    world_path = os.path.join(pkg_sim, 'worlds', 'irc.world')
    
    # 'models' ディレクトリへの絶対パス文字列を作成
    model_path = os.path.join(pkg_sim, 'models')

    # 既存の GAZEBO_MODEL_PATH を取得
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')

    # Gazebo に 'models' ディレクトリの場所を教える環境変数を設定
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        # 'model_path' を既存のパスの先頭に追加
        value=f'{model_path}:{gazebo_model_path}'
    )

    # 'gazebo.launch.py' (gzserver と gzclient の両方) を起動する設定
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 
                'launch',
                'gazebo.launch.py'  # ⇐ 'gazebo.launch.py' を使用
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'gui': 'true'  # ⇐ GUIを有効
        }.items(),
    )

    return LaunchDescription([
        set_gazebo_model_path,
        gazebo
    ])


