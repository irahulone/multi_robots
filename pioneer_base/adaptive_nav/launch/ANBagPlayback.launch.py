import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

COLORS = {
    'p1': [1.0, 0.0, 0.0],  # 赤
    'p2': [0.0, 1.0, 0.0],  # 緑
    'p3': [0.0, 0.0, 1.0],  # 青
    'p4': [1.0, 1.0, 0.0],  # 黄
    'p5': [1.0, 0.0, 1.0],  # マゼンタ
}

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_description')
    xacro_file = os.path.join(pkg_share, 'src/description/pioneer_robot.xacro')
    custom_rviz_config = os.path.join(pkg_share, 'rviz/clusterp1-p3withhw.rviz')

    nodes = []

    # 各ロボット（p1, p2, p3）に対してTF発行ノードを作成
    robot_ids = ['p1', 'p2', 'p3']

    for robot_id in robot_ids:
        # 1. JointState Publisher - Pose2DをJointStateに変換
        nodes.append(
            Node(
                package='fake_rover_state_controller',
                executable='jointstate_publisher',
                name=f'{robot_id}_jointstate_publisher',
                output='screen',
                parameters=[{
                    'robot_id': robot_id
                }]
            )
        )

        # 2. Robot State Publisher (ハードウェア用) - JointStateとURDFからTFを発行
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=f"{robot_id}hw",
                output="screen",
                parameters=[{
                    "robot_description": Command([
                        "xacro ",
                        xacro_file,
                        f" r:={COLORS[robot_id][0]} g:={COLORS[robot_id][1]} b:={COLORS[robot_id][2]} a:=0.8"
                    ]),
                    "use_sim_time": False,
                    "frame_prefix": f"{robot_id}hw/",
                }]
            )
        )

        # 3. Static TF Publisher - world → {robot_id}hw/world
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}hw/world"],
            )
        )

        # 4. Robot State Publisher (Desired用) - 目標位置の可視化
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="state_publisher",
                namespace=f"{robot_id}desired",
                output="screen",
                parameters=[{
                    "robot_description": Command([
                        "xacro ",
                        xacro_file,
                        f" r:={COLORS[robot_id][0]} g:={COLORS[robot_id][1]} b:={COLORS[robot_id][2]} a:=0.3"
                    ]),
                    "use_sim_time": False,
                    "frame_prefix": f"{robot_id}desired/",
                }]
            )
        )

        # 5. Static TF Publisher - world → {robot_id}desired/world
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "world", f"{robot_id}desired/world"],
            )
        )

    # RViz2でビジュアライゼーション
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=['-d', custom_rviz_config],
            parameters=[{'use_sim_time': False}]
        )
    )

    return LaunchDescription(nodes)
