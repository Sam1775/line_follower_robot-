import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('vacuum_bot')
    
    # 1. Process Xacro
    xacro_file = os.path.join(pkg_share, 'urdf', 'vacuum_bot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 2. World Path
    world_file = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    # 3. Launch Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 5. Spawn Robot Entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'vacuum_bot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    # 6. ROS 2 <-> Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/vacuum_bot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/vacuum_bot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/vacuum_bot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/vacuum_bot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/model/vacuum_bot/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/vacuum_bot/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 🎥 7. Auto-Launch Camera View (`rqt_image_view`)
    camera_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_view',
        arguments=['/model/vacuum_bot/image_raw'],
        output='screen'
    )

    # 🧠 8. Line Follower Node
    line_node = Node(
        package='vacuum_bot',
        executable='line_follower_node.py',
        name='line_follower_node',
        output='screen'
    )

    # 🔧 9. Force Gazebo to relink LiDAR to Robot Model
    force_lidar_init = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'model', '-m', 'vacuum_bot', '-l', 'sen_link'],
                output='screen',
                shell=True
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        camera_view,   # 👈 Opens camera window automatically!
        line_node,
        force_lidar_init
    ])