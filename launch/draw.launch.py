import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur_moveit_pkg = get_package_share_directory('ur_moveit_config')
    
    # Lấy đường dẫn tuyệt đối tới file config RViz của bạn
    my_rviz_config = PathJoinSubstitution(
        [FindPackageShare('ur_draw_letter'), 'rviz', 've_chu.rviz']
    )

    # 1. Khởi chạy UR3e Fake Hardware + Ép RViz gốc đọc file của bạn!
    ur_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_moveit_pkg, 'launch', 'ur_moveit.launch.py')
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'use_fake_hardware': 'true',
            'launch_rviz': 'true',               # BẬT LẠI RViz gốc
            'rviz_config_file': my_rviz_config   # TRUYỀN FILE config của bạn vào đây
        }.items()
    )

    # Cầu nối tọa độ tĩnh
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    # 2. Cấu hình mô hình cho Node C++
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_description'), 'urdf', 'ur.urdf.xacro']), ' ',
        'name:=ur', ' ', 'ur_type:=ur3e'
    ])
    robot_description = {'robot_description': robot_description_content}

    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        PathJoinSubstitution([FindPackageShare('ur_moveit_config'), 'srdf', 'ur.srdf.xacro']), ' ',
        'name:=ur', ' ', 'ur_type:=ur3e'
    ])
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    robot_description_kinematics = {
        'robot_description_kinematics': {
            'ur_manipulator': {
                'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                'kinematics_solver_search_resolution': 0.005,
                'kinematics_solver_timeout': 0.05,
            }
        }
    }

    # 3. Node C++ vẽ chữ L
    student_node = Node(
        package='ur_draw_letter',
        executable='draw_letter_node',
        name='draw_letter_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': False}
        ]
    )

    return LaunchDescription([
        static_tf_node,
        ur_sim_launch, # Gọi hệ thống gốc chạy
        TimerAction(period=8.0, actions=[student_node])
    ])