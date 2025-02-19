from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# import ros_gz_interfaces.srv._spawn_entity
# import ros_gz_interfaces.srv.SpawnEntity

def generate_launch_description():
    ld = LaunchDescription()

    # Package Directories
    this_pkg = get_package_share_directory('tracking_3')

    ros_gz_bridge_config = os.path.join(this_pkg, 'config', 'ros_gz_bridge.yaml')

    gz_sim = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": "world.sdf",
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_bridge_msg_img = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/rgb/image_raw",],
        output="screen",
    )

    gz_bridge_msg_topic = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
        }],
        output='screen'
      )      
    
    set_gazebo_model_path_env = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", "/home/nebalcon/ros2_project/cv_ros2/src/tracking_3/resource")

    # ld.add_action(service_bridge)
    ld.add_action(set_gazebo_model_path_env)
    ld.add_action(gz_sim)
    ld.add_action(gz_bridge_msg_img)
    ld.add_action(gz_bridge_msg_topic)

    return ld