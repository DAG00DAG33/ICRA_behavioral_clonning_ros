
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    #config = os.path.join(
        #get_package_share_directory('model_inference'),
        #'config',
        #'params.yaml'
        #)
    return LaunchDescription([
        Node(
            package='model_inference',
            executable='model_inference_node',
            #parameters=[config],
        )])
