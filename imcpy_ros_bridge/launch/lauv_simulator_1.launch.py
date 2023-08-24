from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

def generate_launch_description():
   target_name = DeclareLaunchArgument(
      'target_name', default_value=TextSubstitution(text='lauv-simulator-1')
   )


   return LaunchDescription([
      target_name,
      Node(
         package='imcpy_ros_bridge',
         executable='imc2ros',
         namespace='lauv_simulator_1',
         name='imc2ros',
         parameters=[{
            'target_name': LaunchConfiguration('target_name')
         }]
      ),
   ])
