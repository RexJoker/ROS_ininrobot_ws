from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
    Node(
    package='uart_com',
    namespace='uart_com',
    executable='uart_node',
    name='uart_interface',
    remappings=[
    ('/uart_com/UARTTX_topic', '/UARTTX_topic'),
    ('/uart_com/UARTRX_topic', '/UARTRX_topic')
    ]
    ),
    Node(
    package='ipico_drvs',
    namespace='ipico_drvs',
    executable='ipico_node',
    name='pico_interface',
    remappings=[
    ('/ipico_drvs/UARTTX_topic', '/UARTTX_topic'),
    ('/ipico_drvs/UARTRX_topic', '/UARTRX_topic'),
    ('/ipico_drvs/cmd_vel', '/cmd_vel')
    ],
    output='screen'
    ),
])