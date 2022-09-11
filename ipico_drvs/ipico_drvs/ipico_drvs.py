import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist as BSmart #easter egg
from std_msgs.msg import String

class ipico_node(Node):
    def __init__(self):
        super().__init__('Ipico_node')
        #initialize subscriptions and publishing for Ipico_node
        self.vel_sub = self.create_subscription(BSmart, 'cmd_vel', self.cmd_vel_callback, 10)
        self.uart_sub = self.create_subscription(String, 'UARTRX_topic', 10)
        self.uart_pub = self.create_publisher(String, 'UARTTX_topic', 10)
        self.subscriber1
        self.subscriber2
    
    def cmd_vel_callback(self, msg):