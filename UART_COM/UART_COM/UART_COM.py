from socket import timeout
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UART_COM():
    def __init__(self):
        self.port = '/dev/ttyTHS1'
        self.timeout = 0.1
        self.uart = serial.Serial(self.port,timeout=self.timeout)
    def recieve(self):
        # gather full line of data through UART
        response = self.uart.readline()
        # split response from ending characters \r\n
        single_msg = response.split('\r\n')
        # check if data is correct by finding ';'
        if(single_msg[0].find(';' != -1)):
            return single_msg[0] # return recieved data
        else:
            return "F" # return that something goes wrong
    def send(self, arg_msg):
        #try to convert msg to string
        try:
            str_msg = str(arg_msg)
        except:
            # break sending when data cant be converted
            return
        # send converted data
        self.uart.write(str_msg)
    def __del__(self):
        # added closing serial pipe when deleting object
        self.uart.close()

class UART_ROS(Node):
    def __init__(self):
        super().__init__('UART_Node')
        self.subscribe = self.create_subscription(String, 'UART_topic', self.topic_callback, 10)
        self.subscribe
        self.uart_obj = UART_COM()
    def topic_callback(self, msg):
        self.uart_obj.send(msg.data)
    def __del__(self):
        del self.uart_obj

def main(arg=None):
    rclpy.init(args=arg)
    ros_node = UART_ROS()
    rclpy.spin(ros_node)
    
    ros_node.destroy_node()
    del ros_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()
