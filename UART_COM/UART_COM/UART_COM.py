from socket import timeout
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class uart_com():
    def __init__(self):
        self.port = '/dev/serial1'
        self.timeout = 0.1
        # TODO: #1 baudrate = 115200 8n2
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
            str_msg = str_msg + '\r\n'
        except:
            # break sending when data cant be converted
            return
        # send converted data
        self.uart.write(str_msg.encode())
    def __del__(self):
        # added closing serial pipe when deleting object
        self.uart.close()

class uart_ros(Node):
    def __init__(self):
        super().__init__('UART_Node')
        #initialize subscription and publishing for UART_Node
        self.subscriber = self.create_subscription(String, 'UARTTX_topic', self.topic_callback, 10)
        self.publisher = self.create_publisher(String, 'UARTRX_topic', 10)
        self.uart_obj = uart_com()
        self.subscriber
    # typical callback function for catching data from topic and sending them directly via UART
    def topic_callback(self, msg):
        self.uart_obj.send(msg.data)
    # template-temporary feedback callback function which will give data for feedback loop - actions
    def feedback_callback(self):
        self.publisher.publish("rx_feedback")
    def __del__(self):
        del self.uart_obj

def main(arg=None):
    rclpy.init(args=arg)
    ros_node = uart_ros()
    rclpy.spin(ros_node)
    #when finished destroy node and delete objects:
    ros_node.destroy_node()
    del ros_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()