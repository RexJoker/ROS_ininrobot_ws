from socket import timeout
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class uart_com():
    def __init__(self):
        # UART Configuration: 115200 8n1
        self.port = '/dev/ttyAMA1'
        self.timeout = 0.5
        self.baudrate = 115200
        self.parity = serial.PARITY_NONE
        self.stop_bits = 1
        self.uart = serial.Serial(self.port,self.baudrate,timeout=self.timeout)
        self.uart.parity = self.parity
        self.uart.stopbits = self.stop_bits
    def recieve(self):
        # gather full line of data through UART
        response = self.uart.readline()
        str_response = response.decode("utf-8")
        print(response)
        # remove useless characters: (such as \n or \r) and check if message recieved.
        if str_response.find('\n') != -1:
            self.get_logger().info(str_response)
            return self.convert_data(str_response)
        return "NULL"
    def convert_data(self, input_text):
        # search for \r:
        str1 = "\r"
        if str1 in input_text:
            input_text = input_text.replace(str1,'')
         # search for \n:
        str2 = "\n"
        if str2 in input_text:
            input_text = input_text.replace(str2,'')
        return input_text
    def send(self, arg_msg):
        #try to convert msg to string
        try:
            str_msg = str(arg_msg)
            str_msg = str_msg + '\n'
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
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscriber
    # typical callback function for catching data from topic and sending them directly via UART
    def topic_callback(self, msg):
        self.uart_obj.send(msg.data)
    # template-temporary feedback callback function which will give data for feedback loop - actions
    def feedback_callback(self):
        self.publisher.publish("rx_feedback")
    def timer_callback(self):
        msg = String()
        msg.data = self.uart_obj.recieve()
        if msg.data == "NULL":
            return
        self.publisher.publish(msg)
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
