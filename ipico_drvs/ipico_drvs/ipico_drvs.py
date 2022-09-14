from enum import Enum
from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist as BSmart #easter egg
from std_msgs.msg import String

class ipico_node(Node):
    def __init__(self):
        super().__init__('ipico_node')
        #initialize subscriptions and publishing for Ipico_node
        self.vel_sub = self.create_subscription(BSmart, 'cmd_vel', self.cmd_vel_callback, 10)
        self.uart_sub = self.create_subscription(String, 'UARTRX_topic', self.uart_callback, 10)
        self.uart_pub = self.create_publisher(String, 'UARTTX_topic', 10)
        self.update = False
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
            }
        self.rx_update = False
        self.vel_update = True
        self.feedback = "None"
        self.vel_sub
        self.uart_sub
        self.ipico_drvs = ipico_drvs(self)
        self.teleop = teleop_twist()

    def uart_callback(self, msg):
        self.feedback = msg.data
        self.rx_update = True

    def cmd_vel_callback(self, msg):
        # rewriting data from geometry_msg to class variable
        # linear velocity:
        self.velocity["linear"]["x"] = msg.linear.x
        self.velocity["linear"]["y"] = msg.linear.y
        self.velocity["linear"]["z"] = msg.linear.z
        # angular velocity:
        self.velocity["angular"]["x"] = msg.angular.x
        self.velocity["angular"]["y"] = msg.angular.y
        self.velocity["angular"]["z"] = msg.angular.z
        # print them as logs:
        print(self.velocity)
        # set update flag
        self.update = True
        # calculate motors values:
        self.teleop.calculate(self.velocity)
        self.ipico_drvs.move_command(move_type=self.ipico_drvs.move_type.velocity, action_type=self.ipico_drvs.action_type.set,value=self.teleop.motor_request["M1"])
        self.ipico_drvs.move_command(move_type=self.ipico_drvs.move_type.velocity, action_type=self.ipico_drvs.action_type.set,value=self.teleop.motor_request["M2"],driver_nr=2)
        self.teleop.motor["M1"] = self.teleop.motor_request["M1"]
        self.teleop.motor["M2"] = self.teleop.motor_request["M2"]

class teleop_twist():
    def __init__(self):
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
        }
        self.motor = {
            "M1" : 0,
            "M2" : 0
        }
        self.motor_request = {
            "M1" : 0,
            "M2" : 0
        }
        self.ack = False
        self.reached = False
    #basic calculation template with linear function - transformation
    def calculate(self, arg_velocity):
        # linear velocity calculation:
        self.motor_request["M1"] = arg_velocity["linear"]["x"] * 5
        self.motor_request["M2"] = self.motor_request["M1"]
        # angular velocity calculation:
        self.motor_request["M1"] = arg_velocity["angular"]["z"] * 5 + self.motor_request["M1"]
        self.motor_request["M2"] = arg_velocity["angular"]["z"] * -5 + self.motor_request["M2"]
        #check if calculation for M1 didnt reach the limit <-100,100>
        if not self.motor_request["M1"] in range(-100,100):
            #set to max limit lower/upper:
            if self.motor_request["M1"] < 0:
                self.motor_request["M1"] = -100
            else:
                self.motor_request["M1"] = 100
        #check if calculation for M2 didnt reach the limit <-100,100>
        if not self.motor_request["M2"] in range(-100,100):
            #set to max limit lower/upper:
            if self.motor_request["M2"] < 0:
                self.motor_request["M2"] = -100
            else:
                self.motor_request["M2"] = 100

class ipico_drvs():
    class action_type(Enum):
        get = False
        set = True
    class move_type(Enum):
        step = "step"
        velocity = "vel"
    def __init__(self, node):
        self.feedback = True
        self.request_commands = {
            "check":'drv_CHECK\n',
            "start":'drv_START\n',
            "stop":'drv_STOP\n',
            "run":'drv_RUN\n',
            "safety_stop":'drv_SAFETY_STOP\n',
            "fullstep":'drv_FSTEP\n',
            "halfstep":'drv_HSTEP\n',
            "step":'drv_STEP=',
            "vel":'drv_VEL='
        }
        self.ros_node = node
        self.drvs_init_procedure()

    def check_connection(self):
        i = 0
        # send check command via uart to pico_drvs
        self.ros_node.uart_pub.publish(self.request_commands["check"])
        # wait for response - 10 seconds
        print("Waiting for check connection response..")
        while not self.ros_node.rx_update:
            if i > 10:
                return False
            i = i + 1
            sleep(1)
        #response obtained
        # TODO: procedure for identify response:
        return True

    def drvs_init_procedure(self):
        # first check connection with command
        if not self.check_connection():
            raise Exception("Check connection procedure failed")
        # second start drivers
        self.ros_node.uart_pub.publish(self.request_commands["start"])
    # private function for checking if calling driver number is correct
    def __check_driver_number(self, driver_nr):
        if driver_nr > 2:
            raise Exception("Given number of driver is greater then 2 there are only 2 drivers")
        if driver_nr < 1:
            raise Exception("Given number of driver is less then 1, there are only 2 drivers: numbered 1 and 2")
    # main move command function used to publish moving commands like step and vel
    def move_command(self, move_type = move_type.velocity ,action_type = action_type.get, driver_nr = 1, value = 0):
        #catch core command text structure
        cmd = self.request_commands[move_type]
        #check if given number is correct
        self.__check_driver_number(driver_nr=driver_nr)
        #building command related to action type
        if not action_type:
            cmd = cmd + str(driver_nr) + ',?\n'
        else:
            cmd = cmd + str(driver_nr) + ',' + str(value) + '\n'
        #send prepared command
        self.ros_node.uart_pub.publish(cmd)
    
    def send_command(self, request):
        # check if request is command
        if not request in self.request_commands:
            print("There are not such a command in list")
            return False
        # if its pose or velocity command then realize standard procedure for both drivers
        if request == self.move_type.step:
            self.move_command(move_type=self.move_type.step)
            self.move_command(move_type=self.move_type.step,driver_nr=2)
            return True
        if request == self.move_type.velocity:
            self.move_command(move_type=self.move_type.velocity)
            self.move_command(move_type=self.move_type.velocity,driver_nr=2)
            return True
        self.ros_node.uart_pub.publish(self.request_commands[request])
        return True
    # when deleting object: stop drivers
    def __del__(self):
        self.ros_node.uart_pub.publish(self.request_commands["stop"])

def main(arg=None):
    rclpy.init(args=arg)
    ros_node = ipico_node()
    rclpy.spin(ros_node)
    #when finished destroy node and delete objects:
    ros_node.destroy_node()
    del ros_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()