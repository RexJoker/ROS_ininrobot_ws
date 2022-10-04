from enum import Enum
from time import sleep
from typing import NamedTuple
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist as BSmart #easter egg
from std_msgs.msg import String
from teleop_twist_class import teleop_twist

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
        self.teleop = teleop_twist(self)

    def uart_callback(self, msg):
        self.feedback = msg.data
        self.rx_update = True
        self.ipico_drvs.read_data(self.feedback)

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
        #print(self.velocity)
        # set update flag
        self.update = True
        # calculate motors values:
        self.teleop.velocity = self.velocity
        self.ipico_drvs.move_command(move_type=self.ipico_drvs.move_type.velocity.value, action_type=self.ipico_drvs.action_type.set.value,value=self.teleop.motor_request["M1"])
        self.ipico_drvs.move_command(move_type=self.ipico_drvs.move_type.velocity.value, action_type=self.ipico_drvs.action_type.set.value,value=self.teleop.motor_request["M2"],driver_nr=2)
        self.teleop.motor["M1"] = self.teleop.motor_request["M1"]
        self.teleop.motor["M2"] = self.teleop.motor_request["M2"]
    def construct_string_msg(self,arg_string):
        msg = String()
        msg.data = arg_string
        return msg

class ipico_drvs():
    class action_type(Enum):
        get = False
        set = True
    class move_type(Enum):
        step = "step"
        velocity = "vel"
    class cmd():
        def __init__(self):
            self.name = None
            self.action_type = None
            self.move_type = None
            self.motor_number = 0
    def __init__(self, node):
        self.feedback = {
            "Name" : None,
            "Value" : 0
        }
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
        self.__last_command = self.cmd()
        self.ros_node = node
        self.drvs_init_procedure()

    def check_connection(self):
        i = 0
        # send check command via uart to pico_drvs
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["check"]))
        # wait for response - 10 seconds
        print("Waiting for check connection response..")
        while not self.ros_node.rx_update:
            if i > 10:
                return True #here should be False
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
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["start"]))
        self.remember_command("start")
    # private function for checking if calling driver number is correct
    def __check_driver_number(self, driver_nr):
        if driver_nr > 2:
            raise Exception("Given number of driver is greater then 2 there are only 2 drivers")
        if driver_nr < 1:
            raise Exception("Given number of driver is less then 1, there are only 2 drivers: numbered 1 and 2")
    # main move command function used to publish moving commands like step and vel
    def move_command(self, move_type = move_type.velocity.value ,action_type = action_type.get.value, driver_nr = 1, value = 0):
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
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(cmd))
        #remember last command:
        self.remember_command(move_type,arg_move_type=move_type,arg_action_type=action_type,arg_driver_nr=driver_nr)
    
    def send_command(self, request):
        # check if request is command
        if not request in self.request_commands:
            print("There are not such a command in list")
            return False
        # if its pose or velocity command then realize standard procedure for both drivers
        if request == self.move_type.step.value:
            self.move_command(move_type=self.move_type.step.value)
            self.move_command(move_type=self.move_type.step.value,driver_nr=2)
            return True
        if request == self.move_type.velocity.value:
            self.move_command(move_type=self.move_type.velocity.value)
            self.move_command(move_type=self.move_type.velocity.value,driver_nr=2)
            return True
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands[request]))
        #remember last command:
        self.remember_command(request)
        return True
    def read_data(self, arg_msg):
        self.feedback["Name"] = self.__last_command.name
        self.feedback["Value"] = arg_msg
    # when deleting object: stop drivers
    def remember_command(self, arg_name, arg_move_type = None, arg_action_type = None, arg_driver_nr = 0):
        self.__last_command.name = arg_name
        self.__last_command.move_type = arg_move_type
        self.__last_command.action_type = arg_action_type
        self.__last_command.motor_number = arg_driver_nr
    def get_vels(self):
        motor = {
            "M1" : 0,
            "M2" : 0
        }
        #iterate through 2 motors
        for n in range (1,2):
            #send command for getting velocity from motor n
            self.move_command(driver_nr=n)
            i = 0
            #wait until msg appears on uart rx
            while not self.ros_node.rx_update:
                #try 5 times every 1 second to get data
                if i > 5:
                    #if there is no any msg to recievie then its bad
                    raise Exception("Wait for response too long")
                sleep(1)
            # if data appears then check it and pass it to variable
            if self.feedback["Name"] == "vel" and self.__last_command.action_type == self.action_type.get.value:
                txt = "M" + n
                motor[txt] = self.feedback["Value"]
            else:
                raise Exception("Bad feedback")
        return motor
    def __del__(self):
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["stop"]))

def main(arg=None):
    rclpy.init(args=arg)
    ros_node = ipico_node()
    rclpy.spin(ros_node)
    #when finished destroy node and delete objects:
    del ros_node
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()