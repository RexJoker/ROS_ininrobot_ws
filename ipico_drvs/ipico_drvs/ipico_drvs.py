from email.quoprimime import body_check
from enum import Enum
from time import sleep
from typing import NamedTuple
from xmlrpc.client import boolean
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist as BSmart #easter egg
from std_msgs.msg import String
import threading

class ipico_node(Node):
    def __init__(self):
        super().__init__('ipico_node')
        #initialize subscriptions and publishing for Ipico_node
        self.vel_sub = self.create_subscription(BSmart, 'cmd_vel', self.cmd_vel_callback, 1)
        self.uart_sub = self.create_subscription(String, 'UARTRX_topic', self.uart_callback, 1)
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
        if self.ipico_drvs.read_data(self.feedback):
            self.feedback = "None"

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
        # set update flag
        self.update = True
        # calculate motors values:
        self.teleop.velocity = self.velocity
        self.teleop.reached = False
    def construct_string_msg(self,arg_string):
        msg = String()
        msg.data = arg_string
        return msg

class ipico_drvs(threading.Thread):
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
        self.initialized = False
        self.ros_node = node
        threading.Thread.__init__(self)
        threading.Thread.start(self)

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
            print("waiting..")
            sleep(1)
        #response obtained
        print("Connection checked! - All good")
        # TODO: procedure for identify response:
        self.ros_node.rx_update = False
        return True

    def drvs_init_procedure(self):
        # first check connection with command
        if not self.check_connection():
            raise Exception("Check connection procedure failed")
        # second start drivers
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["start"]))
        self.remember_command("start")
        print("IPICO DRIVER Initialized!")
        self.initialized = True
        self.ros_node.rx_update = False
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
        if not self.feedback["Value"] == arg_msg:
            return False
        return True
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
        for n in range (1,3):
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
                txt = "M" + str(n)
                motor[txt] = int(self.feedback["Value"])
                self.ros_node.rx_update = False
            else:
                raise Exception("Bad feedback")
        return motor
    def run(self):
        self.drvs_init_procedure()
     # when deleting object: stop drivers
    def __del__(self):
        self.ros_node.teleop.end = True
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["stop"]))

class teleop_twist(threading.Thread):
    class offset(Enum):
        LL_OFF = -2
        HH_OFF = 2
    def __init__(self,arg_node):
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
        }
        self.motor = {
            "last" : {"M1":0 , "M2":0},
            "actual" : {"M1":0 , "M2":0},
            "goal": {"M1":0 , "M2":0},
            "request": {"M1":0 , "M2":0}
        }
        self.const = 1
        self.reached = True
        self.end = False
        self.ros_node = arg_node
        threading.Thread.__init__(self)
        threading.Thread.start(self)
    def request_velocity(self):
        self.__calculate(self.velocity)
        #print("Actual:")
        #print(self.motor["actual"])
        #print("Goal:")
        #print(self.motor["goal"])
        self.following_control()
        #print("Request:")
        #print(self.motor["request"])
    def limit_speed(self, arg_speed):
        #check if calculation for speed didnt reach the limit <-100,100>
        if not arg_speed in range(-100,101):
            #set to max limit lower/upper:
            if arg_speed < 0:
                return -100
            else:
                return 100
        return arg_speed
    #basic calculation template with linear function - transformation
    def __calculate(self, arg_velocity):
        # linear velocity calculation:
        self.motor["goal"]["M1"] = arg_velocity["linear"]["x"] * 5
        self.motor["goal"]["M2"] = self.motor["goal"]["M1"]
        # angular velocity calculation:
        self.motor["goal"]["M1"] = round(arg_velocity["angular"]["z"] * 5 + self.motor["goal"]["M1"])
        self.motor["goal"]["M2"] = round(arg_velocity["angular"]["z"] * -5 + self.motor["goal"]["M2"])
        #check if calculation for M1 and M2 didnt reach the limit <-100,100>
        for motor_key in self.motor["goal"]:
            self.motor["goal"][motor_key] = self.limit_speed(self.motor["goal"][motor_key])
    def run(self):
        while not self.ros_node.ipico_drvs.initialized:
            print("Waiting for ipico driver init...")
            sleep(1)
        while True:
            while not self.reached:
                self.motor["last"] = self.motor["actual"]
                self.motor["actual"] = self.ros_node.ipico_drvs.get_vels()
                self.request_velocity()
                if self.check_goal( self.motor["last"], self.motor["actual"], self.motor["goal"]):
                    self.reached = True

            #send data through uart by ipico drivers
            if self.end:
                return

    #following control function for smoothly controling motors
    def following_control(self):
        for motor_key in self.motor["goal"]:
            #estabilishing if alorithm need to acc or deacc velocity to reach goal
            coeff = -1
            if self.motor["goal"][motor_key] > self.motor["actual"][motor_key]:
                coeff = 1
            #calculate new requested velocity for motor:
            self.motor["request"][motor_key] = self.motor["actual"][motor_key] + (coeff * self.const)
            #limit requested velocity:
            self.motor["request"][motor_key] = self.limit_speed(self.motor["request"][motor_key])       

    def check_goal(self, arg_LastMotor, arg_ActMotor, arg_GoalMotor):
        boolean_motors = [False, False]
        i = 0
        for motor_key in arg_ActMotor:
            if arg_ActMotor[motor_key] in range((arg_GoalMotor[motor_key] + self.offset.LL_OFF.value),(arg_GoalMotor[motor_key] + self.offset.HH_OFF.value)):
                boolean_motors[i] = True
            i = i + 1
            #2nd reason to return true: when the motors is not making any better results

        if boolean_motors[0] and boolean_motors[1]:
            return True
        return False

    def __del__(self):
        self.end = True

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