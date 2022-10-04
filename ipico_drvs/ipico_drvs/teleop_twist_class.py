from concurrent.futures import thread
import threading

class teleop_twist(threading.Thread):
    def __init__(self,arg_node):
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
        }
        self.motor = {
            "M1" : 0,
            "M2" : 0
        }
        self.motor_goal = {
            "M1" : 0,
            "M2" : 0
        }
        self.motor_request = {
            "M1" : 0,
            "M2" : 0
        }
        self.const = 1
        self.ack = False
        self.reached = False
        self.__end = True
        self.ros_node = arg_node
        threading.Thread.__init__(self)
    def request_velocity(self):
        self.__calculate(self.velocity)
        #self.ack = True
        #self.reached = False
        self.following_control()
    def limit_speed(self, arg_speed):
        #check if calculation for speed didnt reach the limit <-100,100>
        if not arg_speed in range(-100,100):
            #set to max limit lower/upper:
            if arg_speed < 0:
                return -100
            else:
                return 100
        return arg_speed
    #basic calculation template with linear function - transformation
    def __calculate(self, arg_velocity):
        # linear velocity calculation:
        self.motor_goal["M1"] = arg_velocity["linear"]["x"] * 5
        self.motor_goal["M2"] = self.motor_goal["M1"]
        # angular velocity calculation:
        self.motor_goal["M1"] = round(arg_velocity["angular"]["z"] * 5 + self.motor_goal["M1"])
        self.motor_goal["M2"] = round(arg_velocity["angular"]["z"] * -5 + self.motor_goal["M2"])
        #check if calculation for M1 and M2 didnt reach the limit <-100,100>
        for motor_key in self.motor_goal:
            self.motor_goal[motor_key] = self.limit_speed(self.motor_goal[motor_key])
    def run(self):
        while True:
            self.motor = self.ros_node.ipico_drvs.get_vels()
            self.request_velocity()
            #send data through uart by ipico drivers
            if self.__end:
                return

    #following control function for smoothly controling motors
    def following_control(self):
        for motor_key in self.motor_goal:
            #estabilishing if alorithm need to acc or deacc velocity to reach goal
            coeff = -1
            if self.motor_goal[motor_key] < self.motor[motor_key]:
                coeff = 1
            #calculate new requested velocity for motor:
            self.motor_request[motor_key] = self.motor[motor_key] + (coeff * self.const)
            #limit requested velocity:
            self.motor_request[motor_key] = self.limit_speed(self.motor_request[motor_key])
        