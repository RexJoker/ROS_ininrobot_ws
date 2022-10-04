from concurrent.futures import thread
import threading
class teleop_twist(thread):
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
    def request_velocity(self, req_velocity):
        #copying linear velocity params:
        self.velocity["linear"]["x"] = req_velocity["linear"]["x"]
        #copying angular velocity params:
        self.velocity["angular"]["z"] = req_velocity["angular"]["z"]
        self.__calculate(self.velocity)
        self.ack = True
        self.reached = False
        self.following_control()
    #basic calculation template with linear function - transformation
    def __calculate(self, arg_velocity):
        # linear velocity calculation:
        self.motor_request["M1"] = arg_velocity["linear"]["x"] * 5
        self.motor_request["M2"] = self.motor_request["M1"]
        # angular velocity calculation:
        self.motor_request["M1"] = round(arg_velocity["angular"]["z"] * 5 + self.motor_request["M1"])
        self.motor_request["M2"] = round(arg_velocity["angular"]["z"] * -5 + self.motor_request["M2"])
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
    #following control function for smoothly controling motors
    def following_control(self):
        self.motor["M1"] = self.motor_request["M1"]