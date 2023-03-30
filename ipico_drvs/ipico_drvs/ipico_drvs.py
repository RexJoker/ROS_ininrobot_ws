from enum import Enum
from time import sleep
import time
from xmlrpc.client import boolean
import rclpy
import tf2_ros
import math
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped #easter egg
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import threading

class ipico_node(Node):
    def __init__(self):
        super().__init__('ipico_node')
        #initialize subscriptions and publishing for Ipico_node
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.uart_sub = self.create_subscription(String, 'UARTRX_topic', self.uart_callback, 1)
        self.uart_pub = self.create_publisher(String, 'UARTTX_topic', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.update = False
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
            }
        self.rx_update = False
        self.vel_update = True
        self.feedback = "None"
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
        self.header_stamp = self.current_time.to_msg()
        self.vel_sub
        self.uart_sub
        self.ipico_drvs = ipico_drvs(self)
        self.teleop = teleop_twist(self)
        self.rw = 0.1 # 10 cm in meters // diameter between wheels
        self.l = 0.2 # 20cm in meter // distance between wheels
    def uart_callback(self, msg):
        self.feedback = msg.data
        self.rx_update = True
        # if there was command for getting data then read it:
        if self.ipico_drvs.last_command.action_type == self.ipico_drvs.action_type.get.value:
            self.ipico_drvs.read_data(self.feedback)
    #check for response function to look if the text answer is as expected
    def check_for_response(self, expected_answer = "OK"):
        i = 0
        while not self.rx_update:
            if i > 10:
                return False
            i += 1
            sleep(0.1)
        if self.feedback == expected_answer:
            self.feedback = "None"
            self.rx_update = False
            return True
        return False

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
    def construct_send_odom(self,wl,wr):
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time ).nanoseconds / 1e9
        self.y = 0.0
        self.vx = self.velocity["linear"]["x"]
        self.vy = 0.0
        self.vth = self.velocity["angular"]["z"]
        delta_th = (self.rw/self.l)*(wr-wl)*dt
        self.th = self.th + delta_th
        delta_x = math.cos(self.th)*(self.rw/2)*(wr+wl)*dt
        delta_y = math.sin(self.th)*(self.rw/2)*(wr+wl)*dt
        self.x = self.x + delta_x
        self.y = self.y + delta_y

        # # first, we'll publish the transform over tf
        # self.odom_broadcaster.sendTransform(
        #     (self.x, self.y, 0.),
        #     odom_quat,
        #     self.current_time,
        #     "base_link",
        #     "odom"
        # )

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = "base_link"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = quaternion_from_euler(0, 0, self.th)

        t.transform.rotation.x = odom_quat[0]
        t.transform.rotation.y = odom_quat[1]
        t.transform.rotation.z = odom_quat[2]
        t.transform.rotation.w = odom_quat[3]

        # Send the transformation
        self.odom_broadcaster.sendTransform(t)

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = "odom"
        
        # set the position
        pt = Point()
        pt.x = self.x
        pt.y = self.y
        pt.z = 0.
        quat = Quaternion()
        quat.x = odom_quat[0]
        quat.y = odom_quat[1]
        quat.z = odom_quat[2]
        quat.w = odom_quat[3]
        pose1 = Pose()
        pose1.position = pt
        pose1.orientation = quat
        odom.pose.pose = pose1
        vec = Vector3()
        vec.x = self.vx
        vec.y = self.vy
        vec.z = 0.
        vec2 = Vector3()
        vec2.x = 0.
        vec2.y = 0.
        vec2.z = self.vth
        twist1 = Twist()
        twist1.linear = vec
        twist1.angular = vec2
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = twist1

        # publish the message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

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
            "check":'drv_CHECK',
            "start":'drv_ENABLE',
            "disable" : 'drv_DISABLE',
            "stop":'drv_STOP',
            "run":'drv_RUN',
            "safety_stop":'drv_SAFETY_STOP',
            "fullstep":'drv_FSTEP',
            "halfstep":'drv_HSTEP',
            "step":'drv_POS=',
            "vel":'drv_VEL='
        }
        self.last_command = self.cmd()
        self.initialized = False
        self.ros_node = node
        threading.Thread.__init__(self,daemon=True)
        threading.Thread.start(self)

    def check_connection(self):
        i = 0
        # send check command via uart to pico_drvs
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["check"]))
        self.remember_command("check")
        # wait for response - 10 seconds
        print("Waiting for check connection response..")
        self.ros_node.get_logger().info("Waiting for check connection response..")
        while not self.ros_node.rx_update:
            if i > 10:
                return False
            i += 1
            print("waiting..")
            self.ros_node.get_logger().info("waiting..")
            sleep(1)
        # response check:
        if not self.ros_node.check_for_response():
            print("Missing response after sending check connection command")
            self.ros_node.get_logger().info("Missing response after sending check connection command")
        #response obtained
        print("Connection checked! - All good")
        self.ros_node.get_logger().info("Connection checked! - All good")
        return True
    #basic init procedure:
    def drvs_init_procedure(self):
        error_flag = False
        # first check connection with command
        # if not self.check_connection():
        #     print("Check connection procedure failed")
        #     print("Trying to resolve problem..")
        #     self.ros_node.get_logger().info("Check connection procedure failed")
        #     self.ros_node.get_logger().info("Trying to resolve problem..")
        #     #trying to solve problem by sending some control commands:
        #     self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["stop"]))
        #     self.remember_command("stop")
        #     sleep(0.1)
        #     self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["disable"]))
        #     self.remember_command("disable")
        #     self.ros_node.rx_update = False
        #     if not self.check_connection():
        #         raise Exception("Check connection procedure failed")
        # second start drivers
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["fullstep"]))
        self.remember_command("fullstep")
        if not self.ros_node.check_for_response():
            print("Missing response after sending fullstep command")
            self.ros_node.get_logger().info("Missing response after sending fullstep command")
            error_flag = True
        sleep(0.1)
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["start"]))
        self.remember_command("start")
        if not self.ros_node.check_for_response(expected_answer="--"):
            print("Missing response after sending enable command")
            self.ros_node.get_logger().info("Missing response after sending enable command")
            error_flag = True
        sleep(0.1)
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(self.request_commands["run"]))
        self.remember_command("run")
        if not self.ros_node.check_for_response():
            print("Missing response after sending run command")
            self.ros_node.get_logger().info("Missing response after sending run command")
            error_flag = True
        #TODO: get vels and step to update parameters of object
                
        #TODO: Dynamic POS value
        sleep(0.1)
        self.ros_node.ipico_drvs.move_command(move_type=self.ros_node.ipico_drvs.move_type.step.value, action_type=self.ros_node.ipico_drvs.action_type.set.value, driver_nr=1,value=100)
        sleep(0.1)
        self.ros_node.ipico_drvs.move_command(move_type=self.ros_node.ipico_drvs.move_type.step.value, action_type=self.ros_node.ipico_drvs.action_type.set.value, driver_nr=2,value=100)
        
        print("IPICO DRIVER Initialized!")
        self.ros_node.get_logger().info("IPICO DRIVER Initialized!")
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
            cmd = cmd + str(driver_nr) + ',?'
        else:
            cmd = cmd + str(driver_nr) + ',' + str(value)
        #send prepared command
        self.ros_node.uart_pub.publish(self.ros_node.construct_string_msg(cmd))
        #remember last command:
        self.remember_command(move_type,arg_move_type=move_type,arg_action_type=action_type,arg_driver_nr=driver_nr)

    def read_data(self, arg_msg):
        self.feedback["Name"] = self.last_command.name
        self.feedback["Value"] = arg_msg
        if not self.feedback["Value"] == arg_msg:
            return False
        return True
    def remember_command(self, arg_name, arg_move_type = None, arg_action_type = None, arg_driver_nr = 0):
        self.last_command.name = arg_name
        self.last_command.move_type = arg_move_type
        self.last_command.action_type = arg_action_type
        self.last_command.motor_number = arg_driver_nr
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
                    print("Waited for get vel response too long")
                    self.ros_node.get_logger().info("Waited for get vel response too long")
                sleep(0.1)
                i += 1
            # if data appears then check it(just to be sure it's the correct data) and pass it to variable
            if self.feedback["Name"] == "vel" and self.last_command.action_type == self.action_type.get.value:
                txt = "M" + str(n)
                motor[txt] = int(self.feedback["Value"])
                self.ros_node.rx_update = False
            else:
                print("Bad get vel feedback")
                self.ros_node.get_logger().info("Bad get vel feedback")
        return motor
    #main thread function for init procedure
    def run(self):
        self.drvs_init_procedure()
        return

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
        self.delays = 0
        self.delays2 = 0
        self.last_time = 0.000 
        self.delay_time = 0.2 #delay time specified in seconds
        self.reached = True
        self.end = False
        self.ros_node = arg_node
        self.wl = 0.00
        self.wr = 0.00
        self.l = 0.2
        self.rw = 0.1
        self.d1 = 0
        self.d2 = 0
        threading.Thread.__init__(self,daemon=True)
        threading.Thread.start(self)
    #check function for delaying changes into speeds of motors
    def is_time_delayed(self):
        rn_time = time.time()
        if(rn_time - self.last_time) > self.delay_time:
            self.last_time = rn_time
            return True
        return False
    #function for calculating data, request velocities and send commands
    def request_velocity(self):
        error_flag = False
        self.__calculate(self.velocity)
        #temporary disabling following control and giving goal values to request
        self.motor["request"]["M1"] = self.motor["goal"]["M1"]
        self.motor["request"]["M2"] = self.motor["goal"]["M2"]
        self.motor["request"]["M1"] = self.limit_speed(self.motor["request"]["M1"]) 
        self.motor["request"]["M2"] = self.limit_speed(self.motor["request"]["M2"]) 
        if self.motor["request"]["M1"] != 0:
            self.ros_node.ipico_drvs.move_command(action_type = self.ros_node.ipico_drvs.action_type.set.value, driver_nr = 1, value = abs(self.motor["request"]["M1"]))
            if not self.ros_node.check_for_response():
                print("Missing response after sending move command")
                self.ros_node.get_logger().info("Missing response after sending move command")
                error_flag = True
        if self.motor["request"]["M2"] != 0:
            self.ros_node.ipico_drvs.move_command(action_type = self.ros_node.ipico_drvs.action_type.set.value, driver_nr = 2, value = abs(self.motor["request"]["M2"]))
            if not self.ros_node.check_for_response():
                print("Missing response after sending move command")
                self.ros_node.get_logger().info("Missing response after sending move command")
                error_flag = True
        
    #typical function for limiting data speeds of motors 
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
        self.motor["goal"]["M1"] = arg_velocity["linear"]["x"] * 40
        self.motor["goal"]["M2"] = self.motor["goal"]["M1"]
        # angular velocity calculation:
        self.motor["goal"]["M1"] = round(arg_velocity["angular"]["z"] * 40 + self.motor["goal"]["M1"])
        self.motor["goal"]["M2"] = round(arg_velocity["angular"]["z"] * -40 + self.motor["goal"]["M2"])
        #check if calculation for M1 and M2 didnt reach the limit <-100,100>
        for motor_key in self.motor["goal"]:
            self.motor["goal"][motor_key] = self.limit_speed(self.motor["goal"][motor_key])
    #run function which is main thread func.
    def run(self):
        attempts = 0
        print("Thread: Waiting for ipico driver init...")
        self.ros_node.get_logger().info("Thread: Waiting for ipico driver init...")
        #wait for init ipico_drvs:
        while not self.ros_node.ipico_drvs.initialized:
            #if driver didnt get response after 10 seconds then print error and end
            if attempts > 10:
                print("Thread:Something goes wrong with ipico_drvs init, ending..")
                self.ros_node.get_logger().info("Thread:Something goes wrong with ipico_drvs init, ending..")
                return
            attempts += 1
            sleep(1)
        #main thread loop:
        while True:
            #if velocity data points didnt reached goal:
            while not self.reached:
                if self.is_time_delayed():
                    self.motor["actual"] = self.motor["request"]
                    self.motor["last"] = self.motor["actual"]
                    self.request_velocity()
                    self.update_steps()
                    #if new values of motor reach goal then stop:
                    if self.check_goal(self.motor["last"], self.motor["actual"], self.motor["goal"]):
                        if (self.velocity["linear"]["x"] == 0) and (self.velocity["angular"]["z"] == 0):
                            self.reached = True
                            self.d1 = 0
                            self.d2 = 0
                self.wl = (self.velocity["linear"]["x"]/self.rw) - (self.l * self.velocity["angular"]["z"])/(2*self.rw)
                self.wr = (self.velocity["linear"]["x"]/self.rw) + (self.l * self.velocity["angular"]["z"])/(2*self.rw)
                self.ros_node.construct_send_odom(self.wl,self.wr)
            # self.wl = (self.velocity["linear"]["x"]/self.rw) - (self.l * self.velocity["angular"]["z"])/(2*self.rw) * self.d1
            # self.wr = (self.velocity["linear"]["x"]/self.rw) + (self.l * self.velocity["angular"]["z"])/(2*self.rw) * self.d2
            # self.ros_node.construct_send_odom(self.wl,self.wr)
            #send data through uart by ipico drivers
            if self.end:
                return
    # synchronous, forcing steps on drivers
    def update_steps(self):
        steps = {
            "M1" : 300,
            "M2" : 300
        }
        self.d1 = 1
        self.d2 = 1
        if self.motor["request"]["M1"] < 0:
            steps["M1"] = -300
            self.d1 = -1
        elif self.motor["request"]["M1"] == 0:
            steps["M1"] = 0
            self.d1 = 0
        if self.motor["request"]["M2"] < 0:
            steps["M2"] = -300
            self.d2 = -1
        elif self.motor["request"]["M2"] == 0:
            steps["M2"] = 0
            self.d2 = 0

        if steps["M1"] != 0:
            self.ros_node.ipico_drvs.move_command(move_type=self.ros_node.ipico_drvs.move_type.step.value, action_type=self.ros_node.ipico_drvs.action_type.set.value, driver_nr=1,value=steps["M1"])
            if not self.ros_node.check_for_response():
                print("Motor1 is busy")
                self.ros_node.get_logger().info("Motor1 is busy")
                error_flag = True
        if steps["M2"] != 0:
            self.ros_node.ipico_drvs.move_command(move_type=self.ros_node.ipico_drvs.move_type.step.value, action_type=self.ros_node.ipico_drvs.action_type.set.value, driver_nr=2,value=steps["M2"])
            if not self.ros_node.check_for_response():
                print("Motor2 is busy")
                self.ros_node.get_logger().info("Motor2 is busy")
                error_flag = True
    #following control function for smoothly controling motors
    def following_control(self):
        for motor_key in self.motor["goal"]:
            #estabilishing if alorithm need to acc or deacc velocity to reach goal
            coeff = -1
            if self.motor["goal"][motor_key] > self.motor["actual"][motor_key]:
                coeff = 1
            #calculate new requested velocity for motor:
            abs_delta = abs(self.motor["goal"][motor_key] - self.motor["actual"][motor_key]) 
            if abs_delta > 2:
                step_value = abs_delta * 0.5
                if step_value < 2:
                    step_value = 1
                self.motor["request"][motor_key] = self.motor["actual"][motor_key] + (coeff * int(step_value))
            else:
                self.motor["request"][motor_key] = self.motor["actual"][motor_key] + (coeff * self.const)
            #limit requested velocity:
            self.motor["request"][motor_key] = self.limit_speed(self.motor["request"][motor_key])       
    #check function for reaching the goal of velocity data points of motors:
    def check_goal(self, arg_LastMotor, arg_ActMotor, arg_GoalMotor):
        boolean_motors = [False, False]
        i = 0
        #check every motor for:
        for motor_key in arg_ActMotor:
            #for reaching the goal in specific range (<goal-1,goal+1>)
            if arg_ActMotor[motor_key] in range((arg_GoalMotor[motor_key] + self.offset.LL_OFF.value),(arg_GoalMotor[motor_key] + self.offset.HH_OFF.value)):
                boolean_motors[i] = True
            i = i + 1
            #2nd reason to return true: when the motors is not making any better results
        #if actual motor velocity data is correct then return true:
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