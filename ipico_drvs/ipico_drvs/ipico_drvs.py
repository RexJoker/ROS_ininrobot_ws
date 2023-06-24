from time import sleep
import time
import rclpy
import tf2_ros
import math
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped #easter egg
from std_msgs.msg import String
from nav_msgs.msg import Odometry


WHEEL_LENGHT = 0.175 # 17,5 cm distance between wheels [meters]
WHEEL_RADIUS = 0.046 # 4,6 cm radius of wheel [meters]
ROBOT_MAX_VEL_MOTOR = 0.5*math.pi() # rad/s
PRECISION_SIGNAL = 3 # number of digits that will be send via uart

class ipico_node(Node):
    def __init__(self):
        super().__init__('ipico_node')
        #initialize subscriptions and publishing for Ipico_node
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)
        self.uart_sub = self.create_subscription(String, 'UARTRX_topic', self.uart_callback, 1)
        self.uart_pub = self.create_publisher(String, 'UARTTX_topic', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self.velocity = {
            "linear" : {"x":0.0,"y":0.0,"z":0.0},
            "angular": {"x":0.0,"y":0.0,"z":0.0},
            }
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
        self.timer = self.create_timer(0.1,self.timer_callback)
    def timer_callback(self):
        self.construct_send_odom(self.ipico_drvs.Velocity["Requested"]["Left"],self.ipico_drvs.Velocity["Requested"]["Right"])
    def uart_callback(self, msg):
        if not self.ipico_drvs.Update_RealVel(msg.data):
            self.get_logger().info("[WARNING] Uart callback, conversion failed")
            return
        self.construct_send_odom(self.ipico_drvs.Velocity["Real"]["Left"],self.ipico_drvs.Velocity["Real"]["Right"])

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
        # calculate wheel velocites:
        self.ipico_drvs.CalculateWheel_Velocity(msg)

    def publish_uart(self,arg_string):
        msg = String()
        msg.data = arg_string
        
        self.uart_pub.publish(msg)
    def construct_send_odom(self,wl,wr):
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time ).nanoseconds / 1e9
        self.y = 0.0
        self.vx = self.velocity["linear"]["x"]
        self.vy = 0.0
        self.vth = self.velocity["angular"]["z"]
        delta_th = (WHEEL_RADIUS/WHEEL_LENGHT)*(wr-wl)*dt
        self.th = self.th + delta_th
        delta_x = math.cos(self.th)*(WHEEL_RADIUS/2)*(wr+wl)*dt*10
        delta_y = math.sin(self.th)*(WHEEL_RADIUS/2)*(wr+wl)*dt*10
        print(self.x)
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

class ipico_drvs(object):
    def __init__(self,node = None):
        self.Velocity = {
            "Real":{
                "Left" : 0.00,
                "Right" : 0.00
            },
            "Requested":{
                "Left" : 0.00,
                "Right" : 0.00
            }
        }
        self.ros_node = node
        self.UART_HEADERS = ["M1=","M2="] # Motor Left = M1, Motor Right = M2
    def CalculateWheel_Velocity(self, velocity):
        # Calculate desired wheel velocities, based on odometry:
        self.Velocity["Requested"]["Left"] = (velocity.linear.x/WHEEL_RADIUS) - (WHEEL_LENGHT * velocity.angular.z)/(2*WHEEL_RADIUS)
        self.Velocity["Requested"]["Right"] = (velocity.linear.x/WHEEL_RADIUS) + (WHEEL_LENGHT * velocity.angular.z)/(2*WHEEL_RADIUS)
        # Convert velocites to control signals for Pico
        converted_vels = self.Convert_velocity([self.Velocity["Requested"]["Left"],self.Velocity["Requested"]["Right"]])
        # Velocity saturation
        self.Velocity["Requested"]["Left"] = self.Limit_Velocities(self.Velocity["Requested"]["Left"])
        self.Velocity["Requested"]["Right"] = self.Limit_Velocities(self.Velocity["Requested"]["Right"])
        # Control signals should be published to UART:
        self.Send_Signals(converted_vels)
    def Send_Signals(self, vels):
        j = 0
        for i in self.UART_HEADERS:
            rounded_value = round(vels[j],PRECISION_SIGNAL)
            text = i + str(rounded_value) + "_"
            if j == 0:
                text = i + str(-rounded_value) + "_"
            self.ros_node.publish_uart(text)
            time.sleep(0.01)
            j += 1
    def Convert_velocity(self, wheel_vels):
        saturated_velocities = [0.00,0.00]
        j = 0
        # Velocity saturation [-MAX,MAX]:
        for i in wheel_vels:
            saturated_velocities[j] = self.Limit_Velocities(i)
            j += 1
        # Convert velocity to control signal between <-1,1> for pico:
        j = 0
        for i in saturated_velocities:
            # f(x) = ax +b
            # b = 0
            # a = 1/MAX_VEL
            #f(x) := <-1,1>
            saturated_velocities[j] = (1/ROBOT_MAX_VEL_MOTOR) * saturated_velocities[j]
            j+=1
        
        return saturated_velocities
    def Limit_Velocities(self, velocity):
        if velocity >= 0:
            if velocity > ROBOT_MAX_VEL_MOTOR:
                return ROBOT_MAX_VEL_MOTOR
            else:
                return velocity
        else:
            if velocity < -ROBOT_MAX_VEL_MOTOR:
                return -ROBOT_MAX_VEL_MOTOR
            else:
                return velocity
    def Update_RealVel(self, uart_feedback = None):
        if type(uart_feedback) == type(None):
            return False
        # Just in case: Convert uart rx feedback to string:
        text = str(uart_feedback)
        # Split text by "=", this should create 2 element table:
        splitted_text = text.split("=")
        # Just in case: Check if splitted text table actually have 2 elements:
        if not len(splitted_text) == 2:
            self.get_logger().info("[WARNING] RX Uart feedback bad!")
            return False

        if splitted_text[0] == "A1":
            self.Velocity["Real"]["Left"] = float(splitted_text[1])
        elif splitted_text[0] == "A2":
            self.Velocity["Real"]["Right"] = float(splitted_text[1])
        else:
            return False
        return True
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
