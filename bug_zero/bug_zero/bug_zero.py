#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import math

V_MAX = 0.1 # m/s
OMEGA_MAX = 0.5*math.pi
OMEGA_OBSTACLE = 0.25*math.pi
MIN_DISTANCE = 0.4 #m
MAX_CLOSEST_DISTANCE = 0.25#m

class bug_rasiewf(Node):
    def __init__(self):
         super().__init__('bug_rasiewf')
         self.pub = self.create_publisher(Twist,'/cmd_vel', 1)
         self.sub1 = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.PoseGoal_callback, 10)
         self.sub2 = self.create_subscription(Odometry, '/odom', self.Odom_callback, 10)
         self.sub3 = self.create_subscription(LaserScan, '/scan', self.Laser_callback, 10)
         self.cmd_vel_msg = Twist()
         self.LaserScan_msg = LaserScan()
         self.GoalPose_msg = PoseStamped()
         self.OdomMsg = Odometry()
         timer_period = 0.1  # seconds
         self.timer = self.create_timer(timer_period, self.timer_callback)
         self.bug_algorithm = BUG_ALGORITHM(self)
         self.sub1
         self.sub2
         self.sub3
    def timer_callback(self):
        self.bug_algorithm.Process()
    def PoseGoal_callback(self,recieved_msg):
        self.GoalPose_msg = recieved_msg
    def Odom_callback(self,recieved_msg):
        self.OdomMsg = recieved_msg
    def Laser_callback(self,recieved_msg):
        self.LaserScan_msg = recieved_msg
    def Publish_CMDVEL(self,msg):
        self.pub.publish(msg)
    def Publish_ZeroCMDVEL(self):
        msg = Twist()
        msg.linear.x = 0.00
        msg.linear.y = 0.00
        msg.linear.z = 0.00

        msg.angular.x = 0.00
        msg.angular.y = 0.00
        msg.angular.z = 0.00
        self.pub.publish(msg)
        
class BUG_ALGORITHM():
    def __init__(self, bug = None):
        self.ROS_NODE = bug
        self.Goal = PoseStamped()
        self.ActualPosition = Odometry()
        self.cmd_vel = Twist()
        self.Laser = LaserScan()
        self.ObstacleDetected = False
        self.ObstacleDetectedRight = False
        self.ObstacleDetectedLeft = False
        self.ObstacleDetectedBack = False
        self.position_accuracy = 0.1
        self.iterations = 0
        self.iteration_max = 1000
    def Follow2Target(self):
        # Calculate position difference between goal point and robot point
        x = self.ActualPosition.pose.pose.position.x - self.Goal.pose.position.x
        y = self.ActualPosition.pose.pose.position.y - self.Goal.pose.position.y
        # angles:
        orientation_q = self.ActualPosition.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w]
        (_,_,yaw) = euler_from_quaternion(orientation_list)
        # norms:
        xy_norm = math.sqrt((x)**2+(y)**2)
        return x, y, xy_norm, yaw
    def FollowObstacle(self):
        ranges = self.Laser.ranges
        closest_distance = 1000
        closest_distance_right = 0
        point = [0,0]
        self.ObstacleDetected = False
        self.ObstacleDetectedRight = False
        closest_distance_left = 0
        self.ObstacleDetectedLeft = False
        closest_distance_back = 0
        self.ObstacleDetectedBack = False
        for i in range(len(ranges)):
            if ranges[i] > self.Laser.range_min and ranges[i] > self.Laser.range_min:
                    if ranges[i] < MIN_DISTANCE and ranges[i] < closest_distance:
                        angle = self.Laser.angle_min + i*self.Laser.angle_increment
                        if angle > math.radians(290) or angle < math.radians(50):
                            x = ranges[i]*math.cos(angle)
                            closest_distance = ranges[i]
                            y = closest_distance*math.sin(angle)
                            self.ObstacleDetected = True
                        elif angle < math.radians(290) or angle > math.radians(215):
                            closest_distance_right = ranges[i]
                            self.ObstacleDetectedRight = True
                        elif angle < math.radians(125) or angle > math.radians(50):
                            closest_distance_left = ranges[i]
                            self.ObstacleDetectedLeft = True
                        elif angle < math.radians(215) or angle > math.radians(125):
                            closest_distance_back = ranges[i]
                            self.ObstacleDetectedBack = True


        if self.ObstacleDetected:
            #Constructing cmd_vel msg:
            v_lin = V_MAX + 0.1
            if closest_distance < MAX_CLOSEST_DISTANCE:
                v_lin = -V_MAX - 0.1
            self.cmd_vel.linear.x = v_lin
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0

            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = OMEGA_OBSTACLE
        if self.ObstacleDetectedRight:
            #Constructing cmd_vel msg:
            v_lin = V_MAX + 0.1
            self.cmd_vel.linear.x = v_lin
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0

            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = 0
        if self.ObstacleDetectedLeft:
            #Constructing cmd_vel msg:
            v_lin = 0
            self.cmd_vel.linear.x = v_lin
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0

            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = OMEGA_OBSTACLE
        if self.ObstacleDetectedBack:
            #Constructing cmd_vel msg:
            v_lin = 0
            if closest_distance_back < 0.15:
                v_lin = V_MAX + 0.1
            self.cmd_vel.linear.x = v_lin
            self.cmd_vel.linear.y = 0
            self.cmd_vel.linear.z = 0

            self.cmd_vel.angular.x = 0
            self.cmd_vel.angular.y = 0
            self.cmd_vel.angular.z = -OMEGA_OBSTACLE
        return self.ObstacleDetected

    def Calculate_CMDVEL(self,x_pos,y_pos,yaw):
        theta_ref = math.atan2(-y_pos,-x_pos)
        error_theta = theta_ref - yaw
        if( error_theta > math.radians(180) ):
            error_theta -= math.radians(360)
        if( error_theta < -math.radians(180) ):
            error_theta += math.radians(360)
        v_ref = V_MAX
        omega_ref = error_theta
        omega_ref = min( max(omega_ref, -OMEGA_MAX), OMEGA_MAX)
        
        #Constructing cmd_vel msg:
        self.cmd_vel.linear.x = v_ref
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0

        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = omega_ref
    def Process(self):
        self.ActualPosition = self.ROS_NODE.OdomMsg
        self.Goal = self.ROS_NODE.GoalPose_msg
        self.Laser = self.ROS_NODE.LaserScan_msg
        if abs(self.Goal.pose.position.x) > 0.1 or abs(self.Goal.pose.position.y) > 0.1:
            cmd_x, cmd_y, cmd_norm, cmd_yaw = self.Follow2Target()
            if cmd_norm > self.position_accuracy and self.iterations < self.iteration_max:
                if not self.FollowObstacle():
                    self.Calculate_CMDVEL(cmd_x, cmd_y, cmd_yaw)
                    self.ROS_NODE.Publish_CMDVEL(msg=self.cmd_vel)
                else:
                    self.ROS_NODE.Publish_CMDVEL(msg=self.cmd_vel)
                self.iterations += 1
            else:
                self.iterations = 0
                self.ROS_NODE.Publish_ZeroCMDVEL()

def main(arg=None):
    rclpy.init(args=arg)
    ros_node = bug_rasiewf()
    rclpy.spin(ros_node)
    #when finished destroy node and delete objects:
    ros_node.destroy_node()
    del ros_node
    rclpy.shutdown()

if __name__ == '__main__':
    main()