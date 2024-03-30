import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import sys
import math

class robot_go_goal(Node):

    def __init__(self):
        super().__init__('goal_movement_node')
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Odometry,'/odom',self.pose_callback,10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.go_to_goal_function)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.vel_msg = Twist()
        self.angle_to_goal = None 
        self.distance_to_goal = None

    def pose_callback(self, data):
        self.robot_pose.x = float(data.pose.pose.position.x)
        self.robot_pose.y = float(data.pose.pose.position.y)
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        (_, _, yaw) = self.euler_from_quaternion(*quaternion)
        self.robot_pose.z = yaw

    def go_to_goal_function(self):
        if self.goal_pose.x is None or self.goal_pose.y is None:
            return

        self.angle_offset = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0

        self.distance_to_goal = math.sqrt(
            (self.goal_pose.x - self.robot_pose.x) ** 2 +
            (self.goal_pose.y - self.robot_pose.y) ** 2
        )
        self.angle_to_goal = math.atan2(self.goal_pose.y - self.robot_pose.y, self.goal_pose.x - self.robot_pose.x) + self.angle_offset
        angle_to_turn = self.angle_to_goal - self.robot_pose.z

        if abs(angle_to_turn) > 0.1:
            self.vel_msg.angular.z = angle_to_turn * 0.5  # Adjust turning speed
            self.vel_msg.linear.x = 0.0
        else:
            self.vel_msg.linear.x = min(self.distance_to_goal, 0.5)  # Adjust maximum linear speed

        msg = 'Distance to goal: {:.3f}, Angle to goal: {:.3f}'.format(self.distance_to_goal, self.angle_to_goal)
        self.get_logger().info(msg)
        self.velocity_pub.publish(self.vel_msg)

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

def main(args=None):
    rclpy.init(args=args)

    gtg_node = robot_go_goal()

    # Set the goal position
    gtg_node.goal_pose.x = float(1)
    gtg_node.goal_pose.y = float(1)

    # Spin until the robot reaches the goal
    while rclpy.ok() and gtg_node.distance_to_goal is None or gtg_node.distance_to_goal > 0.01:
        rclpy.spin_once(gtg_node, timeout_sec=1)


    # Stop the robot
    gtg_node.vel_msg.linear.x = 0
    gtg_node.vel_msg.angular.z = 0
    gtg_node.velocity_pub.publish(gtg_node.vel_msg)

    # Destroy the node explicitly
    gtg_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
