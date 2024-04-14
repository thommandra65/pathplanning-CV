import rclpy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
import math

class TurtleMove:
    def __init__(self):
        self.node = rclpy.create_node('turtle_move')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.node.create_subscription(Odometry,'/odom',self.pose_callback,10)
        self.robot_pose = Point()
        self.goal_pose = Point()
        self.target_distance = 1.0
        self.speed = 1.0
        self.rate = 10
    
    def pose_callback(self, data):
        self.robot_pose.x = data.pose.pose.position.x
        self.robot_pose.y = data.pose.pose.position.y
    
    def move_forward(self):
        twist_msg = Twist()
        initial_x = self.robot_pose.x
        initial_y = self.robot_pose.y
        while True:
            twist_msg.linear.x = self.speed
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)
            distance_moved = math.sqrt((self.robot_pose.x - initial_x)**2 + (self.robot_pose.y - initial_y)**2)
            if distance_moved >= self.target_distance:
                break
            rclpy.spin_once(self.node, timeout_sec=1.0 / self.rate)
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
    
    def run(self):
        # Wait for the initial pose message to be received
        while self.robot_pose.x == 0.0 and self.robot_pose.y == 0.0:
            rclpy.spin_once(self.node)
        
        # Set the target position based on the current pose
        self.goal_pose.x = self.robot_pose.x + self.target_distance
        self.goal_pose.y = self.robot_pose.y
        
        # Move forward
        self.move_forward()
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    turtle_move = TurtleMove()
    turtle_move.run()

if __name__ == '__main__':
    main()
