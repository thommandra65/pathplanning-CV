import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
from .bot_pathplanning import bot_pathplanner
from .bot_motionplanning import bot_motionplanner
from nav_msgs.msg import Odometry
import numpy as np
import os

class maze_solver(Node):
    def __init__(self):
        super().__init__('maze_solving_node')
        self.subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.2  # seconds
        self.msg = Twist()
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.bot_pathplanner = bot_pathplanner()
        self.bot_motionplanner = bot_motionplanner()

        self.bot_subscriber = self.create_subscription(Image,'/Botcamera/image_raw',self.process_data_bot,10)

        self.pose_subscriber = self.create_subscription(Odometry,'/odom',self.bot_motionplanner.get_pose,10)

        self.timer = self.create_timer(timer_period, self.maze_solving)
        self.sat_view = np.zeros((100,100))
        self.bridge = CvBridge()

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.sat_view = frame
        cv2.imshow("sat_view", self.sat_view)
        #cv2.waitKey(1)

    def process_data_bot(self, data):
        self.bot_view = self.bridge.imgmsg_to_cv2(data,'bgr8') # performing conversion

    def maze_solving(self):
        frame_disp = self.sat_view.copy()
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)
        self.bot_mapper.graphify(self.bot_localizer.maze_og)
        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="dijisktra")
        self.bot_pathplanner.find_path_nd_display(self.bot_mapper.Graph.graph, start, end, maze,method="a_star")
        print("\nNodes Visited [Dijisktra V A-Star*] = [ {} V {} ]".format(self.bot_pathplanner.dijisktra.dijiktra_nodes_visited,self.bot_pathplanner.astar.astar_nodes_visited))

        # [Stage 4: MotionPlanning] Reach the (maze exit) by navigating the path previously computed
        bot_loc = self.bot_localizer.loc_car
        path = self.bot_pathplanner.path_to_goal
        self.bot_motionplanner.nav_path(bot_loc, path, self.msg, self.velocity_publisher)

        # Displaying bot solving maze  (Live)
        img_shortest_path = self.bot_pathplanner.img_shortest_path
        self.bot_motionplanner.display_control_mechanism_in_action(bot_loc, path, img_shortest_path, self.bot_localizer, frame_disp)

        # View bot view on left to frame Display
        bot_view = cv2.resize(self.bot_view, (int(frame_disp.shape[0]/3),int(frame_disp.shape[1]/2)))
        frame_disp[0:bot_view.shape[0],0:bot_view.shape[1]] = bot_view
        
        cv2.imshow("Maze (Live)", frame_disp)
        cv2.waitKey(1)  

def main(args=None):
        rclpy.init(args=args)
        node_obj = maze_solver()
        rclpy.spin(node_obj)
        rclpy.shutdown()

if __name__ == '__main__':
        main()