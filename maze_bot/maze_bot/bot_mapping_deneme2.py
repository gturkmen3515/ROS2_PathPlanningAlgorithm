'''
> Purpose :
Node to perform the actual (worthy of your time) task of maze solving ;) 
- Robot velocity interface
- Upper Video camera as well

> Usage :
You need to write below command in terminal where your pacakge is sourced
- ros2 run maze_bot maze_solver

Note : Name of the node is actually name of executable file described in setup.py file of our package and not the name of python file

> Inputs:
This node is subscribing video feed from (Satellite or DroneCam)

> Outputs:
This node publishes on topic "/cmd_vel" , the required velocity ( linear and angular ) to move the robot

Author :
Haider Abbasi

Date :
18/03/22
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray,MultiArrayDimension    
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from math import pi,cos,sin
import matplotlib.pyplot as plt
from .bot_localization import bot_localizer
from .bot_mapping import bot_mapper
#from .bot_pathplanning import bot_pathplanner
#from .bot_motionplanning import bot_motionplanner

from nav_msgs.msg import Odometry

import numpy as np
from numpy import interp

from .utilities import Debugging
from . import config
gridx=[]
gridy=[]

class maze_solver(Node):

    def __init__(self):
        
        super().__init__("maze_solving_node")
        
        self.obx_publisher = self.create_publisher(Float64MultiArray,'/obsx',10)
        self.oby_publisher = self.create_publisher(Float64MultiArray,'/obsy',10)
        self.gridsize_publisher = self.create_publisher(Float64MultiArray,'/gridsize',10)
        self.loccation_publisher = self.create_publisher(Float64MultiArray,'/loccation',10)


        self.videofeed_subscriber = self.create_subscription(Image,'/upper_camera/image_raw',self.get_video_feed_cb,10)
        self.timer = self.create_timer(0.2, self.maze_solving)
        self.bridge = CvBridge()
        self.vel_msg = Twist()
        # Creating objects for each stage of the robot navigation
        self.bot_localizer = bot_localizer()
        self.bot_mapper = bot_mapper()
        self.sat_view = np.zeros((100,100))
        self.debugging = Debugging()

    def get_video_feed_cb(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        # scale_percent = 40 # percent of original size
        # width = int(frame_b .shape[1] * scale_percent / 100)
        # height = int(frame_b .shape[0] * scale_percent / 100)
        # dim = (width, height)
        # frame = cv2.resize(frame_b , dim, interpolation = cv2.INTER_AREA)
        self.sat_view = frame
    
    def overlay(self,image,overlay_img):

        gray = cv2.cvtColor(overlay_img, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
        mask_inv = cv2.bitwise_not(mask)

        roi = image
        img2 = overlay_img
        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        image = img1_bg + img2_fg
        return image

    # Overlay detected regions (User-specified-amount) over the frame_disp
    def overlay_cropped(self,frame_disp,image_rot,crop_loc_row,crop_loc_col,overlay_cols):
        
        image_rot_cols = image_rot.shape[1]
        gray = cv2.cvtColor(image_rot[:,image_rot_cols-overlay_cols:image_rot_cols], cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(gray, 5, 255, cv2.THRESH_BINARY)[1]
        mask_inv = cv2.bitwise_not(mask)

        frame_overlay_cols = crop_loc_col + image_rot_cols
        roi = frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols]            
        img2 = image_rot[:,image_rot_cols-overlay_cols:image_rot_cols]

        # Now black-out the area of logo in ROI
        img1_bg = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # Take only region of logo from logo image.
        img2_fg = cv2.bitwise_and(img2,img2,mask = mask)
        
        frame_disp[crop_loc_row:crop_loc_row + image_rot.shape[0],frame_overlay_cols-overlay_cols:frame_overlay_cols] = img1_bg + img2_fg


    def overlay_live(self,frame_disp,overlay,overlay_map,overlay_path):

        overlay_rot = cv2.rotate(overlay, cv2.ROTATE_90_CLOCKWISE)
        map_rot = cv2.rotate(overlay_map, cv2.ROTATE_90_CLOCKWISE)
        image_rot = cv2.rotate(overlay_path, cv2.ROTATE_90_CLOCKWISE)

        crop_loc_col = self.bot_localizer.transform_arr[0]+self.bot_mapper.crp_amt
        crop_loc_endCol = self.bot_localizer.transform_arr[0]+self.bot_localizer.transform_arr[2]+self.bot_mapper.crp_amt
        crop_loc_row = self.bot_localizer.transform_arr[1]+self.bot_mapper.crp_amt

        new_cols = int(overlay_rot.shape[1]*config.debug_live_amount)
        new_path_cols = int(overlay_rot.shape[1]*config.debug_path_live_amount)
        new_map_cols = int(overlay_rot.shape[1]*config.debug_map_live_amount)


        frame_disp[crop_loc_row:crop_loc_row + overlay_rot.shape[0],crop_loc_col:crop_loc_col + new_cols] = overlay_rot[:,0:new_cols]
        
        if config.debug_map_live_amount>0:
            self.overlay_cropped(frame_disp,map_rot,crop_loc_row,crop_loc_col,new_map_cols)
        if config.debug_path_live_amount>0:
            self.overlay_cropped(frame_disp,image_rot,crop_loc_row,crop_loc_col,new_path_cols)

    def maze_solving(self):
        
        frame_disp = self.sat_view.copy()
              
        self.bot_localizer.localize_bot(self.sat_view, frame_disp)
        self.bot_mapper.graphify(self.bot_localizer.maze_og)

        start = self.bot_mapper.Graph.start
        end = self.bot_mapper.Graph.end
        maze = self.bot_mapper.maze

        print("start x:",start[0],"start y:",start[1])
        print("end",end)

        # Draw & Display [For better Understanding of current robot state]
        center_frame_disp = int(frame_disp.shape[0]/2)

        o_oy=[]
        ox=np.where(self.bot_localizer.maze_og==0)[0]
        oy=np.where(self.bot_localizer.maze_og==0)[1]

        #print("Columns",np.shape(self.bot_localizer.maze_og)[0],"Rows",np.shape(self.bot_localizer.maze_og)[1])#471,354
 
        # #print("ox",ox)
        # print("oy shape",np.shape(oy))
        # print("oy type",type(oy))
        ox_v=[]
        oy_v=[]
        for i in range(0,len(ox),10):
            ox_v.append(ox[i])
        for i in range(0,len(oy),10):
            oy_v.append(oy[i])

        my_msg_ox = Float64MultiArray()
        my_msg_ox.data=[float(value1) for value1 in ox]
        self.obx_publisher.publish(my_msg_ox)
        
        my_msg_oy = Float64MultiArray()
        my_msg_oy.data=[float(value2) for value2 in oy]
        self.oby_publisher.publish(my_msg_oy)

        my_msg_size = Float64MultiArray()
        my_msg_size.data=[float(np.shape(self.bot_localizer.maze_og)[1]),float(np.shape(self.bot_localizer.maze_og)[0]),float(start[0]),float(start[1]),float(end[0]),float(end[1])]
        self.gridsize_publisher.publish(my_msg_size)


        #print("grid_x",datax)
        #print("grid_y",datay)
        #plt.figure(1)
        #plt.scatter(ox,oy)
        #plt.scatter(data[0,:],data[1,:])
        #plt.show()
        cv2.waitKey(1)

def main(args =None):
    rclpy.init()
    node_obj =maze_solver()
    rclpy.spin(node_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()