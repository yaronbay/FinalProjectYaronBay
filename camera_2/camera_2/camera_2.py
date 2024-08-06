#!/usr/bin/env python

#this node detectes blue balls with calor filter

# convert ros image to cv2 image -> convert RGB image to HSV image -> define color range in HSV ->
# -> threshhold pixels within range -> find counters in thresholded picture -> create bounderies rectangeles around the balls ->
#-> return to ros image

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Cam_2Node(Node):    # create node that detect circales by color
     
     def __init__(self):
       super().__init__("camera_2") 
       center=[0.0,0.0]
       self.image_pub = self.create_publisher(Image,"/image_raw2", 1 )       #create publisher to the picture that coming fron the camera
   
       self.bridge = CvBridge()
       self.image_sub = self.create_subscription(Image,"/image_raw",self.callback, 1 )  #create buscriber that retures information to the camera
       

       self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 1 ) #create a publisher that cause movement -sends velocity

       self.declare_parameter('center',center)

       self.location_subscriber_=self.create_subscription(Pose, "/turtle1/pose",self.pose_callback, 1 )  #creat subsctiber that monitors location and pose
       

     def callback(self, image1: Image):
       try:
         image = self.bridge.imgmsg_to_cv2(image1, "bgr8")   #convert ros image into opencv image
       except CvBridgeError as e:
         print(e)
         
       hsv_image=cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  #Convert image from RGB to HSV
       low_bound_blue=np.array([105,50,50])    #define low boundery for blue color
       upper_bound_blue=np.array([135,255,255])     #define upper boundery for blue color
       mask= cv2.inRange(hsv_image,low_bound_blue , upper_bound_blue)   #check and thresholds any pixel whithin designated color range
       contours, _ =cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    #find external contours in threshold data
       (x_t, y_t, w_t, h_t)=(0, 0, 0, 0)


       for contour in contours:    #create a loop to create a rectangle around every blue ball
          (x, y, w, h)= cv2.boundingRect(contour)      #find bounderies for each contour
          #create to much rectangles i need to filter it or sort the right one - maybe based on size
          if (w*h)>(w_t*h_t):    #sort rectangles based on area size
            (x_t, y_t, w_t, h_t)=(x, y, w, h) #remember the last coorinates data
            (x_tc, y_tc)=((x_t*2+w_t)/2,(y_t*2+h_t)/2 )  #find center coodinates of rectangle
            param=self.get_parameter('center').get_parameter_value().double_array_value           
            rec_center=[x_tc,y_tc]
            new_param= rclpy.Parameter('center', rclpy.Parameter.Type.DOUBLE_ARRAY,rec_center)
            all_new_parameters=[new_param]
            self.set_parameters(all_new_parameters)

       cv2.rectangle(image, (x_t, y_t), (x_t+w_t , y_t+h_t), (0,255,0),2)
       #cv2.line(image, (x_tc, y_tc), (x_tc, y_tc), (0,255,0), 2)
   
       try:
         self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))  #return to ros image
       except CvBridgeError as e:
         print(e)

         

     def pose_callback(self, pose: Pose):
        r=0.1     #convergence radius
        cmd=Twist()
        width_r= 640/11.088889122009277 #width ratio between ros image scale and turtlesim scale
        height_r=480/11.088889122009277 #height ratio between ros image scale and turtlesim scale
        middle= 5.544445
        param=self.get_parameter('center').get_parameter_value()._double_array_value
        x=param[0]/width_r
        y=param[1]/height_r
        c_dis_y=abs(y-middle)

        if y-middle > 0:
          y=y-2*c_dis_y
        else:
          y=y+2*c_dis_y
        length= np.sqrt((x-pose.x)**2+(y-pose.y)**2) #finds distace every time second
        del_x=x-pose.x
        del_y=y-pose.y
        theta_r=np.arctan2(del_y,del_x)      #finds angle from current loaction


        if length > r:   #if current distance is bigger than convegence radius start a oparate
           
           if abs ( pose.theta - theta_r) > 0.1: #while Delta angle is very very small
             
              if pose.theta <  theta_r :
                cmd.angular.z= 1.0
              else:
                 cmd.angular.z= -1.0

           else:
              cmd.linear.x = 1.5
                 
        else:
           pass
        print (x , y, theta_r ,pose.theta)
        self.cmd_vel_pub_.publish(cmd)

         
   
def main(args=None):
    rclpy.init(args=args)
    node=Cam_2Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()
