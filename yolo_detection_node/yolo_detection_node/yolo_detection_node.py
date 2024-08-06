#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO


class Cam_detectNode(Node):    # create node that detect yolo

    def __init__(self):
       super().__init__("yolo_det_node") 

       self.model=YOLO('yolov8n.pt')    #create yolo nano model 
       self.bridge = CvBridge()         #define cvbrige
       self.image_pub = self.create_publisher(Image,"/image_detection", 1 )       #create publisher to the picture that coming fron the camera 
       self.image_sub = self.create_subscription(Image,"/image_raw",self.callback, 1 )  #create subcriber that retures information to the camera

    def callback(self, image1: Image):
         
         image = self.bridge.imgmsg_to_cv2(image1, "bgr8")   #convert ros image into opencv image

         presults=self.model.predict(image)   #create yolo model on each and every frame of usb_cam
         presult=presults[0]   #isolate only rquired arrgument

         for box in presult.boxes:
                
                #extract information of images:
            class_num=round(box.cls[0].item())
            class_name=presult.names[box.cls[0].item()]  
            cords = box.xyxy[0].tolist()
            cords= [round(x) for x in cords]
            confidence=round(box.conf[0].item(),3)
            text= class_name + str(confidence)
                #print infomation and draw a rectangle
            print("class number and type:",class_num,",", class_name)
            print ("cordinates:" ,cords)
            print("confidence", confidence)
            cv2.rectangle(image,(cords[0],cords[1]),(cords[2],cords[3]),(0,255,0),2)
            cv2.putText(image, text,(cords[0],cords[1]), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,255),1,cv2.LINE_AA)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))  #return to ros image

#
#  next part is to check about calman filter- find out what it does
#

def main(args=None):
    rclpy.init(args=args)
    node=Cam_detectNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


 
    
