#!/usr/bin/env python

#this node detects object via yolov8 and a linear distance with a depth camera using rgbd topic

import rclpy
import numpy as np
import matplotlib.pyplot as plt
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, PointCloud2
from realsense2_camera_msgs.msg._rgbd import RGBD
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import serial
import open3d as o3d
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


arduino = serial.Serial(port='/dev/ttyACM0',  baudrate=115200, timeout=.1)


 
class Cam_YOLOdepth(Node):    # create node that detect yolo

    def __init__(self):
       super().__init__("yolo_depth_node") 

       self.model=YOLO('yolov8n.pt')    #create yolo nano model 
       self.model.to('cuda')

       self.bridge = CvBridge()         #define cvbrige
       self.current_angle = 90

       self.image_pub = self.create_publisher(Image,"/image_depth_detection", 1 )       #create publisher to the picture that coming fron the camera 

       self.angle_pub = self.create_publisher(Float64 , "/Camera_Angle", 1)      #create publisher that publishes the the angle the moter needs to move to

       self.center_pub_ = self.create_publisher(Point,"/person_coords" , 1 )

       self.image_sub = self.create_subscription(RGBD,"/camera/camera/rgbd",self.callback, 1 )  #create subcriber that retures information to the camera


       
    def callback(self, D_image: RGBD):
        
         rgb= D_image.rgb    #take only rbb part of the camera
         depth=  D_image.depth  #take only depth part
         cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")   #convert ros image into opencv image
         cv_depth=self.bridge.imgmsg_to_cv2(depth, "passthrough")   #convert depth part
         
        # pc1=self.bridge.imgmsg_to_cv2(pc, "passthrough")     #convert point cloud
         cv_depth =np.transpose(cv_depth)

         presults=self.model.predict(cv_image)   #create yolo model on each and every frame of usb_cam
         presult=presults[0]   #isolate only rquired arrgument

        #create point cloud for entire picture:
        #  fx = 500
        #  fy = 500
        #  cx = cv_depth.shape[1] / 2  # x coodrinates - middle of coordinate system
        #  cy = cv_depth.shape[0] /2 # y coordinates - middle of coordinate system
        #  print (range(cv_depth.shape[0]))

        #  point_cloud = []
        #  pc_depth = []

        #  for i in range(int(cv_depth.shape[0]/5)) : #create martix- take each fifth pixel
        #      for j in range(int(cv_depth.shape[1]/5)):
        #          depth_coor= cv_depth[5*i , 5*j]    #find depth for each coordinate in each matrix
        #          if depth_coor > 0:  #ignore invalid depth values
        #              x = (5*j - cx)*depth_coor / fx
        #              y = (5*i - cy)*depth_coor / fy
        #              z= depth_coor
        #              point_cloud.append ([x, y, z])
        #              pc_depth.append ([z])
        #  point_cloud= np.array(point_cloud) #convert point_cloud into a numpy array
        #  pc_depth = np.array(pc_depth)
    
         for box in presult.boxes:
                
             #extract information of images:
             class_num=round(box.cls[0].item())    #get class naumber
             class_name=presult.names[box.cls[0].item()]  #get class name
             cords = box.xyxy[0].tolist()    #get coordintes of the box x1,y1,x2,y2
             cords= [round(x) for x in cords]    #round the coordinates
             confidence=round(box.conf[0].item(),3)    #get confidance level 3 numbers after dot

             #find geometric center of rectangle:
             center = (cords[0]+cords[2])/2, (cords[1]+cords[3])/2 

             #find value at center            
             central_depth = cv_depth[round(center[0])][round(center[1])]

             text= class_name + str(confidence)    #conbine class name and confidance level
             
             #print infomation:
            
            #  print("class number and type:",class_num,",", class_name," , " ," depth: ", central_depth)
            #  print ("cordinates:" ,cords)
            #  print("center:" , center)
            #  print("confidence:", confidence)

             if class_num ==0 : #if finds a person

                #create point cloud in the required rectangle:
                frame_center= rgb.width/2 , rgb.height/2
                pc_depth = []
                pc_total = []

#                 for i in range(int(cords[2]-cords[0])):
#                     for j in range(int(cords[3] - cords[1])):
#                      pixel_dist_from_frame_center= np.sqrt((frame_center[0]- int(cords[0]) + i - 1)**2 + (frame_center[1] - int(cords[1]) + j -1)**2)
#                      depth_coor = cv_depth[int(cords[0]) + i - 1 , int(cords[1]) + j - 1]
#                      if depth_coor > 0: # and depth_coor > pixel_dist_from_frame_center: #ignore depths below 0
#                          x=int(cords[0]) + i - 1
#                          y=int(cords[1]) + j - 1
#                          z= np.sqrt(depth_coor**2 - pixel_dist_from_frame_center**2)
#                          #pc_depth.append([z]) # only depths of aray
#                          pc_total.append([x , y , z])                       
# #                pc_depth = np.array(pc_depth) #orgenized pc inside an array

                for i in range(int(cords[2] - cords[0])):
                    for j in range(int(cords[3] - cords[1])):
                        pixel_dist_from_frame_center = np.sqrt((frame_center[0] - int(cords[0]) + i - 1)**2 + (frame_center[1] - int(cords[1]) + j - 1)**2)
                        depth_coor = cv_depth[int(cords[0]) + i - 1, int(cords[1]) + j - 1]
                        
                        if depth_coor > 0:  # Ignore invalid depth values
                            x = int(cords[0]) + i - 1
                            y = int(cords[1]) + j - 1
                            value_inside_sqrt = depth_coor**2 - pixel_dist_from_frame_center**2
                            
                            if value_inside_sqrt >= 0:
                                z = np.sqrt(value_inside_sqrt)
                            else:
                                z = 0  # or continue to skip this point if negative value

                            pc_total.append([x, y, z])



                pc_total = np.array(pc_total)

                #time.sleep(0.2)

                pcd= o3d.geometry.PointCloud()  # create point cloud class 
                pcd.points=o3d.utility.Vector3dVector(pc_total.reshape(-1,3))
                #pcd.points=o3d.utility.Vector3dVector(pc_total)        
                downpcd=pcd.voxel_down_sample(voxel_size=10.0)   #filter the ammount of pixels that are relevant to point cloud 

                #prepere data for DBSCAN
                x_y_z_converted= np.asarray(downpcd.points)
                nan_mask = np.isnan(x_y_z_converted)
                x_y_z_converted[nan_mask]=-1

                noramalized_points = StandardScaler().fit_transform(x_y_z_converted)

                #after filtering of the pointcloud a DBSCAN clustering is the nest step:
                db=DBSCAN(eps=15.0, min_samples=10)
                labels=db.fit_predict(x_y_z_converted)

                unique_labels = np.unique(labels)
                colors = plt.cm.viridis(np.linspace(0, 1 ,len(unique_labels))) #give color to each label

                #find the biggest cluster:
                cluster_sizes= {}


                for label in unique_labels:
                   if label != -1: #take out noises
                        cluster_size = np.sum(labels == label)
                        cluster_sizes[label] = cluster_size

                #the next if else is in case that cluster_sizes is empty
                if cluster_sizes:
                   
                    largest_cluster_label= max(cluster_sizes, key = cluster_sizes.get) #biggest label number

                else:
                   
                   largest_cluster_label = 0
                
                labels_mask = (labels == largest_cluster_label) #create mask on label number
                indices = np.where(labels_mask) #find required indices of mask
                labels_coordiantes = np.asarray(list(x_y_z_converted[indices])) #create array of only biggest cluster label
                
                center_of_mass= np.sum(labels_coordiantes, axis = 0) / np.size(indices)



                pose=Float64()
                
                if np.isnan(center_of_mass).any():
                   continue # Skip the iteration if any part of center_of_mass is NaN

                else:

                    #find angle on the xz plane:
                 dx = 320-center_of_mass [0]
                 XZ_angle = np.rad2deg(np.arctan2(dx,center_of_mass[2] )) #get raw angle
                 person_rel_angle = -XZ_angle + 90         # get angle relative to the to current coordinates systems 
                 angle_movement = person_rel_angle - 90    # how much movement was made sience the last iteration
                 new_angle = self.current_angle +angle_movement #get new angle relative to the world

                 if (new_angle > 180 or new_angle < 0):  # set to beggining if surpassed limits
                      self.current_angle = 90
                      pose.data = 90
                      self.angle_pub.publish(pose)


                 elif abs(new_angle-self.current_angle) > 5:  # define standat deviation for movement
                
                      
                      self.current_angle = new_angle                 # reset global integer current_angle
                 
                 #self.current_angle = person_rel_angle

          
                 #pose.data = -XZ_angle + 90
                 #pose.data = person_rel_angle
                      pose.data = new_angle
                 #if new_angle > 0 and new_angle < 180:
                  #pose.data = self.current_angle
                 #self.angle_pub.publish(pose)

                      self.angle_pub.publish(pose)
                      

                  #time.sleep(0.2)

                 com_ = Point()
                 com_.x = center_of_mass[0]
                 com_.y = center_of_mass[1]
                 com_.z = center_of_mass[2]
                 self.center_pub_.publish(com_)

                 # draw rectangle:
                 cv2.rectangle(cv_image,(cords[0],cords[1]),(cords[2],cords[3]),(0,255,0),2)
                 cv2.putText(cv_image, text,(cords[0],cords[1]), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,255),1,cv2.LINE_AA)             
                 #draw information:
                 cv2.circle(cv_image,(round(center_of_mass[0]),round(center_of_mass[1])),3,(0,0,255),1)
                 cv2.putText(cv_image, str(central_depth),(round(center_of_mass[0]),round(center_of_mass[1])), cv2.FONT_HERSHEY_SIMPLEX, 1,(0,0,255),1,cv2.LINE_AA)
                              
                   

                 self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))  #return to ros image


def main(args=None):
    rclpy.init(args=args)
    node=Cam_YOLOdepth()
    current_angle = 90
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()


 
    
