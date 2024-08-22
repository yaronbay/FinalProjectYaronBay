#this file is a driver connected to the yolo_depth node\
#this node / driver publishes the TF and sends commands to the ardunino 

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
import numpy as np
import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0',  baudrate=115200, timeout=.1)

def euler_to_quaternion(yaw, pitch, roll):
        """Convert given Euler angles in radians to a quaternion."""
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]


class CamAngle_driver(Node):    # create node that detect yolo

    def __init__(self):
       super().__init__("cam_angle_driver_node") 
       self.broadcaster = TransformBroadcaster(self)

       #self.timer = self.create_timer(1, self.broadcaster_callback)
       
       self.angle_sub = self.create_subscription(Float64 , "/Camera_Angle",self.broadcaster_callback, 1 )  #create subcriber that retures information to the camera



    def broadcaster_callback(self, angle: Float64):

        angle_rad = np.radians(angle.data)
        q = euler_to_quaternion(angle_rad , 0 , 0)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'camera_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
      
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)

        #from here moving the arduino
        a=angle.data

        # if a<0 or a>180:
        #      a= 90

        servo_signal=str(a)

        # servo_signal="0" 
        
        # if angle.data < -60 :#move left when pixel is in leftest quadrent
        #   servo_signal= "1"

        # elif angle.data > 60: #move right when pixel is in the rigtest quadrent
        #   servo_signal ="-1"

        # else  :
        #   servo_signal="0" #in every other senario stay steel
        #time.sleep(0.15)
        arduino.write(bytes(servo_signal,  'utf-8')) #send signal to arduino
        #time.sleep(0.15)
        print("motor position:", servo_signal) 
        servo_data = arduino.readline()
               




def main(args=None):
    rclpy.init(args=args)
    node=CamAngle_driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()