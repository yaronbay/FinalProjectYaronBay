#this node is the Extended Kalman Filter node
#it will recive the current poistion state [x x_dot y x_dot]
# and will publish the estimated location

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("extended_kalman_filter")

        self.sub_ =self.create_subscription(Point, "/person_coords", self.callback , 1 )
        self.pub_ = self.create_publisher(Float32MultiArray , "/person_estimate" , 1)



    def callback(self , coord : Point):
        x=0
        

def main(args=None):
    rclpy.init(args=args)
    extended_kalman_filter=ExtendedKalmanFilter()
    rclpy.spin(extended_kalman_filter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
