#this node is the Extended Kalman Filter node
#it will recive the current poistion state [x x_dot y x_dot]
# and will publish the estimated location

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("extended_kalman_filter")

        self.sub_ =self.create_subscription(Point, "/person_coords", self.callback , 1 )

        self.pub_ = self.create_publisher(Float32MultiArray , "/person_estimate" , 1)
        self.pub_point_ = self.create_publisher(Point , "/person_estimate_coor" , 1)


        self.state = np.zeros(4)  #State
        self.P = np.eye(4)        # Error
        self.Q = np.eye(4) * 0.1  # covariance of noise matrix
        self.R = np.eye(2) * 0.1  # covariance of noise matrix
        
        self.F = np.eye(4)        # state transition matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]])  # measurement matrix

        self.prev_time = time.time()



    def callback(self , msg : Point):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        self.F[0, 2] = dt
        self.F[1, 3] = dt

        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

        measurement = np.array([msg.x, msg.y])
        self.update(measurement)


        estimate_msg = Float32MultiArray()
        estimate_msg.data = self.state.tolist()
        self.pub_.publish(estimate_msg)

        xy_point = Point()
        xy_point.x = estimate_msg.data[0]
        xy_point.y = estimate_msg.data[1]
        xy_point.z = 0.0
        self.pub_point_.publish(xy_point)



    def update(self, measurement):
        z = measurement
        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        self.state = self.state + K @ y
        self.P = (np.eye(len(self.state)) - K @ self.H) @ self.P


        
def main(args=None):
    rclpy.init(args=args)
    extended_kalman_filter=ExtendedKalmanFilter()
    rclpy.spin(extended_kalman_filter)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
