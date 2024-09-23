#this node is the Extended Kalman Filter node
#it will recive the current poistion state [x x_dot y x_dot]
# and will publish the estimated location

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import matplotlib.pyplot as plt

class ExtendedKalmanFilter(Node):
    def __init__(self):
        super().__init__("extended_kalman_filter")

        self.sub_ =self.create_subscription(Point, "/person_coords", self.callback , 1 )

        self.pub_ = self.create_publisher(Float32MultiArray , "/person_estimate" , 1)
        self.pub_point_ = self.create_publisher(Point , "/person_estimate_coor" , 1)


        self.state = np.zeros(4)  #State
        self.P = np.eye(4)        # Error
        self.Q = np.eye(4) * 0.1  # covariance of noise matrixS
        self.R = np.eye(4) * 0.1  # covariance of noise matrix
        
        self.F = np.eye(4)        # state transition matrix
        self.H = np.array([[1, 0, 0, 0],
                           [0 ,0 ,1 ,0],
                           [0 ,1, 0, 0],  # measurement matrix
                           [0, 0, 0, 1]])  # measurement matrix

        self.prev_time = time.time()
        self.prev_position_x = 0.0
        self.prev_position_y = 0.0

        self.mse_list = []
        self.std_dev_list = []
        self.time_list = []



    def callback(self , msg : Point):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        self.F[0, 2] = dt
        self.F[1, 3] = dt

        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q

        x_dot = (msg.x - self.prev_position_x) / dt
        y_dot = (msg.y - self.prev_position_y) / dt



        measurement = np.array([msg.x, x_dot, msg.y, y_dot])
        self.update(measurement)

        # Calculate error
        error = measurement - self.state[:4]
        mse = np.mean(error**2)/100
        std_dev = np.std(error)
        self.mse_list.append(mse)
        self.std_dev_list.append(std_dev)
        if not self.time_list:
            self.time_list.append(0)  # Start from time 0
        else:
            self.time_list.append(self.time_list[-1] + dt)


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

    def plot_results(self):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time_list, self.mse_list, label='MSE')
        plt.plot(self.time_list, self.std_dev_list, label='Standard Deviation', linestyle='--')
        plt.xlabel('Time (s)')
        plt.ylabel('Error')
        plt.title('MSE and Standard Deviation of Estimation Error Over Time')
        plt.legend()
        plt.grid(True)
        plt.show()
        print('MSE:', self.mse_list[-1])
        print('Standard Deviation:', self.std_dev_list[-1])


        
def main(args=None):
    rclpy.init(args=args)
    extended_kalman_filter = ExtendedKalmanFilter()
    try:
        rclpy.spin(extended_kalman_filter)
    except KeyboardInterrupt:
        extended_kalman_filter.plot_results()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
