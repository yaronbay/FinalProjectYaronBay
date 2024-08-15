#this node is subscribed to the persons location and publishes its velocity

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import time

class GetVelocity(Node):
    def __init__(self):
        super().__init__("get_velocity")

        self.point_sub = self.create_subscription(Point, "/person_coords" ,self.position_callback, 1)

        self.vel_coor_pub = self.create_publisher(Float32MultiArray, "/person_vel" , 1)

        self.prev_position = None

        self.prev_time = None



    def position_callback(self, coor_msg : Point ):
        
        current_time =time.time()

        if self.prev_position is not None and self.prev_time is not None:

            dt = current_time - self.prev_time

            dx = (coor_msg.x -self.prev_position.x) * 0.01
            dy = (coor_msg.y - self.prev_position.y) * 0.01

            vx = dx/dt
            vy = dy/dt

            velocity_and_coors = Float32MultiArray()
            velocity_and_coors.data = [coor_msg.x, vx, coor_msg.y, vy]
            self.vel_coor_pub.publish(velocity_and_coors)

        self.prev_position = coor_msg
        self. prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    get_velocity = GetVelocity()
    rclpy.spin(get_velocity)
    get_velocity.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
