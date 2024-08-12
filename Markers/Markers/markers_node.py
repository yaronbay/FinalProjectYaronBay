#!/usr/bin/env python

import rclpy
from rclpy import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
def main():
    print('Hi from Markers.')


class MarkerPublisher(Node):

    def __init__(self):
        super().__init__("marker_publisher")

        self.marker_pub_ = self.create_publisher(Marker , "/person marker" , 1 )
        
        self.coordinates = self.create_subscription(Point, "/person_coords", self.marker_callback , 1)



    def marker_callback(self, coordinates: Point):
        x_coor = coordinates.data[0]
        y_coor = coordinates.data[1]


        marker = Marker()

        marker.header.stamp =self.get_clock().now().to_msg()
        marker.header.frame_id = 'camera_link'
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x_coor
        marker.pose.position.y = y_coor
        marker.pose.position.z = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node=MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
