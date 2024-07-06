#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from functools import partial
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import numpy as np
import cv2 as cv
import sensor_msgs_py.point_cloud2 as pc2

class TurtleControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("turtle_controller")
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
        self.image_subscribe = self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.image_callback, 10)
        self.pointCloud_subscirbe = self.create_subscription(PointCloud2, "/camera/points", self.pointCloud_callback, 10)
        self.get_logger().info("Turtle controller node has been started")

    def tf_callback(self, msg: TFMessage):
        if msg.transforms[0].header.frame_id == "odom": pass
            # self.get_logger().info("x: " + str(msg.transforms[0].transform.translation.x)\
            #                         +"\n" + "y: " + str(msg.transforms[0].transform.translation.y)\
            #                         +"\n" + "z: " + str(msg.transforms[0].transform.rotation.z)\
            #                         +"\n" + "w: " + str(msg.transforms[0].transform.rotation.w))

    def image_callback(self, msg: CompressedImage): pass
        # picTime = int(msg.header.stamp.sec)
        # self.get_logger().info("time: " + str(picTime))
        # if picTime % 10 == 0:
        #     image = np.asarray(bytearray(msg.data), dtype="uint8")
        #     self.imgDecode = cv.imdecode(image, cv.IMREAD_COLOR)
        #     cv.imwrite(f'/my_bot/src/my_robot_controller/my_robot_controller/temp/{picTime}.png', self.imgDecode)

    def pointCloud_callback(self, msg: PointCloud2):
        pointTime = int(msg.header.stamp.sec)
        self.get_logger().info("Time: " + str(pointTime))
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            self.get_logger().info(f'Point: {point}')

    def send_velocity_command(self):
        msg = Twist() # create a msg object from Twist() class
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()