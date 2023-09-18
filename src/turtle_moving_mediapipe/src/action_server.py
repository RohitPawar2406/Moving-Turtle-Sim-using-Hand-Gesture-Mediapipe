#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from action_tutorials_interfaces.action import Fibo
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from rclpy.executors import MultiThreadedExecutor

from HandDetectorModule import HandDetector

class TurtleSimNode(Node):
    def __init__(self):
        super().__init__("Turtle_Sim_Node_for_Moving")
        self._action_server = ActionServer(
            self,
            Fibo,
            'TurtleSimAction',
            self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel",10)

        self.subsciber_handler = self.create_subscription(Int32, 'topic', self.subscript_callback, 10)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        result = Fibo.Result()
        digit  = 11
        self.get_logger().info(f'GOAL======...{type(digit)}...')
        msg = Twist()
        return result

    def subscript_callback(self, msg_received:Int32):
        self.get_logger().info('I heard: "%s"' % msg_received.data)
        digit_received = msg_received.data
        msg = Twist()
        
        if digit_received == -1:
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        elif digit_received == 0:
            msg.angular.z = -1.0

        elif digit_received == 1:
            msg.linear.x = 1.0
            msg.angular.z = 0.0

        elif digit_received == 2:
            msg.linear.y = 1.0
            msg.angular.z = 0.0

        elif digit_received == 3:
            msg.linear.y = -1.0
            msg.angular.z = 0.0

        elif digit_received == 4:
            msg.linear.x = -1.0
            msg.angular.z = 0.0

        elif digit_received == 5:
            msg.angular.z = 1.0

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    # Write code here
    turtle_action_server = TurtleSimNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(turtle_action_server)
    executor.spin()
    rclpy.shutdown()

if __name__=='__main__':
    main()