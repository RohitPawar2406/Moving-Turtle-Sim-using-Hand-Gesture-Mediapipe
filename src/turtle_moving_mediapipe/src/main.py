#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.action import  ActionClient
from action_tutorials_interfaces.action import Fibo
from std_msgs.msg import Int32

import cv2
import time

from HandDetectorModule import HandDetector


class DigitsDetectionNode(Node):
    def __init__(self):
        super().__init__("Detecting_Digits_MediaPipe")
        self.data = -1
        self.msg = "Not Moving"
        # Intilise action client
        self._action_client = ActionClient(self, Fibo,'TurtleSimAction')

        # Publishing
        self.publish_handler = self.create_publisher(Int32, 'topic', 10)

        hand_object = HandDetector()
        cap = cv2.VideoCapture(0)
        tipIds = [4, 8, 12, 16, 20]
        # FPS pre-define variables
        pTime=cTime=0
        while cap.isOpened():
            success,img = cap.read()
            img = hand_object.findHands(img)
            lmlists= hand_object.findPosition(img)
            self.data = -1
            if len(lmlists) != 0:
                fingers=[]
                # Thumb
                if lmlists[tipIds[0]][1] > lmlists[tipIds[0] - 1][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)
                # Checking Fingers If they are Up or Down
                for id in range(1,5):
                    if lmlists[tipIds[id]][2] < lmlists[tipIds[id]-2][2]:
                        fingers.append(1)
                    else:
                        fingers.append(0)
            
                #print(fingers)
                
                cv2.putText(img,f'{fingers.count(1)}',(40,150),cv2.FONT_HERSHEY_COMPLEX,2,(0,255,0),3)
                self.data = fingers.count(1)
                #self.get_logger().info(f'DIGITS RECEVIED: {self.data}')
                print(f"$$$$$$$$$  {type(self.data)}")
                
                if self.data == -1:
                    self.msg = self.msg
                elif self.data == 0:
                    self.msg = "Rotating Clockwise"

                elif self.data == 1:
                    self.msg = "X axis Positive"

                elif self.data == 2:
                    self.msg = "Y axis Positive"

                elif self.data == 3:
                    self.msg = "Y axis Negative"

                elif self.data == 4:
                    self.msg = "X axis Negative"

                elif self.data == 5:
                    self.msg = "Rotating Anti-Clockwise"
                
            cv2.putText(img,f'{self.msg}',(320,50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,255),2)
            # FPS 
            cTime=time.time()
            fps = 1/(cTime-pTime)
            pTime=cTime
        
            cv2.putText(img,f'FPS:{int(fps)}',(40,70),cv2.FONT_HERSHEY_COMPLEX,2,(255,0,0),3)
            self.get_logger().info('Goal accepted :)')
            cv2.imshow("Image",img)

            # ROS CODE
            self.get_logger().info('Waiting for action server...')
            self._action_client.wait_for_server()

            goal_msg = Fibo.Goal()
            goal_msg.digit_detected = int(self.data)

            msg = Int32()
            msg.data = int(self.data)
            self.publish_handler.publish(msg) 

            self.get_logger().info(f'Sending goal request... {goal_msg}')


            self._action_client.send_goal_async(goal_msg)
            ################
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    # Write code here
    node  = DigitsDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()