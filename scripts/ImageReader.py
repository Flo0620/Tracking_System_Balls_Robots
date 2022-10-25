"""
This script can be used to record images from the camera and saves them in a directory. Can be used i.e. to record a dataset.
"""

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('imageReaderSub')
        self.subscription = self.create_subscription(
            Image,
            '/ceiling_cam/ceiling_cam_publisher/image_color_rect',
            self.listener_callback,
            #self.listener_callback_showImage,   #shows image instead of saving it
            10)

        self.subscription
        self.br = CvBridge()
        self.counter = 0
        self.timerCounter =0

    def get_number(self,count):
        end = str(count)
        string=""
        while count/10000<1:
            string=string+"0"
            count = count*10
        return string+end

    def listener_callback(self, msg):
        self.counter +=1
        print("receiving %d", self.counter)
        img=self.br.imgmsg_to_cv2(msg)
        cv2.imwrite("/homes/19schleid/Desktop/calibrationImages/img"+str(self.get_number(self.counter))+".png",img) #Path were the images are saved
        time.sleep(1)  #can be used to wait between saving an image for instance to ensure that the images differ while recording a dataset

    def listener_callback_showImage(self, msg):
        self.counter +=1
        print("receiving %d", self.counter)
        img=self.br.imgmsg_to_cv2(msg)
        img=cv2.resize(img,(1280,920))
        cv2.imshow('img',img)
        cv2.waitKey(1)

 
    



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()