#import darknet_images
#import darknet
from ast import arguments
from cmath import nan
import cv2
import random
import argparse
import os

from torch import classes
from pytorchyolo import detect
from pytorchyolo import models
from pytorchyolo.utils.utils import rescale_boxes, load_classes
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
import math
import ball_detector_config as cfg

class YoloWrapper(Node):

    def __init__(self):
        super().__init__('imageReaderSub')
        self.subscription = self.create_subscription(
            Image,
            '/ceiling_cam/ceiling_cam_publisher/image_color_rect',
            #'/ceiling_cam/ceiling_cam_publisher/image_color_debayered',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.max_balls = cfg.max_num_balls #Nur die max_balls besten detections werden als transforms gepublished. 
                           #Wenn mit nur einem Ball gespielt wird, wird immer die Detection mit der höchsten confidence verwendet.
                           #-1 wenn alle detections gepublished werden sollen.
        self.ball_radius = cfg.ball_radius
        self.conf_thresh = cfg.conf_thresh
        self.nms_thresh = cfg.nms_thresh
        #print(arguments.ball_radius)

        self.plane_computed = False #boolean wether the field plane has already been computed or not
        self.field_point =TransformStamped()
        self.field_normal = TransformStamped()

        self.tfbr = TransformBroadcaster(self)

        self.tfBuffer=Buffer()
        self.tf_listener = TransformListener(self.tfBuffer, self)


        self.cameraInfoSub = self.create_subscription(
            CameraInfo,
            '/ceiling_cam/ceiling_cam_publisher/camera_info',
            self.cam_info_listener_callback,
            10)

        #self.imgpub = self.create_publisher(Image, '/ceiling_cam/ceiling_cam_publisher/camera_image_rect2', 10)#publishes the Image again so that the apriltag detector can subscribe to this topic since two subscribers on the same image topic slow it down


        self.cx=-1  #camera_projection_matrix parameters are initialized with -1
        self.cy=-1
        self.fx=-1
        self.fy=-1

        #Initializing Yolo network
        random.seed(3)  # deterministic bbox colors
        dir_path=os.path.dirname(os.path.realpath(__file__))
        self.model = models.load_model(os.path.join(dir_path,'config_and_weights/yolov3-custom4.cfg'),os.path.join(dir_path,'config_and_weights/yolov3_aug_ceiling_cam1600_e200.pth'))
        self.frameCounter=0
        #self.startTime = time.time_ns()

        


    def listener_callback(self, msg):
        #self.imgpub.publish(msg)

        img_size = 832
        img=self.br.imgmsg_to_cv2(msg)

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        detections=detect.detect_image(self.model, img, img_size, conf_thres=self.conf_thresh, nms_thres=self.nms_thresh)

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        numBalls = detections.shape[0]

        detections = detections[detections[:,4].argsort()] #sortieren der Detections nach ihren confidence Werten

        for i in range(self.max_balls if self.max_balls != -1 and numBalls>self.max_balls else numBalls):
            detection = detections[i]
            left = int(detection[0])
            top = int(detection[1])
            right = int(detection[2])
            bottom = int(detection[3])
            confidence = detection[4]

            #create annotated image
            cv2.rectangle(img, (left, top), (right, bottom), (0,255,0), 1)
            cv2.putText(img, "{} [{:.2f}]".format("ball", float(confidence)),
                        (left, top - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0,255,0), 2)
            
            detcx = left+(right-left)/2
            detcy = top+(bottom-top)/2
            self.tfFromImgPoint(detcx,detcy,self.ball_radius,i)
            

        width=2048//2
        height=1536//2
        image_resized = cv2.resize(img, (width, height),
                               interpolation=cv2.INTER_LINEAR)
        #self.videoWriter.write(img)
        cv2.imshow('Inference', image_resized)

        #self.frameCounter += 1
        #print("fps: "+str(self.frameCounter/((time.time_ns()-self.startTime)/1e9)))
        cv2.waitKey(1)


    """
    given a 2D point in the image where the center of the ball is, the radius of the ball and the number of the ball,
    this function publishes a transform with the 3D position of the ball on the field in the ceiling_camera frame
    :param icx: the x coordinate of the ball in the image
    :param icy: the y coordinate of the ball in the image
    :param ball_radius: the radius of the ball
    :ball_number: the number of the ball (determines the name of the tf-frame i.e. ball0)
    """
    def tfFromImgPoint(self,icx,icy,ball_radius,ball_number):
        if self.fx != -1:
            x = float((icx-self.cx)*(1.0/self.fx))
            y = float((icy-self.cy)*(1.0/self.fy))
            z= float(1.0)
            ray_direction = np.array([x,y,z])
            try:
                if self.plane_computed == False:
                    self.field_point = TransformStamped() #Point that lies in the plane
                    self.field_point.header.frame_id="field"
                    self.field_point.child_frame_id="field_point"
                    self.field_point.header.stamp = self.get_clock().now().to_msg()
                    self.field_point.transform.translation.x = 0.0
                    self.field_point.transform.translation.y = 0.0
                    self.field_point.transform.translation.z = ball_radius
                    self.tfbr.sendTransform(self.field_point)
                    self.field_point = self.tfBuffer.lookup_transform("ceiling_cam","field_point",time=rclpy.time.Time())

                    self.field_normal = TransformStamped() #Point that lies above the plane and the vector between the field_point and this point is a normal of the plane
                    self.field_normal.header.frame_id="field"
                    self.field_normal.child_frame_id="field_normal"
                    self.field_normal.header.stamp = self.get_clock().now().to_msg()
                    self.field_normal.transform.translation.x = 0.0
                    self.field_normal.transform.translation.y = 0.0
                    self.field_normal.transform.translation.z = 1.0
                    self.tfbr.sendTransform(self.field_normal)
                    self.field_normal = self.tfBuffer.lookup_transform("ceiling_cam","field_normal",time=rclpy.time.Time())

        
                    self.field_normal = np.array([self.field_normal.transform.translation.x, self.field_normal.transform.translation.y, self.field_normal.transform.translation.z])
                    self.field_point = np.array([self.field_point.transform.translation.x, self.field_point.transform.translation.y, self.field_point.transform.translation.z])
                    # field normal is a vector! so it starts at field point and goes up in z direction
                    self.field_normal = self.field_point - self.field_normal
                    self.plane_computed = True

                n_dot_u = np.tensordot(self.field_normal, ray_direction, axes=([0],[0]))
                relative_ray_distance = -self.field_normal.dot(- self.field_point) / n_dot_u

                ray_direction[0] = np.multiply(relative_ray_distance, ray_direction[0])
                ray_direction[1] = np.multiply(relative_ray_distance, ray_direction[1])
                ray_direction[2] = np.multiply(relative_ray_distance, ray_direction[2])


                #publish the 3D coordinate as a transform
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = '/ceiling_cam'
                t.child_frame_id = "ball"+str(ball_number)

                t.transform.translation.x = ray_direction[0]
                t.transform.translation.y = ray_direction[1]
                t.transform.translation.z = ray_direction[2]

                t.transform.rotation.x = float(0)
                t.transform.rotation.y = float(0)
                t.transform.rotation.z = float(0)
                t.transform.rotation.w = float(1)

                # Send the transformation
                self.tfbr.sendTransform(t)
            except Exception:
                self.plane_computed=False
                print(traceback.format_exc())

    
    def cam_info_listener_callback(self,msg):
        #print(msg)

        P=msg.p
        self.fx = P[0]
        self.fy = P[5]
        self.cx = P[2]
        self.cy = P[6]


def main(args=None):
 
    rclpy.init(args=args)
    print("Starting YoloWrapper")

    yoloWrapper = YoloWrapper()

    rclpy.spin(yoloWrapper)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    yoloWrapper.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()




