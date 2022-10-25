"""
This script can be used to evaluate the precision of the AprilTag Detection System. 
For that a target pose is created at a hand-measured position of the tag on the field.
 The measurements are taken in the coordinate system of the field middle.
Then 10 detected poses of the tag are compared with the target pose and the errors are computed and saved in a txt file,
 together with the mean error and standard deviation. Additionally, one image for each position is saved as well. 
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
import math
import tf2_ros
import cv2
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_msgs.msg import TFMessage
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import os

def euler_from_quaternion(x, y, z, w):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            Code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/ (Last accessed: 25.10.22)
            """
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x*(360/(2*math.pi)), pitch_y*(360/(2*math.pi)), yaw_z*(360/(2*math.pi)) # in degrees
    
def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
  Code from https://automaticaddison.com/how-to-convert-euler-angles-to-quaternions-using-python/ (Last accessed: 25.10.22)   
  Input
    :param roll: The roll (rotation around x-axis) angle in degrees.
    :param pitch: The pitch (rotation around y-axis) angle in degrees.
    :param yaw: The yaw (rotation around z-axis) angle in degrees.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  roll=roll*((2*math.pi)/360)
  pitch=pitch*((2*math.pi)/360)
  yaw=yaw*((2*math.pi)/360)
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.targetx=0
        self.targety=0
        self.targetz=0
        self.targetrollx=0
        self.targetpitchy=0
        self.targetyawz=0
        self.measurements = np.zeros((1,6))


        self.savedImage = False
        self.imageName=''
        self.counterMeasurements = 0

        self.tfBuffer = Buffer()
        self.listener = TransformListener(self.tfBuffer,self)

        self.targetPublisher = StaticTransformBroadcaster(self)

        self.subscription = self.create_subscription(
           AprilTagDetectionArray,
           'apriltag_detections',
           self.listener_callback,
           10)
        self.subscription  # prevent unused variable warning
        self.fieldSub = self.create_subscription(
           TFMessage,
           'tf_static',
           self.TFStaticlistener_callback,
           10
        )
        self.tfSub = self.create_subscription(
            TFMessage,
            'tf',
            self.TFlistener_callback,
            10
        )
        self.imgsub = self.create_subscription(
            Image,
            '/ceiling_cam/ceiling_cam_publisher/image_color_rect',
            self.listener_callback_Image,
            10)
        self.br = CvBridge()

        self.imgpub = self.create_publisher(Image, '/ceiling_cam/ceiling_cam_publisher/camera_image_rect2', 10)#publishes the Image again so that the apriltag detector can subscribe to this topic since two subscribers on the same image topic slow it down

        self.maketarget()


    def maketarget(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'ceiling_cam'
        static_transformStamped.child_frame_id = 'targetPose'
        #measured values where the tag is with respect to the mid of the field
        #have to be changed for each position at which the AprilTag tracking should be evaluated
        self.targetx=0.058
        self.targety=-0.024
        self.targetz=3.26


        self.targetx=float(self.targetx)
        self.targety=float(self.targety)
        self.targetz=float(self.targetz)
        static_transformStamped.transform.translation.x = self.targetx
        static_transformStamped.transform.translation.y = self.targety
        static_transformStamped.transform.translation.z = self.targetz
        #measured orientation of the tag with respect to the field coordinate frame
        #have to be changed for each orientation at which the AprilTag tracking should be evaluated

        self.targetrollx = 171.5106133
        self.targetpitchy = -5.912397
        self.targetyawz = -0.7815405

        x,y,z,w=get_quaternion_from_euler(self.targetrollx,self.targetpitchy,self.targetyawz) #(rot,grün,blau) in rviz
        static_transformStamped.transform.rotation.x = x
        static_transformStamped.transform.rotation.y = y
        static_transformStamped.transform.rotation.z = z
        static_transformStamped.transform.rotation.w = w
        self.targetPublisher.sendTransform(static_transformStamped)

    def listener_callback_Image(self, msg):
        self.imgpub.publish(msg)
        if self.savedImage == False:
            DIR='AprilTags/EvaluationImages2'#Directory where for each evaluated position one image is saved
            imgnumber = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])+1
            img=self.br.imgmsg_to_cv2(msg)
            self.imageName = "evalImg00"+str(imgnumber)+'.png'
            cv2.imwrite(DIR+'/'+self.imageName,img)
            self.savedImage = True

    def TFlistener_callback(self, msg):   
        try:
            #calculating the error
            trans = self.tfBuffer.lookup_transform('targetPose','my25Tag0',rclpy.time.Time())#in this case the tag with the name my25Tag0 was used, the name has to be changed if a different tag is used
            transFT = self.tfBuffer.lookup_transform('field','targetPose',rclpy.time.Time())
            transFM = self.tfBuffer.lookup_transform('field','my25Tag0',rclpy.time.Time())
            x=transFT.transform.translation.x-transFM.transform.translation.x
            y=transFT.transform.translation.y-transFM.transform.translation.y
            z=transFT.transform.translation.z-transFM.transform.translation.z

            orientation = trans.transform.rotation
            roll_x, pitch_y, yaw_z = euler_from_quaternion(orientation.x,orientation.y,orientation.z, orientation.w)
            self.get_logger().info('I heard My25Tag0')
            self.get_logger().info('I heard position: "x: %f, y: %f, z: %f"' %(x,y,z))
            self.get_logger().info('I heard orientation: "x: %f, y: %f, z: %f"' %(roll_x, pitch_y, yaw_z))


            if self.counterMeasurements==0 and self.imageName!='':
                f=open('AprilTags/EvaluationData2/measurements.txt','a')
                f.write("new Measurement for :"+str(self.imageName)+": target:x:"+str(self.targetx)+":y:"+str(self.targety)+":z:"+str(self.targetz)+":rollx:"+str(self.targetrollx)+":pitchy:"+str(self.targetpitchy)+":yawz:"+str(self.targetyawz)+"\n")
                f.close()
            if self.counterMeasurements<10 and self.imageName!='':
                f=open('AprilTags/EvaluationData2/measurements.txt','a')
                f.write("x:"+str(x)+":y:"+str(y)+":z:"+str(z)+":rollx:"+str(roll_x)+":pitchy:"+str(pitch_y)+":yawz:"+str(yaw_z)+"\n")
                f.close()
                self.counterMeasurements+=1
                self.measurements=np.append(self.measurements,[[x,y,z,roll_x,pitch_y,yaw_z]],axis=0)
                print(self.measurements)
            if self.counterMeasurements==10:
                self.measurements=np.delete(self.measurements,0,0)
                mean = np.mean(self.measurements,axis=0)
                stdv = np.std(self.measurements,axis=0)
                f=open('AprilTags/EvaluationData2/measurements.txt','a')
                f.write("mean x:"+str(mean[0])+":y:"+str(mean[1])+":z:"+str(mean[2])+":rollx:"+str(mean[3])+":pitchy:"+str(mean[4])+":yawz:"+str(mean[5])+"\n")
                f.write("stdv x:"+str(stdv[0])+":y:"+str(stdv[1])+":z:"+str(stdv[2])+":rollx:"+str(stdv[3])+":pitchy:"+str(stdv[4])+":yawz:"+str(stdv[5])+"\n")
                f.close()
                self.counterMeasurements+=1
        except Exception as e:
            print(e)


def main(args=None):
    print("AprilTagEvaluation started")
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