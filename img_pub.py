#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImagePublisher:
    def __init__(self):
        rospy.init_node('image_publisher', anonymous=True)
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.capture = cv2.VideoCapture(0)  # 打开摄像头
        if not self.capture.isOpened():
            rospy.logerr("无法打开摄像头")
            exit()

    def publish_image(self):
        while not rospy.is_shutdown():
            ret, frame = self.capture.read()
            if ret:
                try:
                    msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.image_pub.publish(msg)
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))
            else:
                rospy.logerr("无法读取摄像头图像")
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        img_pub = ImagePublisher()
        img_pub.publish_image()
    except rospy.ROSInterruptException:
        pass