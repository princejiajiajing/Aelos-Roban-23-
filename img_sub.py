#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/camera/image_processed', Image, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        processed_image = self.process_image(cv_image)
        try:
            msg = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def process_image(self, image):
        # 2. 转换到HSV色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 3. 定义红色在HSV中的范围
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # 4. 对红色区域进行二值化处理
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2  # 合并两段红色区间的检测

        # 5. 形态学操作（可选），去除噪点、填充孔洞
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 开运算去除小噪点
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算填充小黑洞

        # 6. 查找轮廓
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 7. 寻找最大轮廓（假设图中只有一个主要的红球）
        largest_contour = None
        max_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area > max_area:
                max_area = area
                largest_contour = c

        # 8. 若找到轮廓，则进行框选并标注中心点坐标
        if largest_contour is not None:
            x, y, w, h = cv2.boundingRect(largest_contour)
            # 在原图上绘制矩形
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 计算中心坐标
            center_x = x + w / 2
            center_y = y + h / 2

            # 在图上标注中心坐标
            text = f"{center_x:.1f}, {center_y:.1f}"
            cv2.putText(image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2, cv2.LINE_AA)

            rospy.loginfo("检测到的红球坐标： " + text)
        else:
            rospy.loginfo("未检测到明显的红色目标，可能需要调整HSV范围或形态学参数。")

        return image

if __name__ == '__main__':
    try:
        img_sub = ImageSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass