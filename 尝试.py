import cv2
import numpy as np

# 1. 读取图像
img = cv2.imread('2.jpg')  # 替换为你的图片路径
if img is None:
    print("无法打开或找到图像，请检查路径是否正确")
    exit()

# 2. 转换到HSV色彩空间
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# 3. 定义红色在HSV中的范围
#   通常红色在HSV有两段区间(低H值以及高H值)，可根据实际环境调整
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
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 开运算去除小噪点
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
    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 计算中心坐标
    center_x = x + w/2
    center_y = y + h/2

    # 在图上标注中心坐标
    text = f"{center_x:.1f}, {center_y:.1f}"
    cv2.putText(img, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 0, 255), 2, cv2.LINE_AA)

    print("检测到的红球坐标：", text)
else:
    print("未检测到明显的红色目标，可能需要调整HSV范围或形态学参数。")

# 9. 显示结果
cv2.imshow('Original Image with Detection', img)
cv2.imshow('Mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()
