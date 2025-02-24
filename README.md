# 图像处理与ROS任务结果报告

## 项目概述
本项目通过综合使用Linux、Python、ROS和OpenCV，完成图像处理、目标检测和SLAM任务。任务分为三个部分：基本任务（图像处理与发布）、进阶任务（YOLO目标识别）。
- '尝试.py'用于静态opencv红球识别
- 'zhaoyuan.py'用于动态前置摄像头opencv识别
- '1.jpg''2.jpg''3.jpg''4.jpg'是红色球识别部分样本（选择robot数据集）
- 'img_sub''img_pub'是ROS for python上的对应代码
- 'yolov5-master'是yolov5配置文件及教程
- 'onnxruntime-for-yolov5'是最终跑通的cpu版本的yolov5的demo
---

## 1. 基本任务：图像处理与发布

### 任务目标
- 使用Python和OpenCV检测图像中的红色物体，框选并输出其坐标。
- 编写`img_pub.py`和`img_sub.py`两个文件，分别用于图像发布和图像处理。
- 使用`rqt_image`工具查看处理后的图像。

### 实现方法
- **`img_pub.py`**：通过`cv_bridge`获取摄像头图像，并通过ROS的`publisher`发布图像。
- **`img_sub.py`**：订阅图像话题，使用OpenCV进行图像处理，框选红色物体并输出其坐标。
- 使用`rqt_image`工具查看处理后的图像。

### 运行结果
- 成功框选红色物体，并输出其坐标。
- 处理后的图像通过`rqt_image`工具正常显示。

---

## 2. 进阶任务：YOLO目标识别

### 任务目标
- 跑通YOLO框架，完成目标识别。
- 将识别到的物体坐标通过ROS话题发布。

### 实现方法
- 使用YOLOv5框架进行目标检测。
- 在ROS中集成YOLO，将检测结果发布到指定话题。
- 测试视频流中的目标识别功能。

### 运行结果
- 成功识别视频中的物体，并输出其像素坐标。
- 通过ROS话题发布检测结果，验证功能正常。

---

## 3. 学习资源

### github
- **书籍**: 《跟wakaba酱一起学git使用》

### Python
- **书籍**：《Python编程 从入门到实践》  
- **视频**：[超基础Python教程]([https://www.bilibili.com/video/BV1ex411x7Em/?spm_id_from=333.337.search-card.all.click&vd_source=8c2e809fc5440a60dc7429b2c21e32c7])  
- **文档**：[Python 3.9.7官方文档](https://docs.python.org/3.9/)  

### Linux
- **教程**：[Linux入门教程](https://www.runoob.com/linux/linux-tutorial.html)  

### ROS
- **教程**：[古月居ROS入门21讲](https://www.bilibili.com/video/BV1xx411d7v7)  

### OpenCV
- **教程**：[OpenCV从入门到实战](https://www.bilibili.com/video/BV1xx411d7v7)  
- **安装**：[Anaconda安装OpenCV](https://blog.csdn.net/iracer/article/details/10888888)

---

## 4. 运行环境
- **操作系统**：Ubuntu 20.04  
- **Python版本**：3.8  
- **ROS版本**：Noetic  
- **OpenCV版本**：4.8.0.74  
- **YOLO版本**：v5  
- **yolo项目来自**："https://github.com/huange888/yolov5_7.0_pyside6_active_learning.git"（血细胞识别）  https://github.com/ultralytics/yolov5.git（yolo环境配置教程）
---

## 5. 过程说明
- 因假期时间安排原因，本次项目的学习过程集中于开学前几日。
- 首先基于github平台，进行了git项目管理学习，已能熟练掌握fork、pr、branch等概念及操作，并已与团队协作进行小程序开发（https://github.com/princejiajiajing/doubaozhiyuan-final.git）。
- 针对opencv红球识别任务，前置进行了“黑马程序员python”视频课程学习，辅以先前视觉识别实验室实习经验，在python环境利用opencv-python进行开发。
- 学习一定ROS原理及操作,复习Linux操作系统知识。尝试用choco工具进行ROS工具配置(遗憾的是因cpu版本下载入口被墙，clash不稳定等原因中道崩殂，有代码没环境）
- 对https://github.com/ultralytics/yolov5.git进行clone，本地化后学习yolo各模块功能
- 对https://github.com/huange888/yolov5_7.0_pyside6_active_learning.git进行yolo项目实操，cpu版本无法全数加载Qt终端界面（应该是与PyQt5设置有关），但终端显示应已经完成了识别（cpu版本速率实在慢）。
- 开学后，（居然三月二号才交，那ddl还有些时间），对https://github.com/Amelia0911/onnxruntime-for-yolov5.git进行cpu版本的项目实操，并成功进行了一个小demo的实践，成功跑通yolov5（hooray！）
- 分析之前yolo跑不通的问题：①当时初学python虚拟环境环境配的乱乱的，anaconda还在学校的u盘里没带回去，回学校用conda环境被理顺了就好很多了。   ②gpu版本确实不太适合没卡的电脑    ③demo算量极小，可以很快出反馈

## 6. 不足与反思
- 针对红球的视觉识别，颜色阈值（HSV二值化）与性状阈值有待优化，存在误判情况（尤其是动态情况下）。
- ROS环境配置失败（choco工具），将使用新clash节点，或手动配置cpu版windows，ros for python（buff叠满了属于是），或尝试ubuntu上Linux版本的开发。
- yolo识别，考虑在未来对yolo模型进行调换，实现性能的进一步提升。
---

## 7. 项目总结
本项目成功实现了图像处理、目标检测功能。通过实践，掌握了相关技术，部分完成了预期目标，将在未来三个月内进一步学习。

---
感谢各位学长学姐的帮助！
