import cv2
import numpy as np
import time
import argparse
import os
import sys
from pathlib import Path

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

### ROS2 Humble
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2




### yolo v5
import yaml 
import random
from festival_yolo.utils.torch_utils import select_device, load_classifier, time_synchronized
from festival_yolo.utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from festival_yolo.utils.datasets import LoadStreams, LoadImages, letterbox
from festival_yolo.models.experimental import attempt_load
import torch.backends.cudnn as cudnn
import torch

### Realsense
import pyrealsense2.pyrealsense2 as rs

class YoloV5:
    def __init__(self, yolov5_yaml_path):
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''模型初始化'''
        # 设置日志输出
        set_logging()
        # 选择计算设备
        device = select_device(self.yolov5['device'])
        # 如果是GPU则使用半精度浮点数 F16
        is_half = device.type != 'cpu'
        # 载入模型
        model = attempt_load(
            str(ROOT) + self.yolov5['weight']
            , map_location=device)
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # 检查模型的尺寸
        if is_half:
            model.half()  # 将模型转换为半精度
        # 设置BenchMark，加速固定图像的尺寸的推理
        cudnn.benchmark = True  # set True to speed up constant image size inference
        # 图像缓冲区初始化
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        print(is_half)
        # 创建模型
        # run once
        _ = model(img_torch.half())
        # _ = model(img_torch.half() if is_half else img) if device.type != 'cpu' else None
        self.is_half = is_half  # 是否开启半精度
        self.device = device  # 计算设备
        self.model = model  # Yolov5模型
        self.img_torch = img_torch  # 图像缓冲区

    def preprocessing(self, img):
        '''图像预处理'''
        # 图像缩放
        # 注: auto一定要设置为False -> 图像的宽高不同
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # print("img resize shape: {}".format(img_resize.shape))
        # 增加一个维度
        img_arr = np.stack([img_resize], 0)
        # 图像转换 (Convert) BGR格式转换为RGB
        # 转换为 bs x 3 x 416 x
        # 0(图像i), 1(row行), 2(列), 3(RGB三通道)
        # ---> 0, 3, 1, 2
        # BGR to RGB, to bsx3x416x416
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # 数值归一化
        # img_arr =  img_arr.astype(np.float32) / 255.0
        # 将数组在内存的存放地址变成连续的(一维)， 行优先
        # 将一个内存不连续存储的数组转换为内存连续存储的数组，使得运行速度更快
        # https://zhuanlan.zhihu.com/p/59767914
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''模型预测'''
        # 图像预处理
        img_resize = self.preprocessing(img)  # 图像缩放
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # 图像格式转换
        self.img_torch = self.img_torch.half(
        ) if self.is_half else self.img_torch.float()  # 格式转换 uint8-> 浮点数
        self.img_torch /= 255.0  # 图像归一化
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # 模型推理
        t1 = time_synchronized()
        pred = self.model(self.img_torch, augment=False)[0]
        # pred = self.model_trt(self.img_torch, augment=False)[0]
        # NMS 非极大值抑制
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_synchronized()
        # print("推理时间: inference period = {}".format(t2 - t1))
        # 获取检测结果
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # 画面中存在目标对象
            # 将坐标信息恢复到原始图像的尺寸
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = -1 # int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # 绘制矩形框与标签
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][-1], conf)
                    # self.plot_one_box(
                    #     xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        ''''绘制矩形框+标签'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        # cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        # if label:
        #     tf = max(tl - 1, 1)  # font thickness
        #     t_size = cv2.getTextSize(
        #         label, 0, fontScale=tl / 3, thickness=tf)[0]
        #     c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        #     cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        #     cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
        #                 [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

class Yolo_people(Node):
    def __init__(self):
        super().__init__('yolo_people')
        self.bridge = CvBridge()
        self.realsense_publisher = self.create_publisher(Image, '/image_raw', 10)


        self.human = TransformStamped()
        self.end = TransformStamped()
        ### Realsense pipeline and config
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.image_checker = True
                
        self.model = YoloV5(str(ROOT) + str('/config/yolov5.yaml'))

        self.peopletf_broadcaster = TransformBroadcaster(self)
        self.end_tf = TransformBroadcaster(self)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.peopletf)

        # self.peopletf()
        # # print("TFuvk")
    
    def peopletf(self):
        self.peopletf_broadcaster.sendTransform(self.human)
        self.peopletf_broadcaster.sendTransform(self.end)

        people_pose = self.yolopeople_callback()

        self.human.header.stamp = self.get_clock().now().to_msg()
        self.human.header.frame_id = 'front_camera_link'
        self.human.child_frame_id = 'human'

        self.end.header.stamp = self.get_clock().now().to_msg()
        self.end.header.frame_id = 'front_camera_link'
        self.end.child_frame_id = 'end'
        howmanypeoplearethere = len(people_pose)
        # print(people_pose)
        if howmanypeoplearethere > 0 :
            x = people_pose[0][2]
            y = -people_pose[0][0]
            z = -people_pose[0][1]

            self.human.transform.translation.x = people_pose[0][2]
            self.human.transform.translation.y = -people_pose[0][0]
            self.human.transform.translation.z = -people_pose[0][1]            
            # Original
            # self.human.transform.translation.x = people_pose[0][2] * 1
            # self.human.transform.translation.y = -people_pose[0][0] * 1
            # self.human.transform.translation.z = -people_pose[0][1] * 1

            # Photo Shot
            self.end.transform.translation.x = 0.9848*x + 0.1736*z - 0.8
            self.end.transform.translation.y = 0.0
            self.end.transform.translation.z = -0.1736*x + 0.9848*z + 0.25


        # if howmanypeoplearethere == 2:
        #     # print("It's two people")
        #     human.transform.translation.z = (people_pose[0][0] + people_pose[1][0]) / 2
        #     human.transform.translation.x = (people_pose[0][1] + people_pose[1][1]) / 2
        #     human.transform.translation.y = (people_pose[0][2] + people_pose[1][2]) / 2
        # elif howmanypeoplearethere > 0:
        #     human.transform.translation.z = people_pose[0][0] * 1
        #     human.transform.translation.x = people_pose[0][1] * 1
        #     human.transform.translation.y = people_pose[0][2] * 1
        # else:
        #     human.transform.translation.x = 0.0
        #     human.transform.translation.y = 0.0
        #     human.transform.translation.z = 0.0


       
    def yolopeople_callback(self):
        ### Get image to realsense
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = self.get_aligned_images()
        while not depth_image.any() or not color_image.any():
            if self.image_checker:
                self.image_checker = False 
                print("[Wait] Image wait")
        if self.image_checker:
            self.image_checker = False  
            print("[Done] Success get Image")

        ### Image Publish
        image_raw = self.bridge.cv2_to_imgmsg(color_image)
        self.realsense_publisher.publish(image_raw)


        ### Load image to realsense
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha = 0.03),
            cv2.COLORMAP_JET
        )
        images = np.hstack((color_image, depth_colormap))

        ### load yolo v5 model
        t_start = time.time()
        canvas, class_id_list, xyxy_list, conf_list = self.model.detect(color_image)
        t_end = time.time()

        camera_xyz_list=[]
        if xyxy_list:
            for i in range(len(xyxy_list)):
                ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)
                uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)
                dis = aligned_depth_frame.get_distance(ux, uy)
                camera_xyz = rs.rs2_deproject_pixel_to_point(
                    depth_intrin,
                    (ux, uy),
                    dis)
                
                camera_xyz = np.round(np.array(camera_xyz), 3)
                camera_xyz = camera_xyz.tolist()
                # print(camera_xyz)
                # if camera_xyz[0] == 0 or camera_xyz[1] == 0 or camera_xyz[2] == 0:
                #     print("====================0====================")
                cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)
                cv2.putText(canvas, str(camera_xyz), (ux+20, uy+10), 0, 1,
                            [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)
                camera_xyz_list.append(camera_xyz)

        fps = int(1.0 / (t_end - t_start))
        cv2.putText(canvas, text="FPS: {}".format(fps), org=(50, 50),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,
                    lineType=cv2.LINE_AA, color=(0, 0, 0))
        cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                        cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
        cv2.imshow('detection', canvas)
        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            # break
        
        # print(len(camera_xyz_list))

        return camera_xyz_list
        
    def get_aligned_images(self):

        # class self realsense config
        # self.pipeline, self.config, self.profile, self.align, self.align

        frames = self.pipeline.wait_for_frames() 
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        intr = color_frame.profile.as_video_stream_profile().intrinsics 
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
        ).intrinsics
        '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                            'ppx': intr.ppx, 'ppy': intr.ppy,
                            'height': intr.height, 'width': intr.width,
                            'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                            }'''

        depth_image = np.asanyarray(aligned_depth_frame.get_data()) 
        depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03) 
        depth_image_3d = np.dstack(
            (depth_image_8bit, depth_image_8bit, depth_image_8bit)) 
        color_image = np.asanyarray(color_frame.get_data())

        return intr, depth_intrin, color_image, depth_image, aligned_depth_frame
          
def main(args=None):
    rclpy.init(args=args)
    yolo_people = Yolo_people()
    rclpy.spin(yolo_people)
    yolo_people.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

