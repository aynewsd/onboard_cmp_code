#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uav_camera_publisher

功能：
  - 发布 /camera/image_raw (sensor_msgs/Image)
  - 发布 /camera/camera_info (sensor_msgs/CameraInfo)

支持两种数据源：
  1) camera：从本机相机设备采集（OpenCV VideoCapture）
  2) folder：从本地文件夹按文件名顺序读取图片并循环发布

使用方式：
  roslaunch uav_camera_publisher camera_publisher.launch
  然后通过参数 source_type:=camera 或 source_type:=folder 切换
"""

import os
import time
from typing import List

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

try:
    from camera_info_manager import CameraInfoManager
except Exception as e:
    CameraInfoManager = None


def _list_images(folder: str, exts: List[str]) -> List[str]:
    """列出 folder 下所有符合扩展名的文件，按字典序排序。"""
    if not folder:
        return []
    if not os.path.isdir(folder):
        return []
    files = []
    for name in os.listdir(folder):
        path = os.path.join(folder, name)
        if not os.path.isfile(path):
            continue
        lower = name.lower()
        if any(lower.endswith(ext) for ext in exts):
            files.append(path)
    files.sort()
    return files


class CameraPublisherNode:
    def __init__(self):
        # -----------------------------
        # 读取参数
        # -----------------------------
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.loop = bool(rospy.get_param("~loop", True))

        self.image_topic = rospy.get_param("~image_topic", "/camera/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera_info")

        self.camera_name = rospy.get_param("~camera_name", "camera")
        self.camera_info_url = rospy.get_param("~camera_info_url", "")

        self.source_type = rospy.get_param("~source_type", "camera").strip().lower()

        # camera 模式
        self.device_id = int(rospy.get_param("~device_id", 0))
        self.device_path = rospy.get_param("~device_path", "").strip()
        self.width = int(rospy.get_param("~width", 640))
        self.height = int(rospy.get_param("~height", 480))

        # folder 模式
        self.folder_path = rospy.get_param("~folder_path", "").strip()
        self.extensions = rospy.get_param("~extensions", [".jpg", ".jpeg", ".png", ".bmp"])

        self.encoding = rospy.get_param("~encoding", "bgr8")

        # -----------------------------
        # Publisher / 工具
        # -----------------------------
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)

        # camera_info_manager：优先加载标定文件；若不可用则只发布 Image（但一般 noetic 有该包）
        self.cinfo = None
        self.cinfo_pub = None
        if CameraInfoManager is not None:
            self.cinfo = CameraInfoManager(cname=self.camera_name, url=self.camera_info_url)
            try:
                # loadCameraInfo() 在 url 为空时不会报错，但也不会加载标定
                self.cinfo.loadCameraInfo()
            except Exception as e:
                rospy.logwarn("camera_info_url load failed: %s", str(e))
            self.cinfo_pub = rospy.Publisher(self.camera_info_topic, type(self.cinfo.getCameraInfo()), queue_size=10)
        else:
            rospy.logwarn("camera_info_manager not available; /camera/camera_info will NOT be published.")

        # -----------------------------
        # 初始化数据源
        # -----------------------------
        self.cap = None
        self.images = []
        self.image_index = 0

        if self.source_type == "camera":
            self._init_camera()
        elif self.source_type == "folder":
            self._init_folder()
        else:
            raise RuntimeError(f"Unknown source_type: {self.source_type} (use 'camera' or 'folder')")

        rospy.loginfo("uav_camera_publisher started: source_type=%s image_topic=%s camera_info_topic=%s",
                      self.source_type, self.image_topic, self.camera_info_topic)

    def _init_camera(self):
        """初始化 OpenCV 相机采集。"""
        if self.device_path:
            self.cap = cv2.VideoCapture(self.device_path)
        else:
            self.cap = cv2.VideoCapture(self.device_id)

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open camera device. Check device_id/device_path and permissions.")

        # 设置分辨率（不保证所有相机都支持指定值）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))

    def _init_folder(self):
        """初始化文件夹读取列表。"""
        self.images = _list_images(self.folder_path, [e.lower() for e in self.extensions])
        if not self.images:
            raise RuntimeError(f"No images found in folder_path={self.folder_path}")
        self.image_index = 0

    def _publish_camera_info(self, stamp: rospy.Time, width: int, height: int):
        """发布 CameraInfo（如有标定则用标定，否则发布一个带宽高的默认 CameraInfo）。"""
        if self.cinfo is None or self.cinfo_pub is None:
            return

        info = self.cinfo.getCameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id

        # 如果没有标定文件，camera_info_manager 返回的 CameraInfo 往往 width/height 为 0
        if info.width == 0:
            info.width = width
        if info.height == 0:
            info.height = height

        self.cinfo_pub.publish(info)

    def _read_frame(self):
        """从当前数据源读取一帧 OpenCV 图像（numpy array）。"""
        if self.source_type == "camera":
            ok, frame = self.cap.read()
            if not ok or frame is None:
                return None
            return frame

        # folder
        if self.image_index >= len(self.images):
            if not self.loop:
                return None
            self.image_index = 0

        path = self.images[self.image_index]
        self.image_index += 1

        frame = cv2.imread(path, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn("Failed to read image: %s", path)
            return None
        return frame

    def spin(self):
        rate = rospy.Rate(max(0.1, self.publish_rate))

        while not rospy.is_shutdown():
            frame = self._read_frame()
            if frame is None:
                rospy.loginfo("No more frames. Exiting.")
                break

            stamp = rospy.Time.now()
            height, width = frame.shape[:2]

            # 生成并发布 Image 消息
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)

            # 发布 CameraInfo
            self._publish_camera_info(stamp, width, height)

            rate.sleep()

        # 释放相机资源
        if self.cap is not None:
            self.cap.release()


def main():
    rospy.init_node("uav_camera_publisher", anonymous=False)
    try:
        node = CameraPublisherNode()
    except Exception as e:
        rospy.logerr("Failed to start uav_camera_publisher: %s", str(e))
        raise
    node.spin()


if __name__ == "__main__":
    main()

