import os
import json
from time import sleep
import time
import tkinter as tk
from tkinter import ttk
from enum import Enum

import numpy as np
import cv2 as cv
import pyrealsense2 as rs

from ._camerafeedwindow import CameraFeedWindow
from ._positionestimation import PositionEstimation, image_pixel_to_mm, CONFIG_FILENAME
from emioapi._logging_config import logger

DEFAULT_CAMERA_PARAMS = {"hue_h": 90, "hue_l": 36, "sat_h": 255, "sat_l": 138, "value_h": 255, "value_l": 35, "erosion_size": 0, "area": 100}

class CalibrationStatusEnum(Enum):
    NOT_CALIBRATED = 0,
    CALIBRATING = 1,
    CALIBRATED = 2


def compute_contour_center(contour):
    M = cv.moments(contour)
    cX = 0
    cY = 0
    if M['m00'] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    return cX, cY


def compute_median_depth(contour, depth_image):
    image = np.zeros_like(depth_image)
    # Fills the area bounded by the contours if thickness < 0
    cv.drawContours(image, contours=[contour], contourIdx=0, color=255, thickness=-1)
    points = np.where(image == 255)
    depth_values = depth_image[points[0], points[1]].flatten()
    valid_depth_values = depth_values[depth_values > 0]
    if len(valid_depth_values) > 0:
        return np.median(valid_depth_values)
    else:
        return 0


def list_cameras() -> list:
    context = rs.context()
    return [d.get_info(rs.camera_info.serial_number) for d in context.devices]


class DepthCamera:

    height = 480
    width = 640
    device = None
    pipeline_profile = None
    pipeline_wrapper = None
    rsconfig = None
    pipeline = None
    point_cloud = None
    fps = 30
    intr = None
    profile = None
    initialized = False
    pc = None
    compute_point_cloud = False
    position_estimator: PositionEstimation = None
    parameter = {}
    tracking = True
    trackers_pos = []
    maskWindow = None
    frameWindow = None
    hsvWindow = None
    depthWindow = None
    rootWindow = None
    hsvFrame = None
    maskFrame = None
    frame: np.ndarray = None
    depth_frame: np.ndarray = None
    depth_max = 430
    depth_min = 2
    calibration_status = CalibrationStatusEnum.NOT_CALIBRATED

    @property
    def camera_serial(self) -> str:
        """
        Returns the serial of the camera as str

        """
        return self.device.get_info(rs.camera_info.serial_number) if self.device else None


    def __init__(self,
                 camera_serial: str=None,
                 parameter: dict=None,
                 compute_point_cloud: bool=False,
                 show_video_feed: bool=False,
                 tracking: bool=True,
                 configuration: str="extended") -> None:
        """
        Initialize the camera and the parameters.

        Args:
            parameter : dict
                The parameters for the camera. If None, the default parameters will be used.
            comp_point_cloud : bool
                If True, the point cloud will be computed.
            show_video_feed : bool
                If True, the video feed will be shown.
            track: bool
                If True, the tracking will be enabled.
            configuration: str
                Configuration of Emio, either "extended" (default) or "compact"
        """
        self.tracking = tracking
        self.show_video_feed = show_video_feed
        self.compute_point_cloud = compute_point_cloud
        self.configuration = configuration

        self.initialized = True

        if not self.initialized:
            return

        self.trackers_pos = []

        if parameter:
            self.parameter = parameter
        else:
            try:
                with open(CONFIG_FILENAME, 'r') as fp:
                    json_parameters = json.load(fp)
                    self.parameter.update(json_parameters)
                    logger.info(f'Config file {CONFIG_FILENAME} found. Using parameters {self.parameter}')

            except FileNotFoundError:
                logger.warning(f'Config file {CONFIG_FILENAME} not found. Using default parameters {DEFAULT_CAMERA_PARAMS}')
                self.parameter.update(DEFAULT_CAMERA_PARAMS)

        default_param = self.parameter.copy()

        self.initialized = True

        if self.show_video_feed:
            self.create_feed_windows()

        # self.update() # to get a first frame and trackers

    def set_fps(self, new_fps: int):
        if new_fps in [30, 60, 90]:
            self.fps = new_fps
        else:
            raise ValueError("fps can only be 30, 60 or 90")

    def set_depth_max(self, new_depth_max: int):
        if new_depth_max > 0:
            self.depth_max = new_depth_max
        else:
            raise ValueError("depth_max must be greater than 0")

    def set_depth_min(self, new_depth_min: int):
        if new_depth_min >= 0:
            self.depth_min = new_depth_min
        else:
            raise ValueError("depth_min must be greater than or equal to 0")


    def create_feed_windows(self):
        self.rootWindow = tk.Tk()
        self.rootWindow.resizable(False, False)

        self.rootWindow.title("Camera Feed Manager")
        ttk.Button(self.rootWindow, text="Close Windows", command=self.quit).pack(side=tk.BOTTOM, padx=5, pady=5)
        ttk.Button(self.rootWindow, text="Save", command=lambda: json.dump(self.parameter, open(CONFIG_FILENAME, 'w'))).pack(side=tk.BOTTOM, padx=5, pady=5)
        ttk.Button(self.rootWindow, text="Mask Window", command=self.create_mask_window).pack(side=tk.BOTTOM, padx=5, pady=5)
        ttk.Button(self.rootWindow, text="Frame Window", command=self.create_frame_window).pack(side=tk.BOTTOM, padx=5, pady=5)
        ttk.Button(self.rootWindow, text="HSV Window", command=self.create_HSV_window).pack(side=tk.BOTTOM, padx=5, pady=5)
        ttk.Button(self.rootWindow, text="Depth Window", command=self.createDepthWindow).pack(side=tk.BOTTOM, padx=5, pady=5)

        self.create_mask_window()
        self.create_frame_window()

        self.rootWindow.protocol("WM_DELETE_WINDOW", self.quit)
        self.rootWindow.update_idletasks()

    def create_mask_window(self):
        if self.maskWindow is None or not self.maskWindow.running:
            self.maskWindow = CameraFeedWindow(rootWindow=self.rootWindow, trackbarParams=self.parameter, name='Mask')

    def create_frame_window(self):
        if self.frameWindow is None or not self.frameWindow.running:
            self.frameWindow = CameraFeedWindow(rootWindow=self.rootWindow, name='RGB Frame')

    def create_HSV_window(self):
        if self.hsvWindow is None or not self.hsvWindow.running:
            self.hsvWindow = CameraFeedWindow(rootWindow=self.rootWindow, name='HSV Frame')

    def createDepthWindow(self):
        if self.depthWindow is None or not self.depthWindow.running:
            self.depthWindow = CameraFeedWindow(rootWindow=self.rootWindow, name='Depth Frame')

    def quit(self):
        for window in [self.maskWindow, self.frameWindow, self.hsvWindow, self.depthWindow]:
            if window is not None:
                window.closed()
        self.rootWindow.destroy()
        self.show_video_feed = False
        self.rootWindow = None

    def init_realsense(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.rsconfig = rs.config()
        self.pc = rs.pointcloud()

        if  self.camera_serial is not None:
            self.rsconfig.enable_device(self.camera_serial)

        # Get device product line for setting a supporting resolution
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.rsconfig.resolve(self.pipeline_wrapper)

        self.device = self.pipeline_profile.get_device()

        self.rsconfig.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.rsconfig.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        depth_sensor = self.device.first_depth_sensor()
        depth_sensor.set_option(rs.option.depth_units, 0.001)

        cfg = self.pipeline.start(self.rsconfig)

        self.profile = cfg.get_stream(rs.stream.depth)
        self.intr = self.profile.as_video_stream_profile().get_intrinsics()

        # Initialize the position estimation by reading the calibration file
        self.position_estimator = PositionEstimation(self.intr, self.configuration)
        self.position_estimator.intr= self.intr
        self.position_estimator.compute_camera_to_simulation_transform()

        if not self.position_estimator.initialized:
            logger.error('Position estimation initialization failed. Using default parameters.')
            raise Exception('Position estimation initialization failed. Please check the camera calibration.')

    def open(self):
        try:
            self.init_realsense()
        except Exception as err:
            self.initialized = False
            raise Exception('Could not open depthcamera', str(err))


    def calibrate(self):
        starttime = time.time()
        first = False
        success = False
        self.calibration_status = CalibrationStatusEnum.CALIBRATING

        # Create the windows to display the binrary mask and the HSV frame
        calibration_window = CameraFeedWindow(rootWindow=self.rootWindow, name='Calibration')

        if self.position_estimator is not None:
            while self.position_estimator.count_calibration_frames < 200 and time.time() - starttime < 300:
                self.position_estimator.intr= self.intr
                _, color_image, depth_image, _ = self.get_frame()
                success = self.position_estimator.calibrate(color_image, depth_image, first, calibration_window)
                first = success if not first else first
                self.rootWindow.update()

        if success:
            self.position_estimator.compute_camera_to_simulation_transform()
            logger.info(f"Camera {self.camera_serial} successfully calibrated.")

        # Close the calibration window
        calibration_window.closed()

        self.calibration_status = CalibrationStatusEnum.CALIBRATED if success else CalibrationStatusEnum.NOT_CALIBRATED
        return success


    def get_frame(self):
        # Wait for a coherent pair of frames: depth and color

        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return False, color_frame, depth_frame

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        return True, color_image, depth_image, depth_frame


    def update(self):
        ret, self.frame, self.depth_frame, depth_rsframe = self.get_frame()

        if ret is False:
            return
        # if frame is read correctly ret is True

        self.hsvFrame = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)

        # color definition
        red_lower = np.array([self.parameter['hue_l'], self.parameter['sat_l'], self.parameter['value_l']])
        red_upper = np.array([self.parameter['hue_h'], self.parameter['sat_h'], self.parameter['value_h']])

        # red color mask (sort of thresholding, actually segmentation)
        mask = cv.inRange(self.hsvFrame, red_lower, red_upper)
        mask2 = cv.inRange(self.depth_frame, self.depth_min, self.depth_max)

        mask = cv.bitwise_and(mask, mask2, mask=mask)

        erosion_shape = cv.MORPH_RECT
        erosion_size = self.parameter['erosion_size']
        element = cv.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                           (erosion_size, erosion_size))

        mask = cv.erode(mask, element, iterations=3)
        mask = cv.dilate(mask, element, iterations=3)

        self.maskFrame = cv.bitwise_and(self.frame, self.frame, mask=mask)

        if self.tracking:
            contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if len(contours) != 0:
                areas = [cv.contourArea(cnt) for cnt in contours]

                self.trackers_pos = []
                for i, a in enumerate(areas):
                    if a > self.parameter['area']:
                        x, y = compute_contour_center(contours[i])
                        marker_mask = np.zeros_like(mask)

                        depth = compute_median_depth(contours[i], self.depth_frame) if self.depth_frame[y, x] == 0 else self.depth_frame[y, x]
                        worldx, worldy, worldz = self.position_estimator.camera_image_to_simulation(x, y, depth)
                        self.trackers_pos.append([worldx, worldy, worldz])

                        cv.drawContours(marker_mask, [contours[i]], -1, color=255, thickness=-1)
                        for frame in [self.hsvFrame, self.frame]:
                            cv.circle(frame, (x, y), 2, color=255, thickness=-1)
                            cv.putText(frame, f"{i} ({x}, {y}, {depth})", (x, y), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                            cv.putText(frame, f"{i} ({worldx:.2f}, {worldy:.2f}, {worldz:.2f})", (x, y + 15), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                        if self.show_video_feed:
                            cv.drawContours(self.frame, contours[i], -1, (255, 255, 0), 3)

        if self.compute_point_cloud:
            points = self.pc.calculate(depth_rsframe)
            v = points.get_vertices()
            self.point_cloud = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz

        if self.show_video_feed:
            if self.rootWindow is None:
                self.create_feed_windows()

            if self.maskWindow is not None and self.maskWindow.running:
                self.maskWindow.set_frame(self.maskFrame)

            if self.frameWindow is not None and self.frameWindow.running:
                self.frameWindow.set_frame(self.frame)

            if self.hsvWindow is not None and self.hsvWindow.running:
                self.hsvWindow.set_frame(self.hsvFrame)

            if self.depthWindow is not None and self.depthWindow.running:
                colorized = np.asanyarray(rs.colorizer().colorize(depth_rsframe).get_data())
                self.depthWindow.set_frame(colorized)

            self.rootWindow.update()


    def close(self):
        try:
            self.initialized = False
            if self.pipeline:
                self.pipeline.stop()
            if self.rootWindow:
                self.rootWindow.destroy()
        except:
            pass


    def run_loop(self):
        while True:
            if self.rootWindow is None or not self.rootWindow.winfo_exists():
                break
            if self.show_video_feed:
                self.rootWindow.update()
            self.update()

        self.close()
