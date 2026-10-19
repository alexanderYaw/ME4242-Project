from multiprocessing.managers import ListProxy, DictProxy
from multiprocessing.sharedctypes import Synchronized
from multiprocessing.managers import SyncManager
import multiprocessing.sharedctypes
from multiprocessing.synchronize import Lock
from multiprocessing import Process
import multiprocessing
import time
from ctypes import c_wchar_p

import numpy as np

from emioapi._depthcamera import *
from emioapi._logging_config import logger


class MultiprocessEmioCamera:
    """
    A class to interface with the realsense camera on Emio.
    This class creates a process using mulltiprocessing to handle the camera.

    :::warning
    This class does **not** work in a SOFA scene. The multiprocessing clashes with SOFA.
    If you want to use the camera in a SOFA scene, use the not-paralelized version of the class: [EmioCamera](#EmioCamera)
    :::

    Example:
        ```python
        from emioapi import MultiprocessEmioCamera

        # Initialize the camera with default parameters
        camera = MultiprocessEmioCamera(show=True, tracking=True, compute_point_cloud=True)

        # Open the camera (starts the camera process)
        if camera.open():
            print("Camera started successfully.")

            # Access tracker positions and point cloud in a loop
            for _ in range(10):
                print("Trackers positions:", camera.trackers_pos)
                print("Point cloud shape:", camera.point_cloud.shape if camera.point_cloud is not None else None)
                time.sleep(1)

            # Close the camera process
            camera.close()
            print("Camera closed.")
        else:
            print("Failed to start camera.")
        ```
    
    """
    _compute_point_cloud: Synchronized = None
    _show: Synchronized = None
    _camera_process: Process = None
    _manager: SyncManager = None
    _lock_camera: Lock  = None
    _trackers_pos: ListProxy = None
    _point_cloud: ListProxy = None
    _tracking: Synchronized = None
    _running: Synchronized = None
    _parameter: DictProxy = {"hue_h": 90, "hue_l": 36, "sat_h": 255, "sat_l": 138, "value_h": 255, "value_l": 35, "erosion_size": 0, "area": 100}
    _hsv_frame: ListProxy = None
    _mask_frame: ListProxy = None
    _camera_serial: Synchronized = None


    def __init__(self, camera_serial=None, parameter=None, show=False, tracking=True, compute_point_cloud=False):
        """
        Initialize the camera.
        Args:
            camera_name: str: The name of the camera to connect to. If None, the first camera found will be used.
            parameter: dict:  The camera parameters. If None, the lastest save paramters are used from a file, but if no file is found, default values will be used.
            show: bool:  Whether to show the camera HSV and Mask frames or not.
            tracking: bool:  Whether to track objects or not.
            compute_point_cloud: bool: Whether to compute the point cloud or not.
        """
        multiprocessing.freeze_support()
        self._manager = multiprocessing.Manager()
        self._lock_camera = multiprocessing.Lock()
        self._trackers_pos = self._manager.list()
        self._point_cloud = self._manager.list()
        self._hsv_frame = self._manager.list()
        self._mask_frame = self._manager.list()
        self._camera_serial = multiprocessing.Value(c_wchar_p, None)
        self._running = multiprocessing.Value('b', False)
        self._tracking = multiprocessing.Value('b', tracking)
        self._show = multiprocessing.Value('b', show)
        self._compute_point_cloud = multiprocessing.Value('b', compute_point_cloud)
        self._parameter = self._manager.dict()
        if parameter is not None:
            self._parameter.update(parameter)



    ##########################
    #  PROPERTIES
    ##########################



    @property
    def camera_serial(self) -> str:
        """
        Get the current camera serial number
        """
        return self._camera_serial.value
    

    @property
    def is_running(self) -> bool:
        """
        Get the running status of the camera.
        Returns:
            bool: The running status of the camera.
        """
        return self._running.value
    

    @property
    def track_markers(self) -> bool:
        """
        Get whether the camera is tracking objects or not.
        Returns:
            bool: True if the camera is tracking the markers, else False.
        """
        return self._tracking.value
    

    @track_markers.setter
    def track_markers(self, value: bool):
        """
        Set the tracking status of the camera.
        Args:
            value: bool: The new tracking status.
        """
        self._tracking.value = value

    @property
    def compute_point_cloud(self) -> bool:
        """
        Get whether the camera is computing the point cloud or not.
        Returns:
            bool: True if the camera is computing the point cloud, else False.
        """
        return self._compute_point_cloud.value
    

    @compute_point_cloud.setter
    def compute_point_cloud(self, value: bool):
        """
        Set the point cloud computation status of the camera.
        Args:
            value: bool: The new point cloud computation status.
        """
        self._compute_point_cloud.value = value

    
    @property
    def show_frames(self) -> bool:
        """
        Get the show status of the camera.
        Returns:
            bool: The show status of the camera.
        """
        return self._show.value
    

    @show_frames.setter
    def show_frames(self, value: bool):
        """
        Set the show status of the camera.
        Args:
            value: bool: The new show status.
        """
        self._show.value = value

    
    @property
    def parameters(self) -> dict:
        """
        Get the camera parameters in a dict:
            - `hue_h`: int: The upper hue value.
            - `hue_l`: int: The lower hue value.
            - `sat_h`: int: The upper saturation value.
            - `sat_l`: int: The lower saturation value.
            - `value_h`: int: The upper value value.
            - `value_l`: int: The lower value value.
            - `erosion_size`: int: The size of the erosion kernel.
            - `area`: int: The minimum area of the detected objects.

        Returns:
            dict: The camera parameters.
        """
        return self._parameter
    

    @parameters.setter
    def parameters(self, value: dict):
        """
        Set the camera tracking parameters from the dict:
            - `hue_h`: int: The upper hue value.
            - `hue_l`: int: The lower hue value.
            - `sat_h`: int: The upper saturation value.
            - `sat_l`: int: The lower saturation value.
            - `value_h`: int: The upper value value.
            - `value_l`: int: The lower value value.
            - `erosion_size`: int: The size of the erosion kernel.
            - `area`: int: The minimum area of the detected objects.

        :::warning
        - The camera parameters are not saved to a file. You need to save them manually.
        - The paramters are set when opening the camera. To change the parameters programatically, you need to close the camera and open it again with the wanted parameters.
        :::

        Args:
            value: dict: The new camera parameters.
        """
        if value is not None:
            self._parameter.update(value)
        else:
            self._parameter.clear()
    

    @property
    def trackers_pos(self) -> list:
        """
        Get the positions of the trackers.
        Returns:
            list: The positions of the trackers as a list of lists.
        """
        with self._lock_camera:
            if self._tracking:
                return self._trackers_pos
            else:
                return []
    
    @property
    def point_cloud(self) -> np.ndarray:
        """
        Get the point cloud data.
        Returns:
            The point cloud data as a numpy array.
        """
        with self._lock_camera:
            if self._compute_point_cloud:
                return self._point_cloud[0]
            else:
                return np.array([])

    
    @property
    def hsv_frame(self) -> np.ndarray | None:
        """
        Get the HSV frame.
        Returns:
            The HSV frame as a numpy array.
        """
        with self._lock_camera:
            if self._hsv_frame:
                return self._hsv_frame[0]
            else:
                return None
    

    @property
    def mask_frame(self) -> np.ndarray | None:
        """
        Get the mask frame.
        Returns:
            The mask frame as a numpy array.
        """
        with self._lock_camera:
            if self._mask_frame:
                return self._mask_frame[0]
            else:
                return None
            


    ##########################
    #  METHODS
    ##########################



    @staticmethod
    def listCameras():
        """
        Static method to list all the Realsense cameras connected to the computer

        Returns:
            list: A list of the serial numbers as string.
        """
        return list_cameras()


    def __getstate__(self):
        """
        Get the state of the object for pickling.
        This method is used to remove the _manager attribute from the object state based on https://laszukdawid.com/blog/2017/12/13/multiprocessing-in-python-all-about-pickling/
        """
        self_dict = self.__dict__.copy()
        del self_dict['_manager']
        return self_dict


    def open(self, camera_serial: str=None) -> bool:
        """
        Initialize and open the camera in another process.
        This function creates a new process to handle the camera and starts it.
        """
        if self._running.value:
             self._camera_process.terminate()

        if camera_serial is not None:
            self._camera_serial.value = camera_serial


        self._camera_process = Process(target=self._processCamera, args=(self._running, 
                                                                            self._tracking, 
                                                                            self._show, 
                                                                            self._compute_point_cloud, 
                                                                            self._trackers_pos, 
                                                                            self._point_cloud, 
                                                                            self._camera_serial,
                                                                            self._parameter,
                                                                            self._hsv_frame,
                                                                            self._mask_frame))
        self._camera_process.start()

        timeout = time.time() + 5

        while not self._running.value:
            time.sleep(0.5)
            if time.time() > timeout:
                logger.error("Camera process did not start within the timeout period. Exiting.")
                self.close()
                return False
            continue

        return True


    def _processCamera(self, running: Synchronized, tracking: Synchronized, show: Synchronized, 
                       compute_point_cloud: Synchronized, trackers_pos: ListProxy, 
                       point_cloud: ListProxy, camera_serial: Synchronized=None, parameter: DictProxy=None, hsv_frame: ListProxy=None, mask_frame: ListProxy=None):
        """
        Process to handle the camera.
        This function runs in a separate process and updates the camera frames.
        Args:
            running: bool: A boolean indicating whether the camera is running or not.
            tracking: bool: A boolean indicating whether to track objects or not.
            show: bool: A boolean indicating whether to show the camera frames or not.
            trackers_pos: list: A list to store the positions of the trackers.
            point_cloud: list: A list to store the point cloud data.
            parameter: dict: The camera parameters.
            hsv_frame: list: A list to store the HSV frame.
            mask_frame: list: A list to store the mask frame.
        """

        logger.debug("Starting camera {} process with show: {}, tracking: {}, compute_point_cloud: {}".format(camera_serial.value, show.value, tracking.value, compute_point_cloud.value))
        camera = depthcamera.DepthCamera(camera_serial=camera_serial.value, parameter=parameter, compute_point_cloud=compute_point_cloud.value, show_video_feed=show.value, tracking=tracking.value)
        parameter.update(camera.parameter)
        # camera_serial.value = "Test1"

        running.value = True
        while running.value:
            with self._lock_camera:
                camera.compute_point_cloud = compute_point_cloud.value
                camera.tracking = tracking.value

                camera.update()

                show.value = camera.show_video_feed
                
                del hsv_frame[:]
                hsv_frame.append(camera.hsvFrame)
                
                del mask_frame[:]
                mask_frame.append(camera.maskFrame)

                if tracking:
                    del trackers_pos[:]
                    trackers_pos.extend(camera.trackers_pos)

                if compute_point_cloud:
                    del point_cloud[:]
                    point_cloud.append(camera.point_cloud)
                

        camera.close()
        running.value = False

        
    def close(self):
        """
        Close the camera and terminate the process. Sets the running status to False.
        """
        self._running.value = False
        if self._camera_process.is_alive():
            self._camera_process.terminate()
