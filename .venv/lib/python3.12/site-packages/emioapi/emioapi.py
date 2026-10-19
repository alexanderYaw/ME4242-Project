"""
[Emio API](https://github.com/SofaComplianceRobotics/Emio.API) is a Python package created to control the [Emio robot](https://compliance-robotics.com/compliance-lab/).

To install it from PyPI, run the following command: 
```bash
pip install emioapi
```

To install the latest version from the Github repository, run:  
```bash
pip install git+https://github.com/SofaComplianceRobotics/Emio.API.git@main
```

## Camera Calibration Tool
Once emioapi installed, you can directly call the calibration tool to calibrate the camera of Emio.
From a terminal with a Python having the emioapi moduules intalled, run:
```bash
python -m emioapi calibrate
```
"""

from dataclasses import field

from threading import Lock

from emioapi import EmioMotors, motorgroup
from emioapi import MultiprocessEmioCamera
from emioapi import EmioCamera, emiocamera
from emioapi._logging_config import logger

class EmioAPI:
    """
    Class to control emio motors. 
    It is essentially divided into two objects:
    - The [`motors`](#motors) object ([`EmioMotors`](#emiomotors)), which is used to control the motors.
    - The [`camera`](#camera) object ([`EmioCamera`](#emiocamera)), which is used to control the camera.


    The EmioAPI class is the main class that combines both classes and provides a simple interface to control the emio device.
    It also provides static utility methods to list the emio devices connected to the computer.
    
    Motors:
        > The motors are controlled in position mode. The class is thread-safe and can be used in a multi-threaded environment.
        > All the data sent to the motors are list of *4 values* for the *4 motors* of the emio device. The order in the list corresponds to the motor ID's in the emio device.
        > Motor 0 is the first motor in the list, motor 1 is the second motor, etc.
        > You can open a connection directly to the motors using the [`open`](#opendevice_name-str--none) method of the `motors` object.
        > 
        > :::warning 
        > 
        > Emio motors are clamped between 0 and PI radians (0 and 180 degrees). If you input a value outside this range, the motor will not move.
        > 
        > :::

    Camera:
        > The camera is used to track objects and compute the point cloud.
        > The camera parameters are stored in a config file. If the config file is not found, default values are used.
        > The camera can be configured to show the frames, track objects, and compute the point cloud.
        > You can open a connection directly to the camera using the [`open`](#open) method of the `camera` object.
        >
        > :::warning
        > By default, EmioAPI launches the camera in the same process by creating an [EmioCamera](#emiocamera) object.
        > You can launch the camera in another process using a [MultiProcessEmioCamera](#multiprocessemiocamera) by setting the `multiprocess_camera=True` when creating an `EmioAPI` object.
        > :::


    Example:
        ```python
        from emioapi import EmioAPI

        # Create an EmioAPI instance
        emio = EmioAPI(multiprocess_camera=False)

        # Connect to the first available Emio device
        if emio.connectToEmioDevice():
            print("Connected to Emio device.")

            # Print device status
            emio.printStatus()

            # Example: Move all motors to 90 degrees (PI/2 radians)
            target_angles = [math.pi/2] * 4
            emio.motors.angles = target_angles

            # Disconnect when done
            emio.disconnect()
        else:
            print("Failed to connect to Emio device.")
        ```
        
    """

    _emio_list = {}  # Dict of all emio devices connected to the computer
    motors: EmioMotors = None  # The emio motors object: [`EmioMotors`](#emiomotors)
    camera: MultiprocessEmioCamera | EmioCamera = None  # The emio camera object: [`EmioCamera`](#emiocamera) | [`MultiprocessEmioCamera`](#multiprocessemiocamera)
    device_index: int= None


    @property
    def device_name(self) -> str | None:
        """
        Get the port name to which the EmioAPI object is connected if connected, else None
        """
        if self.motors.is_connected:
            return self.motors.device_name
        else:
            return None
    

    @property
    def camera_serial(self) -> str | None:
        """
        Get the camera serial number to which the EmioAPI object is connected if connected, else None
        """
        if self.camera.is_running:
            return self.camera.camera_serial
        else:
            return None
    


    #################
    #### METHODS ####
    #################



    def __init__(self, multiprocess_camera=False):
        self._lock = Lock()
        self.motors = EmioMotors()
        self.camera = MultiprocessEmioCamera() if multiprocess_camera else EmioCamera()


    @staticmethod
    def listEmioDevices() -> list:
        """
        List all the emio devices connected to the computer.
        
        Returns:
            A list of device names (the ports).
        """
        return motorgroup.listMotors()
    
    
    @staticmethod
    def listUnusedEmioDevices() -> list:
        """
        List all the emio devices that are not currently used by any instance of EmioAPI in this process.
        
        Returns:
            A list of device names (the ports).
        """
        return [device for device in EmioAPI.listEmioDevices() if device not in EmioAPI._emio_list]
    
    
    @staticmethod
    def listUsedEmioDevices() -> list:
        """
        List all the emio devices that are currently used by an instance of EmioAPI in this process.
        
        Returns:
            A list of device names (the ports).
        """
        return [device for device in EmioAPI._emio_list.keys()]
    
    @staticmethod
    def listCameraDevices():
        return EmioCamera.listCameras()
    
    
    def connectToEmioDevice(self, device_name: str=None) -> bool:
        """
        Connect to the emio device with the given name.
        
        Args:
            device_name: str: The name of the device to connect to. If None, the first device found that is not used will be used.

        Returns:
            True if the connection is successful, False otherwise.
        """
        connected_index = self.motors.findAndOpen(device_name)

        if connected_index>=0: # try to connect to the camera if foud a Emio to connect to
            self.device_index = connected_index
            EmioAPI._emio_list[self.motors.device_name] = self
            camera_serial = EmioAPI.listCameraDevices()[self.device_index]

            logger.info(f"Connecting to emio number {self.device_index} on port: {self.motors.device_name} with camera serial: {camera_serial}")

            if self.camera.open(camera_serial):
                return True
        return False
    

    def disconnect(self):
        """Close the connection to motors and camera."""
        logger.debug("Closing the connection to the motors and camera.")
        with self._lock:
            self.motors.close()
            logger.debug("Motors connection closed.")

            self.camera.close()
            logger.debug("Camera closed.")

            EmioAPI._emio_list.pop(self.motors.device_name, None)
            logger.info(f"Disconnected from emio device: {self.motors.device_name}")


    def printStatus(self):
        """
        Print the status of the Emio device.
        """
        with self._lock:
            if self.motors.is_connected:
                logger.info(f"Connected to Emio device: {self.motors.device_name}")
            else:
                logger.info("No Emio connected.")
        