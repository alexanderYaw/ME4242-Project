from dataclasses import field
from threading import Lock

import emioapi._motorgroup as motorgroup
import emioapi._emiomotorsparameters as emioparameters
from emioapi._logging_config import logger

class EmioMotors:
    """
    Class to control emio motors.
    The class is designed to be used with the emio device.
    The motors are controlled in position mode. The class is thread-safe and can be used in a multi-threaded environment.

    Example:
        ```python
        from emioapi import EmioMotors

        # Create an instance of EmioMotors
        motors = EmioMotors()

        # Open connection to the motors (optionally specify device name)
        if motors.open():
            # Print current angles in radians
            print("Current angles (rad):", motors.angles)

            # Set new goal angles (example values)
            motors.angles = [0.5, 1.0, -0.5, 1.0]

            # Print status
            motors.printStatus()

            # Close connection when done
            motors.close()
        else:
            print("Failed to connect to motors.")
        ```

    """

    _initialized: bool = False
    _length_to_rad: float = 1.0 / 20.0  # 1/radius of the pulley
    _rad_to_pulse: int = 4096 / (2 * 3.1416)  # the resolution of the Dynamixel xm430 w210
    _length_to_pulse: int = _length_to_rad * _rad_to_pulse
    _pulse_center: int = 2048
    _max_vel: float = 1000  # *0.01 rev/min
    _goal_velocity: list = field(default_factory=lambda: [0] * len(emioparameters.DXL_IDs))
    _goal_position: list = field(default_factory=lambda: [0] * len(emioparameters.DXL_IDs))
    _mg: motorgroup.MotorGroup = None
    _device_index: int = None



    #####################
    ###### METHODS ######
    #####################



    def __init__(self):
        self._lock = Lock()
        if not self._initialized:
            self._mg = motorgroup.MotorGroup(emioparameters)
            self._initialized = True


    def lengthToPulse(self, displacement: list):
        """
        Convert length (mm) to pulse using the conversion factor `lengthToPulse`.

        Args:
            displacement: list: list of length values in mm for each motor.

        Returns:
            A list of pulse values for each motor.
        """
        return [self._pulse_center - int(item * self._length_to_pulse) for item in displacement]


    def pulseToLength(self, pulse: list):
        """
        Convert pulse to length (mm) using the conversion factor `lengthToPulse`.

        Args:
            pulse: list of pulse integer values for each motor.

        Returns:
            A list of length values in mm for each motor.
        """
        return [(self._pulse_center - float(item)) / self._length_to_pulse for item in pulse]


    def pulseToRad(self, pulse: list):
        """
        Convert pulse to radians using the conversion factor `radToPulse`.

        Args:
            pulse: list: list of pulse integer values for each motor.

        Returns:
            A list of angles in radians for each motor.

        """
        return [(self._pulse_center - float(item)) / self._rad_to_pulse for item in pulse]


    def pulseToDeg(self, pulse: list):
        """
        Convert pulse to degrees using the conversion factor `radToPulse`.

        Args:
            pulse: list: list of pulse values for each motor.

        Returns:
            A list of angles in degrees for each motor.
        """
        return [(self._pulse_center - float(item)) / self._rad_to_pulse * 180.0 / 3.1416 for item in pulse]


    def _openAndConfig(self, device_name: str=None) -> bool:
        """Open the connection to the motors, configure it for position mode and enable torque sensing."""
        with self._lock:
            try:
                self._mg.updateDeviceName(device_name)

                if self._mg.deviceName is None:
                    logger.error("Device name is None. Please check the connection.")
                    return False

                self._mg.open()
                self._mg.clearPort()
                self._mg.setInPositionMode()
                self._mg.enableTorque()

                logger.debug(f"Motor group opened and configured. Device name: {self._mg.deviceName}")
                return True
            except Exception as e:
                logger.error(f"Failed to open and configure the motor group: {e}")
                return False


    def open(self, device_name: str=None) -> bool:
        """
        Open the connection to the motors.

        Args:
            device_name: str: if set, it will connect to the device with the given name, If not set, the first emio device will be used.
        """
        if self._openAndConfig(device_name):
            self._device_index = motorgroup.listMotors().index(self._mg.deviceName)
            logger.info(f"Connected to emio device: {self._mg.deviceName}")
            return True
        return False


    def findAndOpen(self, device_name: str=None) -> int:
        """
        Iterate over the serial ports and try to conenct to the first available emio motors.

        Args:
            device_name: str: If set, It will try to connected to the given device name (port name)

        Returns:
            the index in the list of port to which it connected. If no connection was possible, returns -1.
        """
        if device_name is not None:
            try:
                index = motorgroup.listMotors().index(device_name)
                logger.info(f"Trying given emio number {index} on port: {device_name}.")
                self.open(device_name)
                return index if len(motorgroup.listMotors())>0 and self.open(device_name) else -1
            except:
                return -1

        index = 0

        connected = False
        try:

            while not connected and index<len(motorgroup.listMotors()):
                device_name = motorgroup.listMotors()[index]
                logger.info(f"Trying emio number {index} on port: {device_name}.")
                connected = self.open(device_name)

                if connected:
                    self._device_index = index
                    return self.device_index
                index += 1
        except:
            return -1
        return -1


    def close(self):
        """Close the connection to the motors."""
        with self._lock:
            self._mg.close()
            logger.info("Motors connection closed.")


    def printStatus(self):
        """Print the current position of the motors."""
        with self._lock:
            logger.info(f"Current position of the motors in radians: {[ a%3.14 for a in self.pulseToRad(self._mg.getCurrentPosition())]}")



    ####################
    #### PROPERTIES ####
    ####################



    #### Read and Write properties ####
    @property
    def relativePos(self, init_pos: list, rel_pos: list):
        """
        Calculate the new position of the motors based on the initial position and relative position in pulses.

        Args:
            init_pos: list of initial pulse values for each motor.
            rel_pos: list of relative pulse values for each motor.

        Returns:
            A list of new pulse values for each motor.
        """
        new_pos = []
        for i in range(len(init_pos)):
            new_pos.append(init_pos[i] + rel_pos[i])
        return new_pos


    @property
    def angles(self) -> list:
        """Get the current angles of the motors in radians."""
        with self._lock:
            return self.pulseToRad(self._mg.getCurrentPosition())

    @angles.setter
    def angles(self, angles: list):
        """Set the goal angles of the motors in radians."""
        with self._lock:
            self._goal_position = angles
            self._mg.setGoalPosition([int(self._pulse_center - self._rad_to_pulse * a) for a in angles])


    @property
    def goal_velocity(self) -> list:
        """Get the current velocity (rev/min) of the motors."""
        return self._goal_velocity

    @goal_velocity.setter
    def goal_velocity(self, velocities: list):
        """Set the goal velocity (rev/min) of the motors."""
        self._goal_velocity = velocities
        with self._lock:
            self._mg.setGoalVelocity(velocities)


    @property
    def max_velocity(self)-> list:
        """Get the current velocity (rev/min) profile of the motors."""
        return self._max_vel

    @max_velocity.setter
    def max_velocity(self, max_vel: list):
        """Set the maximum velocities (rev/min) in position mode.
        Arguments:
            max_vel: list of maximum velocities for each motor in rev/min.
        """
        self._max_vel = max_vel
        with self._lock:
            self._mg.setVelocityProfile(max_vel)


    @property
    def position_p_gain(self) -> list:
        """Get the current position P gains of the motors."""
        with self._lock:
            return self._mg.getPositionPGain()

    @position_p_gain.setter
    def position_p_gain(self, p_gains: list):
        """Set the position P gains of the motors.
        Arguments:
            p_gains: list of position P gains for each motor.
        """
        with self._lock:
            self._mg.setPositionPGain(p_gains)


    @property
    def position_i_gain(self) -> list:
        """Get the current position I gains of the motors."""
        with self._lock:
            return self._mg.getPositionIGain()

    @position_i_gain.setter
    def position_i_gain(self, i_gains: list):
        """Set the position I gains of the motors.
        Arguments:
            i_gains: list of position I gains for each motor.
        """
        with self._lock:
            self._mg.setPositionIGain(i_gains)


    @property
    def position_d_gain(self) -> list:
        """Get the current position D gains of the motors."""
        with self._lock:
            return self._mg.getPositionDGain()

    @position_d_gain.setter
    def position_d_gain(self, d_gains: list):
        """Set the position D gains of the motors.
        Arguments:
            d_gains: list of position D gains for each motor.
        """
        with self._lock:
            self._mg.setPositionDGain(d_gains)


    #### Read-only properties ####
    @property
    def is_connected(self) -> bool:
        """Check if the motors are connected."""
        with self._lock:
            return self._mg.isConnected


    @property
    def device_name(self) -> str:
        """Get the name of the device."""
        with self._lock:
            return self._mg.deviceName


    @property
    def device_index(self) -> int:
        """Get the index of the device in the list of Emio Devices from EmioAPI"""
        return self._device_index


    @property
    def moving(self) -> list:
        """Check if the motors are moving."""
        with self._lock:
            return self._mg.isMoving()


    @property
    def moving_status(self) -> list:
        """Get the moving status of the motors.
        Returns:
         A Byte encoding different informations on the moving status like whether the desired position has been reached or not, if the profile is in progress or not, the kind of Profile used.

        See [here](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#moving-status) for more details."""
        with self._lock:
            return self._mg.getMovingStatus()


    @property
    def velocity(self) -> list:
        """Get the current velocity (rev/min) of the motors."""
        with self._lock:
            return self._mg.getCurrentVelocity()


    @property
    def velocity_trajectory(self)-> list:
        """Get the velocity (rev/min) trajectory of the motors."""
        with self._lock:
            return self._mg.getVelocityTrajectory()


    @property
    def position_trajectory(self)-> list:
        """Get the position (pulse) trajectory of the motors."""
        with self._lock:
            return self._mg.getPositionTrajectory()
