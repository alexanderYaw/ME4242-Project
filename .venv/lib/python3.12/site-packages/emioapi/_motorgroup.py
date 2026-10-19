from dynamixel_sdk import *

import emioapi._emiomotorsparameters as MotorsParametersTemplate
from emioapi._logging_config import logger

def listMotors():
    """
    List all the emio devices connected to the computer.

    Returns:
        A list containing the devices name (string)
    """
    ports = []
    comports = serial.tools.list_ports.comports()

    for p in comports:
        if p.manufacturer is not None and "FTDI" in p.manufacturer:
            ports.append(p.device)
        elif p.description is not None and "FTDI" in p.description:
            ports.append(p.device)
        elif p.serial_number is not None and "FTDI" in p.serial_number:
            ports.append(p.device)

    if ports is None or len(ports) == 0:
        logger.warning("No motor found. Please check the connection.")
        return ports

    return ports


def getDevicePort(entry, method="manufacturer"):
        """
        Get the device port based on the device name and method. This will get the first FTDI device found.

        Args:
            entry (str): The name of the device to search for.
            method (str): The method to use for searching (default is "manufacturer").
        Returns:
            The first port of the device if found, otherwise None.
        """
        ports = []
        comports = serial.tools.list_ports.comports()

        if comports is None or len(comports) == 0:
            logger.error("Serial ports check failed, list of ports is empty.")
            return

        if method == "manufacturer":
            ports = [p for p in comports if p.manufacturer is not None and entry in p.manufacturer]
        if method == "description":
            ports = [p for p in comports if p.description is not None and entry in p.description]
        if method == "serial_number":
            ports = [p for p in comports if p.serial_number is not None and entry in p.serial_number]

        if not ports:
            logger.error("No serial port found with " + method + " = " + entry)
            return

        if len(ports) > 1:
            logger.warning("Multiple port found with " + method + " = " + entry + ". Using the first.")

        logger.debug("Found port with " + method + " = " + entry + ": \n" +
                    "device : " + ports[0].device + "\n" +
                    "manufacturer : " + ports[0].manufacturer + "\n" +
                    "description : " + ports[0].description + "\n" +
                    "serial number : " + ports[0].serial_number
                    )
        return ports[0].device


def _valToArray( val):
    """Convert a 32-bit integer to a list of 4 bytes.
    Args:
        val (int): The 32-bit integer to convert.
    Returns:
        list of bytes: The list of 4 bytes representing the integer.
    """
    return [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)), DXL_LOBYTE(DXL_HIWORD(val)),
            DXL_HIBYTE(DXL_HIWORD(val))]


def _valTo2Bytes(val):
    """Convert a 16-bit integer to a list of 2 bytes."""
    return [DXL_LOBYTE(val), DXL_HIBYTE(val)]


class DisconnectedException(Exception):
    """Custom exception for disconnected motors."""
    def __init__(self):
        message = "MotorGroup is not connected. It is either disconnected or permission denied."
        super().__init__(message)

class MotorGroup:

    def __init__(self, parameters: MotorsParametersTemplate) -> None:

        self.parameters = parameters
        self.deviceName = None

        self.packetHandler = PacketHandler(self.parameters.PROTOCOL_VERSION)
        self.portHandler = PortHandler(self.deviceName)

        self.groupReaders = {}
        self.groupWriters = {}

        self.groupReaders["position"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                        self.parameters.ADDR_PRESENT_POSITION,
                                                        self.parameters.LEN_PRESENT_POSITION)
        self.groupReaders["velocity"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                       self.parameters.ADDR_PRESENT_VELOCITY,
                                                       self.parameters.LEN_PRESENT_VELOCITY)
        self.groupReaders["goal_position"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_GOAL_POSITION,
                                                            self.parameters.LEN_GOAL_POSITION)
        self.groupReaders["goal_velocity"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                        self.parameters.ADDR_GOAL_VELOCITY,
                                                        self.parameters.LEN_GOAL_VELOCITY)
        self.groupReaders["moving"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                         self.parameters.ADDR_MOVING,
                                                         1)
        self.groupReaders["moving_status"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_MOVING_STATUS,
                                                            1)
        self.groupReaders["velocity_trajectory"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_VELOCITY_TRAJECTORY,
                                                            self.parameters.LEN_VELOCITY_TRAJECTORY)
        self.groupReaders["position_trajectory"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_POSITION_TRAJECTORY,
                                                            self.parameters.LEN_POSITION_TRAJECTORY)
        self.groupReaders["position_p_gain"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_P_GAIN,
                                                                self.parameters.LEN_POSITION_P_GAIN)
        self.groupReaders["position_i_gain"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_I_GAIN,
                                                                self.parameters.LEN_POSITION_I_GAIN)
        self.groupReaders["position_d_gain"] = GroupSyncRead(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_D_GAIN,
                                                                self.parameters.LEN_POSITION_D_GAIN)

        self.groupWriters["goal_position"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_GOAL_POSITION,
                                                            self.parameters.LEN_GOAL_POSITION)
        self.groupWriters["goal_velocity"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                            self.parameters.ADDR_GOAL_VELOCITY,
                                                            self.parameters.LEN_GOAL_POSITION)
        self.groupWriters["velocity_profile"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_VELOCITY_PROFILE,
                                                                self.parameters.LEN_GOAL_POSITION)
        self.groupWriters["position_p_gain"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_P_GAIN,
                                                                self.parameters.LEN_POSITION_P_GAIN)
        self.groupWriters["position_i_gain"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_I_GAIN,
                                                                self.parameters.LEN_POSITION_I_GAIN)
        self.groupWriters["position_d_gain"] = GroupSyncWrite(self.portHandler, self.packetHandler,
                                                                self.parameters.ADDR_POSITION_D_GAIN,
                                                                self.parameters.LEN_POSITION_D_GAIN)

        for DXL_ID in self.parameters.DXL_IDs:
            for group in self.groupReaders.values():
                group.addParam(DXL_ID)


    @property
    def isConnected(self):
        """Check if the motor group is connected."""
        try:
            if self.portHandler and self.portHandler.is_open  and self._isDeviceDetected():
                return True
        except Exception as e:
            logger.exception(f"Failed to check connection: {e}")
            return False

    def _updateGroups(self):
        """
        Update the port handler with the new device name.
        """
        for group in self.groupReaders.values():
            group.port = self.portHandler
            group.ph = self.packetHandler

        for group in self.groupWriters.values():
            group.port = self.portHandler
            group.ph = self.packetHandler

    def updateDeviceName(self, device_name: str=None):
        """
        Update the device name based on the available ports. This will get the first FTDI device found if no device name is provided.
        If no device is found, the device name will be None.
        """
        self.deviceName = device_name  if device_name is not None else getDevicePort("FTDI", method="manufacturer")
        self.portHandler = PortHandler(self.deviceName)
        self._updateGroups()

        logger.debug(f"Device name updated to: {self.deviceName}")

        return

    def _isDeviceDetected(self):
        for port in serial.tools.list_ports.comports():
            if port.device == self.deviceName:
                return True
        return False


    def _readMotorsData(self, groupSyncRead:GroupSyncRead):
        """Read data from the motor.

        Args:
            DXL_ID (int): The ID of the motor.
            addr (int): The address to read from.
            length (int): The length of the data to read.

        Returns:
            int: The value read from the motor.

        Raises:
            Exception: If the motor group is not connected or if the read operation fails.
        """
        if not self.isConnected:
            raise DisconnectedException()

        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            raise Exception(f"Failed to read data from motor: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        result = list()

        for DXL_ID in self.parameters.DXL_IDs:
            dxl_getdata_result = groupSyncRead.isAvailable(DXL_ID, groupSyncRead.start_address, groupSyncRead.data_length)
            if dxl_getdata_result != True:
                return None
            result.append(groupSyncRead.getData(DXL_ID, groupSyncRead.start_address, groupSyncRead.data_length))

        return result


    def setOperatingMode(self, mode):
        """Set the operating mode of the motors.
        Args:
            mode (int): The operating mode to set.
                0: Current Control Mode
                1: Velocity Control Mode
                3: (Default) Position Control Mode
                4: Extended Position Control Mode
                5: Current-bqsed Position Control Mode
                16: PWM Control Mode

                See https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/#operating-mode for more details.
        """
        if not self.isConnected:
            raise DisconnectedException()

        for DXL_ID in self.parameters.DXL_IDs:
            value = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, self.parameters.ADDR_OPERATING_MODE)
            if value != mode:
                logger.debug("Motor mode changed to mode %s (%s,%s)" % (mode, self.deviceName, DXL_ID))
                self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.parameters.ADDR_OPERATING_MODE, mode)


    def setInVelocityMode(self):
        self.setOperatingMode(self.parameters.VELOCITY_MODE)


    def setInExtendedPositionMode(self):
        self.setOperatingMode(self.parameters.EXT_POSITION_MODE)


    def setInPositionMode(self):
        self.setOperatingMode(self.parameters.POSITION_MODE)


    def __writeMotorsData(self, group: GroupSyncWrite, values):
        """Helper function to write data to the motors.
        Args:
            group (GroupSyncWrite): The group sync write object.
            values (list of numbers): The values to write to the motors.
        """
        if not self.isConnected:
            raise DisconnectedException()

        group.clearParam()
        for index, DXL_ID in enumerate(self.parameters.DXL_IDs):
            if group.data_length == 2:
                data = _valTo2Bytes(values[index])
            elif group.data_length == 4:
                data = _valToArray(values[index])
            else:
                raise Exception(f"Unsupported data length: {group.data_length}")
            group.addParam(DXL_ID, data)
        group.txPacket()


    def setGoalVelocity(self, speeds):
        """Set the goal velocity

        Args:
            speeds (list of numbers): unit depends on motor type
        """
        self.__writeMotorsData(self.groupWriters["goal_velocity"] , speeds)


    def setGoalPosition(self, positions):
        """Set the goal position

        Args:
            positions (list of numbers): unit = 1 pulse
        """
        self.__writeMotorsData(self.groupWriters["goal_position"], positions)


    def setVelocityProfile(self, max_vel):
        """Set the maximum velocities in position mode

        Args:
            positions (list of numbers): unit depends on the motor type
        """
        self.__writeMotorsData(self.groupWriters["velocity_profile"], max_vel)


    def setPositionPGain(self, p_gains):
        """Set the position P gains

        Args:
            p_gains (list of numbers): unit depends on the motor type
        """
        self.__writeMotorsData(self.groupWriters["position_p_gain"], p_gains)


    def setPositionIGain(self, i_gains):
        """Set the position I gains

        Args:
            i_gains (list of numbers): unit depends on the motor type
        """
        self.__writeMotorsData(self.groupWriters["position_i_gain"], i_gains)


    def setPositionDGain(self, d_gains):
        """Set the position D gains

        Args:
            d_gains (list of numbers): unit depends on the motor type
        """
        self.__writeMotorsData(self.groupWriters["position_d_gain"], d_gains)


    def getCurrentPosition(self) -> list:
        """Get the current position of the motors
        Returns:
            list of numbers: unit = 1 pulse
        """
        return self._readMotorsData(self.groupReaders["position"])


    def getGoalPosition(self) -> list:
        """Get the goal position of the motors
        Returns:
            list of numbers: unit = 1 pulse
        """
        return self._readMotorsData(self.groupReaders["goal_position"])

    def getGoalVelocity(self) -> list:
        """Get the goal velocity of the motors
        Returns:
            list of velocities: unit is rev/min
        """
        return self._readMotorsData(self.groupReaders["goal_velocity"])


    def getCurrentVelocity(self) -> list:
        """Get the current velocity of the motors
        Returns:
            list of velocities: unit is rev/min
        """
        return self._readMotorsData(self.groupReaders["velocity"])


    def isMoving(self) -> list:
        """Check if the motors are moving
        Returns:
            list of booleans: True if the motor is moving, False otherwise
        """
        return self._readMotorsData(self.groupReaders["moving"])


    def getMovingStatus(self) -> list:
        """Get the moving status of the motors
        Returns:
            list of booleans: True if the motor is moving, False otherwise
        """
        return self._readMotorsData(self.groupReaders["moving_status"])


    def getVelocityTrajectory(self) -> list:
        """Get the velocity trajectory of the motors
        Returns:
            list of velocities: unit is rev/min
        """
        return self._readMotorsData(self.groupReaders["velocity_trajectory"])


    def getPositionTrajectory(self) -> list:
        """Get the position trajectory of the motors
        Returns:
            list of positions: unit = 1 pulse
        """
        return self._readMotorsData(self.groupReaders["position_trajectory"])


    def getPositionPGain(self) -> list:
        """Get the position P gains of the motors
        Returns:
            list of P gains: unit depends on the motor type
        """
        return self._readMotorsData(self.groupReaders["position_p_gain"])


    def getPositionIGain(self) -> list:
        """Get the position I gains of the motors
        Returns:
            list of I gains: unit depends on the motor type
        """
        return self._readMotorsData(self.groupReaders["position_i_gain"])


    def getPositionDGain(self) -> list:
        """Get the position D gains of the motors
        Returns:
            list of D gains: unit depends on the motor type
        """
        return self._readMotorsData(self.groupReaders["position_d_gain"])


    def open(self) -> None:
        """Open the port and set the baud rate.
        Raises:
            Exception: If the port cannot be opened or the baud rate cannot be set.
        """
        try:
            self.portHandler.openPort()
            self.portHandler.setBaudRate(self.parameters.BAUDRATE)
        except Exception as e:
            raise Exception(f"Failed to open port: {e}")


    def enableTorque(self):
        """Enable the torque of the motors."""
        if not self.isConnected:
            raise DisconnectedException()

        for DXL_ID in self.parameters.DXL_IDs:
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.parameters.ADDR_TORQUE_ENABLE,
                                              self.parameters.TORQUE_ENABLE)

    def close(self) -> None:
        """Close the port and disable the torque of the motors."""
        try:
            for DXL_ID in self.parameters.DXL_IDs:
                self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.parameters.ADDR_TORQUE_ENABLE,
                                                  self.parameters.TORQUE_DISABLE)
            self.portHandler.closePort()
            self.deviceName = None
        except Exception as e:
            raise Exception(f"Failed to close port: {e}")

    def clearPort(self) -> None:
        """Clear the port."""
        if not self.isConnected:
            raise DisconnectedException()

        if self.portHandler:
            self.portHandler.clearPort()
