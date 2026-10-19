#********* DYNAMIXEL Model definition *********
# https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/

#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V



# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.


DXL_IDs = (0, 1, 2, 3)

import serial.tools.list_ports as list_ports

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
BAUDRATE                    = 1000000


# Generic motor parameters

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
VELOCITY_MODE               = 1
POSITION_MODE               = 3
EXT_POSITION_MODE           = 4
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES': #from https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_GOAL_VELOCITY          = 104
    LEN_GOAL_VELOCITY           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    ADDR_PRESENT_VELOCITY       = 128
    LEN_PRESENT_VELOCITY        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    ADDR_OPERATING_MODE         = 11
    ADDR_VELOCITY_PROFILE       = 112
    ADDR_MOVING                 = 122
    ADDR_MOVING_STATUS          = 123
    ADDR_VELOCITY_TRAJECTORY    = 136
    LEN_VELOCITY_TRAJECTORY     = 4
    ADDR_POSITION_TRAJECTORY    = 140
    LEN_POSITION_TRAJECTORY     = 4
    ADDR_POSITION_P_GAIN        = 84
    LEN_POSITION_P_GAIN         = 2
    ADDR_POSITION_I_GAIN        = 82
    LEN_POSITION_I_GAIN         = 2
    ADDR_POSITION_D_GAIN        = 80
    LEN_POSITION_D_GAIN         = 2

elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    LEN_GOAL_POSITION           = 4
    ADDR_PRESENT_POSITION       = 611
    LEN_PRESENT_POSITION        = 4
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    raise Exception("not defined yet")

elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES': #from https://emanual.robotis.com/docs/en/dxl/p/pm42-010-s260-r/
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_GOAL_VELOCITY          = 552
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    ADDR_PRESENT_VELOCITY       = 576
    LEN_PRESENT_VELOCITY        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = -150000    # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000     # Refer to the Maximum Position Limit of product eManual
    ADDR_OPERATING_MODE         = 11
    ADDR_VELOCITY_PROFILE       = 560
    ADDR_MOVING                 = 570
    ADDR_MOVING_STATUS          = 571
    ADDR_VELOCITY_TRAJECTORY    = 584
    LEN_VELOCITY_TRAJECTORY     = 4
    ADDR_POSITION_TRAJECTORY    = 588
    LEN_POSITION_TRAJECTORY     = 4
