"""
ZDT Stepper Motor Control Library Amended for Open-APEX Project
---------------------------------
Provides UART command builders and utilities for ZDT stepper motors.
Includes functions for parameter reading, control (velocity/position),
homing operations, and real-time position monitoring.

---------------------------------
Origin Code from stepper motor manufacturer: https://pan.baidu.com/s/1HLKVfbbok9VPvwhSLCTOdw
"""

import time
import struct

# Lookup Table
"""
    S_VER = 0       # Read firmware version and corresponding hardware version
    S_RL = 1        # Read phase resistance and inductance
    S_PID = 2       # Read PID parameters
    S_VBUS = 3      # Read bus voltage
    S_CPHA = 5      # Read phase current
    S_ENCL = 7      # Read encoder value after linear calibration
    S_TPOS = 8      # Read target position angle of the motor
    S_VEL = 9       # Read real-time motor speed
    S_CPOS = 10     # Read real-time motor position angle
    S_PERR = 11     # Read motor position error angle
    S_FLAG = 13     # Read enable/reached/stall status flag
    S_Conf = 14     # Read driver parameters
    S_State = 15    # Read system status parameters
    S_ORG = 16      # Read homing/homing failure status flag
"""

def Read_Sys_Params(addr, s):  # Read driver board parameters
    i = 0
    cmd = bytearray(16)
    cmd[i] = addr
    i += 1
    func_codes = {
        "S_VER": 0x1F,
        "S_RL": 0x20,
        "S_PID": 0x21,
        "S_VBUS": 0x24,
        "S_CPHA": 0x27,
        "S_ENCL": 0x31,
        "S_TPOS": 0x33,
        "S_VEL": 0x35,
        "S_CPOS": 0x36,
        "S_PERR": 0x37,
        "S_FLAG": 0x3A,
        "S_ORG": 0x3B,
        "S_Conf": 0x42,  # Read driver parameters, an additional sub-code 0x6C is required
        "S_State": 0x43,  # Read system status parameters, an additional sub-code 0x7A is required
    }
    if s in func_codes:
        cmd[i] = func_codes[s]
        i += 1
    cmd[i] = 0x6B
    i += 1
    return cmd[:i]

def Reset_CurPos_To_Zero(addr):  # Reset current position to zero
    cmd = bytearray(4)
    cmd[0] = addr  # Address
    cmd[1] = 0x0A  # Function code
    cmd[2] = 0x6D  # Sub-code
    cmd[3] = 0x6B  # Check byte
    return cmd

def Reset_Clog_Pro(addr):  # Release stall protection
    cmd = bytearray(4)
    cmd[0] = addr  # Address
    cmd[1] = 0x0E  # Function code
    cmd[2] = 0x52  # Sub-code
    cmd[3] = 0x6B  # Check byte
    return cmd

def Modify_Ctrl_Mode(addr, svF, ctrl_mode):  # Modify control mode
    cmd = bytearray(6)
    cmd[0] = addr  # Address
    cmd[1] = 0x46  # Function code
    cmd[2] = 0x69  # Sub-code
    cmd[3] = 0x01 if svF else 0x00  # Save flag, 1 = save, 0 = do not save
    cmd[4] = ctrl_mode  # Control mode
    cmd[5] = 0x6B  # Check byte
    return cmd

def En_Control(addr, state, snF):  # Enable motor at address, and enable multi-motor sync
    cmd = bytearray(16)
    cmd[0] = addr  # Address
    cmd[1] = 0xF3  # Function code
    cmd[2] = 0xAB  # Sub-code
    cmd[3] = 0x01 if state else 0x00  # Enable state, true=0x01, false=0x00
    cmd[4] = 0x01 if snF else 0x00  # Multi-motor sync flag, true=0x01, false=0x00
    cmd[5] = 0x6B  # Check byte
    return cmd[:6]

def Vel_Control(addr, dir, vel, acc, snF):  # Set speed/acceleration for motor
    cmd = bytearray(16)
    cmd[0] = addr  # Address
    cmd[1] = 0xF6  # Function code
    cmd[2] = dir  # Direction, 0=CW, other=CCW
    cmd[3] = (vel >> 8) & 0xFF  # Speed high byte (RPM)
    cmd[4] = vel & 0xFF  # Speed low byte (RPM)
    cmd[5] = acc  # Acceleration, note: 0 means start immediately
    cmd[6] = 0x01 if snF else 0x00  # Multi-motor sync flag
    cmd[7] = 0x6B  # Check byte
    return cmd[:8]

def Pos_Control(addr, dir, vel, acc, clk, raF, snF):  # Position control for motor
    cmd = bytearray(16)
    cmd[0] = addr  # Address
    cmd[1] = 0xFD  # Function code
    cmd[2] = dir  # Direction
    cmd[3] = (vel >> 8) & 0xFF  # Speed high byte (RPM)
    cmd[4] = vel & 0xFF  # Speed low byte (RPM)
    cmd[5] = acc  # Acceleration, note: 0 means start immediately
    cmd[6] = (clk >> 24) & 0xFF  # Pulse count (bit24-bit31)
    cmd[7] = (clk >> 16) & 0xFF  # Pulse count (bit16-bit23)
    cmd[8] = (clk >> 8) & 0xFF   # Pulse count (bit8-bit15)
    cmd[9] = clk & 0xFF          # Pulse count (bit0-bit7)
    cmd[10] = 0x01 if raF else 0x00  # Relative/absolute flag
    cmd[11] = 0x01 if snF else 0x00  # Multi-motor sync flag
    cmd[12] = 0x6B  # Check byte
    return cmd[:13]

def Stop_Now(addr, snF):  # Immediately stop the motor
    cmd = bytearray(5)
    cmd[0] = addr  # Address
    cmd[1] = 0xFE  # Function code
    cmd[2] = 0x98  # Sub-code
    cmd[3] = 0x01 if snF else 0x00  # Multi-motor sync flag
    cmd[4] = 0x6B  # Check byte
    return cmd

def Synchronous_motion(addr):  # Execute synchronous motion command
    cmd = bytearray(4)
    cmd[0] = addr  # Address
    cmd[1] = 0xFF  # Function code
    cmd[2] = 0x66  # Sub-code
    cmd[3] = 0x6B  # Check byte
    return cmd

def Origin_Set_O(addr, svF):  # Set homing zero-point position
    cmd = bytearray(5)
    cmd[0] = addr  # Address
    cmd[1] = 0x93  # Function code
    cmd[2] = 0x88  # Sub-code
    cmd[3] = 0x01 if svF else 0x00  # Save flag
    cmd[4] = 0x6B  # Check byte
    return cmd

def Origin_Modify_Params(addr, svF, o_mode, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, potF):  # Modify homing parameters
    cmd = bytearray(20)
    cmd[0] = addr  # Address
    cmd[1] = 0x4C  # Function code
    cmd[2] = 0xAE  # Sub-code
    cmd[3] = 0x01 if svF else 0x00  # Save flag
    cmd[4] = o_mode  # Homing mode
    cmd[5] = o_dir  # Homing direction
    cmd[6] = (o_vel >> 8) & 0xFF  # Homing speed high byte
    cmd[7] = o_vel & 0xFF  # Homing speed low byte
    cmd[8] = (o_tm >> 24) & 0xFF  # Homing timeout high byte
    cmd[9] = (o_tm >> 16) & 0xFF  # Homing timeout mid-high byte
    cmd[10] = (o_tm >> 8) & 0xFF  # Homing timeout mid-low byte
    cmd[11] = o_tm & 0xFF  # Homing timeout low byte
    cmd[12] = (sl_vel >> 8) & 0xFF  # Limit collision detection speed high byte
    cmd[13] = sl_vel & 0xFF  # Limit collision detection speed low byte
    cmd[14] = (sl_ma >> 8) & 0xFF  # Limit collision detection current high byte
    cmd[15] = sl_ma & 0xFF  # Limit collision detection current low byte
    cmd[16] = (sl_ms >> 8) & 0xFF  # Limit collision detection time high byte
    cmd[17] = sl_ms & 0xFF  # Limit collision detection time low byte
    cmd[18] = 0x01 if potF else 0x00  # Power-on auto homing flag
    cmd[19] = 0x6B  # Check byte
    return cmd

def Origin_Trigger_Return(addr, o_mode, snF):  # Trigger homing return
    cmd = bytearray(5)
    cmd[0] = addr  # Address
    cmd[1] = 0x9A  # Function code
    cmd[2] = o_mode  # Homing mode
    cmd[3] = 0x01 if snF else 0x00  # Multi-motor sync flag
    cmd[4] = 0x6B  # Check byte
    return cmd

def Origin_Interrupt(addr):  # Force interrupt homing
    cmd = bytearray(4)
    cmd[0] = addr  # Address
    cmd[1] = 0x9C  # Function code
    cmd[2] = 0x48  # Sub-code
    cmd[3] = 0x6B  # Check byte
    return cmd

def Receive_Data(uart):
    i = 0
    rxCmd = bytearray(128)
    lTime = cTime = time.ticks_ms()
    while True:
        if uart.any():
            if i < 128:
                rxCmd[i] = uart.read(1)[0]
                i += 1
                lTime = time.ticks_ms()
        else:
            cTime = time.ticks_ms()
            if time.ticks_diff(cTime, lTime) > 100:
                hex_data = " ".join(["{:02x}".format(b) for b in rxCmd[:i]])  # Convert to hex string with leading zeros
                hex_data = hex_data.strip("00 ")  # Remove invalid leading/trailing zeros
                if hex_data and hex_data[0] != "0":  # Ensure first character is not 0
                    hex_data = "0" + hex_data
                return hex_data, len(hex_data.replace(" ", "")) // 2  # Return data and data length

def Real_time_location(uart):
    # Define real-time position variable
    pos = 0.0
    Motor_Cur_Pos = 0.0
    # Read real-time motor position
    Read_Sys_Params(1, "S_CPOS")
    time.sleep_ms(1)
    # Command data is cached in data array, length is count
    data, count = Receive_Data(uart)
    data_hex = data.split()
    if count > 0 and data and int(data_hex[0], 16) == 0x01 and int(data_hex[1], 16) == 0x36:
        # Combine into uint32_t
        pos = struct.unpack(">I", bytes.fromhex("".join(data_hex[3:7])))[0]
        # Angle conversion
        Motor_Cur_Pos = float(pos) * 360.0 / 65536.0
        if int(data_hex[2], 16):
            Motor_Cur_Pos = -Motor_Cur_Pos
    else:
        pass
    print('Motor1: {:.1f}'.format(Motor_Cur_Pos))  # Print float value with 1 decimal place
    time.sleep_ms(1)
