#!/usr/bin/env python

"""
  ArbotiX Driver: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2010 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import serial, time, sys, thread
from ax12 import *
from struct import unpack

# ArbotiX-specific instructions
AX_SIZE_POSE = 7
AX_LOAD_POSE = 8
AX_ARB_LOAD_SEQ = 9
AX_PLAY_SEQ = 10
AX_LOOP_SEQ = 11

# ArbotiX Driver
class ArbotiX:
    """ Class to open a serial port and control AX-12 servos 
    through an ArbotiX board or USBDynamixel. """
    def __init__(self, port="/dev/ttyUSB0",baud=38400, interpolation=False, direct=False, timeout = 0.1):
        """ This may throw errors up the line -- that's a good thing. """
        self.mutex = thread.allocate_lock()
        self.ser = serial.Serial()
        
        self.mutex.acquire()
        self.ser.baudrate = baud
        self.ser.port = port
        self.ser.timeout = timeout
        self.ser.open()
        self.mutex.release()

        self.error = 0
        self.hasInterpolation = interpolation
        self.direct = direct

    def getPacket(self, mode, id=-1, leng=-1, error=-1, params = None):
        """ Read a dynamixel return packet, iterative attempt. Returns error level. """ 
        # need a positive byte
        d = self.ser.read()
        #print ord(d),
        if d == '': 
            print "Fail Read"
            return None

        # now process our byte
        if mode == 0:           # get our first 0xFF
            if ord(d) == 0xff:   
                return self.getPacket(1)
            else:
                return self.getPacket(0)
        elif mode == 1:         # get our second 0xFF
            if ord(d) == 0xff:
                return self.getPacket(2)
            else:
                return self.getPacket(0)
        elif mode == 2:         # get id
            if d != 0xff:
                return self.getPacket(3, ord(d))
            else:              
                return self.getPacket(0)
        elif mode == 3:         # get length
            return self.getPacket(4, id, ord(d))
        elif mode == 4:         # read error    
            self.error = ord(d)
            if leng == 2:
                return self.getPacket(6, id, leng, ord(d), list())
            else:
                return self.getPacket(5, id, leng, ord(d), list())
        elif mode == 5:         # read params
            params.append(ord(d))
            if len(params) + 2 == leng:
                return self.getPacket(6, id, leng, error, params)
            else:
                return self.getPacket(5, id, leng, error, params)
        elif mode == 6:         # read checksum
            checksum = id + leng + error + sum(params) + ord(d)
            #print checksum            
            if checksum % 256 != 255:
                print "Checksum ERROR"
                #return None
            return params
        # fail
        return None

    def execute(self, index, ins, params, ret=True):
        """ Send an instruction to a device. """
        values = None
        self.mutex.acquire()        
        self.ser.flushInput()
        length = 2 + len(params)
        checksum = 255 - ((index + length + ins + sum(params))%256)
        self.ser.write(chr(0xFF)+chr(0xFF)+chr(index)+chr(length)+chr(ins))
        for val in params:
            self.ser.write(chr(val))
        self.ser.write(chr(checksum))
        if ret:
            values = self.getPacket(0)
        self.mutex.release()
        return values
    
    def read(self, index, start, length):
        """ Read values of _length_ registers, starting at addr _start_. """
        values = self.execute(index, AX_READ_DATA, [start, length])
        if values == None:
            print "Read Failed: Servo ID = " + str(index)
            return -1        
        else:
            return values

    def write(self, index, start, values):
        """ Write values to registers, starting at _start_, returns 
        error level. Should be called like setReg(1,1,(0x01,0x05)) """ 
        self.execute(index, AX_WRITE_DATA, [start] + values)
        return self.error     

    def syncWrite(self, start, values):
        """ Set the value of registers. Should be called as such:
        ax12.syncWrite(reg, ((id1, val1, val2), (id2, val1, val2))) """ 
        output = list()
        for i in values:
            output = output + i          
        checksum = 255 - ((254 + length + AX_SYNC_WRITE + start + len(values[0]) - 1 + valsum)%256)        
        self.execute(0xFE, AX_SYNC_WRITE, [start, len(values[0])-1 ] + values, False)

    def syncRead(self, servos, start, length):
        """ syncRead( [1,2,3,4], P_PRESENT_POSITION_L, 2) """
        return self.execute(0xFE, AX_SYNC_READ, [start, length] + servos )
    
    ##########################################################################
    # Common Helper Functions
    def getReturnLevel(self, index):
        """ Returns the return level (integer). """
        try:
            return int(self.read(index, P_RETURN_LEVEL, 1)[0])
        except:
            return -1
    def setReturnLevel(self, index, value):
        """ Set the value of the return level. """
        return self.write(index, P_RETURN_LEVEL, [value])        
        
    def enableTorque(self, index):
        return self.write(index, P_TORQUE_ENABLE, [1])
    def disableTorque(self, index):
        return self.write(index, P_TORQUE_ENABLE, [0])

    def setLed(self, index, value):
        """ Set LED status. """
        return self.write(index, P_LED, [value])

    def setPosition(self, index, value):
        """ Move to position in ticks. """
        return self.write(index, P_GOAL_POSITION_L, [value%256, value>>8])

    def setSpeed(self, index, value):
        """ Set Speed of movement. """
        return self.write(index, P_GOAL_SPEED_L, [value%256, value>>8])

    def getPosition(self, index):
        """ Returns position in ticks """
        values = self.read(index, P_PRESENT_POSITION_L, 2)
        try:
            return int(values[0]) + (int(values[1])<<8)
        except:
            return -1

    def getVoltage(self, index):
        """ Returns voltage (V). """
        try:
            return int(self.read(index, P_PRESENT_VOLTAGE, 1)[0])/10.0
        except:
            return -1    
    
    def getTemperature(self, index):
        """ Returns temperature (C). """
        try:
            return int(self.read(index, P_PRESENT_TEMPERATURE, 1)[0])
        except:
            return -1

    def isMoving(self, index):
        """ Returns True if servo is moving. """
        try:
            d = self.read(index, P_MOVING, 1)[0]
        except:
            return True
        return d != 0

    ###########################################################################
    # Extended ArbotiX Driver
    LOW = 0
    HIGH = 0xff
    INPUT = 0
    OUTPUT = 0xff

    # ArbotiX-specific register table
                        # We do Model, Version, ID, Baud, just like the AX-12
    DIG_BLOCK_1 = 5     # Read digital pins 0-7
    DIG_BLOCK_2 = 6     # Read digital pins 8-15
    DIG_BASE = 7        # Write digital
                        # Addresses 15 unused
                        # 16, 17 = RETURN, ALARM
    ANA_BASE = 18       # First analog port (Read only)
                        # Each additional port is BASE + index
    SERVO_BASE = 26     # Up to 10 servos, each uses 2 bytes (L, then H), pulse width (0, 1000-2000ms) (Write only)
                        # Address 46 is Moving, just like an AX-12
    LEFT_SIGN = 47      # Motor pwm (-255 to 255), 1 byte sign + 1 byte speed per side
    LEFT_PWM = 48
    RIGHT_SIGN = 49
    RIGHT_PWM = 50

    LEFT_SPEED_L = NUKE_X_SPEED_L = 51   # Motor Speed (ticks per frame, 2 bytes, signed) 
    RIGHT_SPEED_L = NUKE_R_SPEED_L = 53
    NUKE_Y_SPEED_L = 55

    LEFT_ENC_L = NUKE_X_ENC_L = 57   # Endpoints (ticks, 4 bytes, signed)
    RIGHT_ENC_L = NUKE_R_ENC_L = 61
    NUKE_Y_ENC_L = 65

    KP = 69             # PID control parameters
    KD = 70
    KI = 71
    KO = 72
    
    GP_SCAN_ENABLE = 79
    GP_BASE = 80

    def getAnalog(self, index):
        """ Read an analog port, returns 0-255 (-1 if error). """
        try:
            return int(self.read(253, self.ANA_BASE+int(index), 1)[0])
        except:
            return -1

    def getDigital(self, index):
        """ Read a digital port, returns 0 (low) or 0xFF (high) (-1 if error).\
            (index = 0 to 15) """
        try:
            if index > 7:
                x = self.read(253, self.DIG_BLOCK_2, 1)[0]
            else:
                x = self.read(253, self.DIG_BLOCK_1, 1)[0]                
        except:
            return -1
        if x & (2**(index%8)):
            return 255
        else:
            return 0

    def setDigital(self, index, val, direction=OUTPUT):
        """ Set value (and direction) of a digital IO (index = 0 to 7) """
        if val == LOW and direction == OUTPUT:
            self.write(253, self.DIG_BASE + 1 + int(index), [1])
        elif val == HIGH and direction == OUTPUT:
            self.write(253, self.DIG_BASE + 1 + int(index), [3])
        elif val == HIGH and direction == INPUT:
            self.write(253, self.DIG_BASE + 1 + int(index), [2])
        else:
            self.write(253, self.DIG_BASE + 1 + int(index), [0])
            
    def setServo(self, index, val):
        """ Set a servo value (in milliseconds) """
        if val != 0 and (val < 500 or val > 2500):
            print "ArbotiX Error: Servo value out of range:",val
        else:
            self.write(253, self.SERVO_BASE + 2*index, [val%256, val>>8])

    def setMotors(self, left, right):
        """ Set raw motor pwm speeds. """
        if left < -255 or left > 255 or right < -255 or right > 255:
            print "ArbotiX Error: Motor values out of range"
        else:
            self.write(253, self.LEFT_SIGN, [1*(left<0), abs(left), 1*(right<0), abs(right)])    
    
    def setSpeeds(self, left, right):
        """ Send a closed-loop speed. """
        self.write(253, self.LEFT_SPEED_L, [left%256, left>>8, right%256, right>>8] )

    def baseMoving(self):
        try:
            return int(self.read(253, P_MOVING, 1)[0])
        except:
            return -1

    # Functions to read 32-bit (signed) encoder values
    def getEnc(self, addr):
        values = self.read(253, addr, 4)
        values = "".join([chr(k) for k in values])
        try:
            return unpack('l',values)[0]
        except:
            return None        
    def getLenc(self):
        return self.getEnc(self.ENC_LEFT_L)        
    def getRenc(self):
        return self.getEnc(self.ENC_RIGHT_L)
    def getEncoders(self):
        values = self.read(253, self.ENC_LEFT_L, 8)
        left_values = "".joint([chr(k) for k in values[0:4] ])        
        right_values = "".joint([chr(k) for k in values[4:] ])
        try:
            left = unpack('l',left_values)[0]
            right = unpack('l',right_values)[0]
            return [left, right]
        except:
            return None        

    def enableGp(self, value):
        if value:
            return self.write(253, self.GP_SCAN_ENABLE, [1])
        else:
            return self.write(253, self.GP_SCAN_ENABLE, [0])

if __name__ == "__main__":
    # some simple testing
    print "Testing arbotix.py"
    d = ArbotiX(sys.argv[1])  # open a port
    d.setPosition(1,512)
    print d.getPosition(1)

