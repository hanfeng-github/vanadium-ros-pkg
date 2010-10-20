/* 
  Common Definitions for ROS driver ArbotiX Firmware
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
*/ 

/* ArbotiX (id:253) Instruction Definitions */
#define ARB_SIZE_POSE       7    // pose size: a single param for size of pose
#define ARB_LOAD_POSE       8    // load pose: index, then pose positions (# of params = 2*pose_size)
#define ARB_LOAD_SEQ        9    // seq size: a single param for the size of the seq
#define ARB_PLAY_SEQ        10   // load seq: index/times (# of params = 3*seq_size)
#define ARB_LOOP_SEQ        11   // play seq: no params
#define ARB_TEST            25   // hardware test: no params
#define ARB_SYNC_READ       0x84

/* ArbotiX (id:253) Register Table Definitions */
#define REG_MODEL_NUMBER_L  0
#define REG_MODEL_NUMBER_H  1
#define REG_VERSION         2
#define REG_ID              3
#define REG_BAUD_RATE       4
#define REG_DIGITAL_L       5   // Read values of lower 8 digital pins
#define REG_DIGITAL_H       6   // Read values of upper 8 digital pins
#define REG_DIG_BASE        7   // base + index, bit 1 = value (0,1), bit 0 = direction (0,1)
#define REG_RESCAN          15
#define REG_RETURN_LEVEL    16
#define REG_ALARM_LED       17
#define REG_ANA_BASE        18  // First Analog Port
#define REG_SERVO_BASE      26  // Up to 10 servos, each uses 2 bytes (L, then H), pulse width (0, 1000-2000ms)
#define REG_MOVING          46
#define REG_LM_SIGN         47  // Raw motor pwm (-255 to 255), 1 byte sign + 1 byte speed per side
#define REG_LM_PWM          48
#define REG_RM_SIGN         49
#define REG_RM_PWM          50

#define REG_LM_SPEED_L      51  // Motor Speed (ticks/sec, 2 bytes, signed)
#define REG_LM_SPEED_H      52
#define REG_RM_SPEED_L      53
#define REG_RM_SPEED_H      54

#define REG_X_SPEED_L       51  // OR Nuke Speed (mm/s, 2 bytes, signed) 
#define REG_X_SPEED_H       52
#define REG_R_SPEED_L       53
#define REG_R_SPEED_H       54
#define REG_Y_SPEED_L       55
#define REG_Y_SPEED_H       56

#define REG_ENC_LEFT_L      57  // Current Encoder Values (ticks, 4 bytes, signed)    
#define REG_ENC_RIGHT_L     61

#define REG_ENC_X_L         57  // OR Nuke Encoder Values (mm, 4 bytes, signed)
#define REG_ENC_R_L         61
#define REG_ENC_Y_L         65

#define REG_KP              69  // PID parameters ... not currently implemented
#define REG_KD              70
#define REG_KI              71
#define REG_KO              72

#define REG_PML_SERVO       80  // ID for PML servo
#define REG_PML_SENSOR      81  // ID for PML sensor
#define REG_STEP_START_L    82  // minimum ticks, low byte
#define REG_STEP_START_H    83
#define REG_STEP_VALUE      84  // ticks per step
#define REG_STEP_COUNT      85  // number of steps
#define REG_PML_ENABLE      86  // scan or no?

#define REG_PML_DIR         88  // direction of scan last taken
#define REG_PML_TIME_L      89  // low byte of time offset from first read on this scan
#define REG_PML_TIME_H      90  // high byte of time offset from first read on this scan
#define REG_PML_BASE        91  // start of data bytes for this scan

/* Packet Decoding */
int mode = 0;                   // where we are in the frame

unsigned char id = 0;           // id of this frame
unsigned char length = 0;       // length of this frame
unsigned char ins = 0;          // instruction of this frame

unsigned char params[50];       // parameters
unsigned char index = 0;        // index in param buffer

#ifdef USE_PML
int scan_dir;
int step_start;
int step_value;
#endif

int checksum;                   // checksum

#define INSTRUCTION_ERROR   0x40
#define CHECKSUM_ERROR      0x10

