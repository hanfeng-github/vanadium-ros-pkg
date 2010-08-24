/* 
  ServoStiK Firmware for ROS driver
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

/* Build Configuration */ 
#define ROS_SERVO_STIK
#define USE_GP_LIDAR

#include <dynamixel.h>
#include <BioloidController2.h>
#include <HServo.h>

/* Hardware Constructs */
BioloidController2 bioloid = BioloidController2(1000000);
HServo servos[2];

#include "ros.h"

#ifdef USE_GP_LIDAR 
  #define LIDAR_SERVO     3
  #include "gp_lidar.h"
#endif

/* Register Storage */
unsigned char baud = 7;         // ?
unsigned char ret_level = 1;    // ?
unsigned char alarm_led = 0;    // ?
int servo_vals[10];             // in millis

/* 
 * Setup
 */
void setup(){
  Serial.begin(38400);   
#ifdef USE_GP_LIDAR
  init_gp_lidar();
#endif
  pinMode(0,OUTPUT);     // status LED
}

/*
 * Handle Write Requests to ArbotiX Registers 
 */
unsigned char handleWrite(){
  int addr  = params[0];  // address to write
  int bytes = length-3;   // # of bytes left to write
  int k = 1;              // index in parameters of value to write
  while(bytes > 0){
    if(addr < REG_BAUD_RATE){
      return INSTRUCTION_ERROR;
    }else if(addr == REG_BAUD_RATE){
      // TODO: update baud
    }else if(addr < REG_RETURN_LEVEL){
      // write digital 
      int pin = addr - REG_DIG_BASE;
        if(params[k] & 0x02)    // high
          digitalWrite(pin, HIGH);
        else
          digitalWrite(pin, LOW);
        if(params[k] & 0x01)    // output
          pinMode(pin, OUTPUT);
        else
          pinMode(pin, INPUT);
    }else if(addr == REG_RETURN_LEVEL){
      ret_level = params[k];
    }else if(addr == REG_ALARM_LED){
      // TODO: 
    }else if(addr < REG_SERVO_BASE){
      return INSTRUCTION_ERROR; // error - analog are read only
    }else if(addr < REG_MOVING){
      // write servo
      int s = addr - REG_SERVO_BASE;
      if( s%2 == 0 ){ // low byte
        s = s/2;
        servo_vals[s] = params[k];
      }else{          // high byte
        s = s/2;
        servo_vals[s] += (params[k]<<8);
        if(servo_vals[s] > 500 && servo_vals[s] < 2500){
          servos[s].writeMicroseconds(servo_vals[s]);
          if(!servos[s].attached())            
            servos[s].attach(s);
        }else if(servo_vals[s] == 0){
          servos[s].detach();
        }
      }
    }else if(addr == REG_MOVING){
      return INSTRUCTION_ERROR;
#ifdef USE_GP_LIDAR
    }else if(addr == REG_GP_SCAN){
      gp_lidar_enable = params[k];
#endif
    }else{
      return INSTRUCTION_ERROR;
    }
    addr++;k++;bytes--;
  }
  return 0;
}

int handleRead(){
  int checksum = 0;
  int addr = params[0];
  int bytes = params[1];
  unsigned char v;
  while(bytes > 0){
    if(addr == REG_MODEL_NUMBER_L){ 
      v = 44;
    }else if(addr == REG_MODEL_NUMBER_H){
      v = 1;  // 300 
    }else if(addr == REG_VERSION){
      v = 0;
    }else if(addr == REG_ID){
      v = 253;
    }else if(addr == REG_BAUD_RATE){
      v = 34; // 56700
    }else if(addr < REG_RETURN_LEVEL){
      // send digital read
      v = PINB;
    }else if(addr == REG_RETURN_LEVEL){
      v = ret_level;
    }else if(addr == REG_ALARM_LED){
      // TODO
    }else if(addr < REG_SERVO_BASE){
      // send analog reading
      v = analogRead(addr-REG_ANA_BASE)>>2;
    }else if(addr < REG_MOVING){
      // send servo position
      v = 0;
#ifdef USE_GP_LIDAR
    }else if(addr < REG_GP_BASE + 60){
      v = gp_readings[addr-REG_GP_BASE];
#endif
    }else{
      v = 0;        
    }
    checksum += v;
    Serial.print(v, BYTE);
    addr++;bytes--;
  }
  return checksum;
}

/*
 * Send status packet
 */
void statusPacket(int id, int err){
  Serial.print(0xff,BYTE);
  Serial.print(0xff,BYTE);
  Serial.print(id,BYTE);
  Serial.print(2,BYTE);
  Serial.print(err,BYTE);
  Serial.print(255-((id+2+err)%256),BYTE);
}

/* 
 * decode packets: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix 
 */
void loop(){
  int i;
    
  // process messages
  if(Serial.available() > 0){
    // We need to 0xFF at start of packet
    if(mode == 0){         // start of new packet
      if(Serial.read() == 0xff){
        mode = 2;
        digitalWrite(0,HIGH-digitalRead(0));
      }
   //}else if(mode == 1){   // another start byte
   //    if(Serial.read() == 0xff)
   //        mode = 2;
   //    else
   //        mode = 0;
   }else if(mode == 2){   // next byte is index of servo
     id = Serial.read();    
     if(id != 0xff)
       mode = 3;
   }else if(mode == 3){   // next byte is length
     length = Serial.read();
     checksum = id + length;
     mode = 4;
   }else if(mode == 4){   // next byte is instruction
     ins = Serial.read();
     checksum += ins;
     index = 0;
     mode = 5;
   }else if(mode == 5){   // read data in 
     params[index] = Serial.read();
     checksum += (int) params[index];
     index++;
     if(index + 1 == length){  // we've read params & checksum
       mode = 0;
       if((checksum%256) != 255){ 
         // return an error packet: FF FF id Len Err=bad checksum, params=None check
         statusPacket(id,CHECKSUM_ERROR);
       }else if(id == 253){  // ID = 253, ArbotiX instruction
         switch(ins){     
           case AX_WRITE_DATA:
             // send return packet
             statusPacket(id,handleWrite());
             break;
             
           case AX_READ_DATA:
             checksum = id + params[1] + 2;                            
             Serial.print(0xff,BYTE);
             Serial.print(0xff,BYTE);
             Serial.print(id,BYTE);
             Serial.print(2+params[1],BYTE);
             Serial.print(0,BYTE);
             // send actual data
             checksum += handleRead();
             Serial.print(255-((checksum)%256),BYTE);
             break;
         }
       }else if(id == 0xFE){
         // sync read or write
         if(ins == ARB_SYNC_READ){
           int start = params[0];    // address to read in control table
           int bytes = params[1];    // # of bytes to read from each servo
           int k = 2;
           checksum = id + (bytes*(length-4)) + 2;                            
           Serial.print(0xff,BYTE);
           Serial.print(0xff,BYTE);
           Serial.print(id,BYTE);
           Serial.print(2+(bytes*(length-4)),BYTE);
           Serial.print(0,BYTE);     // error code
           // send actual data
           for(k=2; k<length-2; k++){
             dynamixelGetRegister(params[k], start, bytes);
             for(i=0;i<bytes;i++){
               checksum += dynamixel_rx_buffer[5+i];
               Serial.print(dynamixel_rx_buffer[5+i],BYTE);
             }
           }
           Serial.print(255-((checksum)%256),BYTE);
         }else{    // TODO: sync write pass thru
         
           
           // no return
         }       
       }else{ // ID != 253, pass thru 
         switch(ins){
           // TODO: streamline this
           case AX_READ_DATA:
             dynamixelGetRegister(id, params[0], params[1]);
             // return a packet: FF FF id Len Err params check
             if(dynamixel_rx_buffer[3] > 0){
               for(i=0;i<dynamixel_rx_buffer[3]+4;i++)
                 Serial.print(dynamixel_rx_buffer[i],BYTE);
             }
             dynamixel_rx_buffer[3] = 0;
             break;
             
           case AX_WRITE_DATA:
             if(length == 4){
               dynamixelSetRegister(id, params[0], params[1]);
             }else{
               int x = params[1] + (params[2]<<8);
               dynamixelSetRegister2(id, params[0], x);
             }
             statusPacket(id,0);
             break;
             
         }
       }
     }
   } // end mode == 5
 } // end while(available)
#ifdef USE_GP_LIDAR
 step_gp_lidar();
#endif
}
