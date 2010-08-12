/* 
  Poor Man's LIDAR for ROS driver ArbotiX Firmware
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

#include <WProgram.h>

/* Takes 30 readings, 6 degrees apart for 180 degree coverage */
#define MAX_READINGS    30
#define MAX_STEPS       60
unsigned char gp_readings[2*MAX_READINGS];
int gp_lidar_step;
#define GP_ANGLE_STEP   21
#define GP_START        206

#define GP_TIME_FRAME   50    // how long between movements
unsigned long gp_lidar_time;  // last time we moved
int gp_lidar_enable;

void init_gp_lidar(){
    gp_lidar_step = 15;
    gp_lidar_enable = 0;
    gp_lidar_time = millis() + GP_TIME_FRAME;
    SetPosition(LIDAR_SERVO, 512);
}

/* take our next reading */
void step_gp_lidar(){
  unsigned long j = millis();
  if(j > gp_lidar_time){
    if((gp_lidar_enable > 0) | ((gp_lidar_step != 15) && (gp_lidar_step != 45)) ){
      // reading from IR sensor  
      int v = analogRead(0);
      // save reading
      if(gp_lidar_step >= MAX_READINGS){
          gp_readings[58 - ((gp_lidar_step*2)-60)] = v % 256;
          gp_readings[59 - ((gp_lidar_step*2)-60)] = v >> 8;
          //gp_readings[MAX_STEPS - ((gp_lidar_step*2) + 2)] = v % 256;
          //gp_readings[MAX_STEPS - ((gp_lidar_step*2) + 1)] = v>>8;
      }else{
          gp_readings[gp_lidar_step*2] = v % 256;
          gp_readings[(gp_lidar_step*2) + 1] = v>>8;
      }
      // advance    
      gp_lidar_step = (gp_lidar_step+1)%MAX_STEPS;
      if(gp_lidar_step >= MAX_READINGS){
          SetPosition(LIDAR_SERVO, GP_START + (GP_ANGLE_STEP * (MAX_STEPS - gp_lidar_step - 1)));
      }else{
          SetPosition(LIDAR_SERVO, GP_START + (GP_ANGLE_STEP * gp_lidar_step));
      }
    }else{
      int v = analogRead(0);
      gp_readings[30] = v %256;
      gp_readings[31] = v>>8; 
    }
    gp_lidar_time = j+GP_TIME_FRAME;
  }
}

