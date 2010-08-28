/* 
  Poor Man's LIDAR (PML) for ROS driver ArbotiX Firmware
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

#ifdef USE_PML

#include <WProgram.h>

/* Takes 30 readings, 6 degrees apart for 180 degree coverage */
#define MAX_READINGS    30
#define MAX_STEPS       60
unsigned char pml_readings[2*MAX_READINGS];
#define PML_ANGLE_STEP   21
#define PML_START        206

#define PML_TIME_FRAME   50   // how long between movements
unsigned long pml_time;       // last time we moved
int pml_enable;
int pml_servo;
int pml_step;

/* setup default parameters */
void init_pml(){
  pml_step = 15;
  pml_enable = 0;
  pml_servo = -1;
  pml_time = millis() + PML_TIME_FRAME;
}

/* select servo, center it */
void set_pml_servo(int id){
  pml_servo = id;
  SetPosition(pml_servo, 512);
}

/* enable our PML */
void set_pml_enable(int k){
  if((k > 0) && (pml_servo > 0)){
    pml_enable = 1;
  }else{
    pml_enable = 0;
  }
}

/* take our next reading */
void step_pml(){
  unsigned long j = millis();
  if((j > pml_time) && (pml_servo > 0)){
    if((pml_enable > 0) | ((pml_step != 15) && (pml_step != 45)) ){
      // reading from IR sensor  
      int v = analogRead(0);
      // save reading
      if(pml_step >= MAX_READINGS){
          pml_readings[58 - ((pml_step*2)-60)] = v % 256;
          pml_readings[59 - ((pml_step*2)-60)] = v >> 8;
      }else{
          pml_readings[pml_step*2] = v % 256;
          pml_readings[(pml_step*2) + 1] = v>>8;
      }
      // advance    
      pml_step = (pml_step+1)%(MAX_STEPS);
      if(pml_step >= MAX_READINGS){
          SetPosition(pml_servo, PML_START + (PML_ANGLE_STEP * (MAX_STEPS -pml_step - 1)));
      }else{
          SetPosition(pml_servo, PML_START + (PML_ANGLE_STEP * pml_step));
      }
    }else{
      int v = analogRead(0);
      pml_readings[30] = v%256;
      pml_readings[31] = v>>8; 
    }
    pml_time = j+PML_TIME_FRAME;
  }
}

#endif
