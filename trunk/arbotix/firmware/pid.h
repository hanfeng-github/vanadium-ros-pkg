/******************************************************************************
 * Cerebellum Motor Controller (Version A.0)
 * Copyright 2009-2010 Vanadium Labs LLC
 *
 * This PID loop runs at ~30hz
 *****************************************************************************/ 

#include <WProgram.h>
#include <math.h>

/* PID Parameters */
int Kp;
int Kd;
int Ki;
int Ko;                      // output scale
int maxAccel;                // maximum acceleration per frame (ticks)

/* PID modes */
unsigned int PIDmode;
#define PID_NONE        0
#define PID_SPEED       1
#define PID_DISTANCE    2
#define PID_BOTH        3

#define FRAME_RATE      33   // frame rate in millis (30Hz)
#define FRAMES          30
unsigned long f_time;        // last frame

unsigned char moving = 0;    // base in motion
unsigned char paused = 0;    // base was in motion, can resume
#define MAXOUTPUT       255  // motor PWM

/* Setpoint Info For a Motor */
typedef struct{
  // These 2 are set up by user
  long Endpoint;             // desired distance to travel (in counts)
  int  VelocitySetpoint;     // desired average speed to travel (in counts/frame)
  // 
  long Encoder;              // actual reading
  long PrevEnc;              // last reading
  int  Velocity;             // current desired average speed (counts/frame), taking ramping into accound
  int PrevErr;
  int Ierror;   
  int output;                // last motor setting
  long rampdown;             // how long it will take to slow down
} SetPointInfo;

SetPointInfo left, right;

/* Initialize PID parameters to something known */
void setupPID(){
  // Default values for the PR-MINI
  Kp = 25;
  Kd = 30;
  Ki = 0;
  Ko = 100;
  maxAccel = 50;
  f_time = 0;
}

/* Linearly Ramp Velocity TO VelocitySetpoint */
void Ramp(SetPointInfo * x){
  if(x->Velocity < x->VelocitySetpoint){
    x->Velocity += maxAccel;
    if( x->Velocity > x->VelocitySetpoint)
      x->Velocity = x->VelocitySetpoint;
  }else{
    x->Velocity -= maxAccel;
    if( x->Velocity < x->VelocitySetpoint)
      x->Velocity = x->VelocitySetpoint;
  }  
}

/* PID control of motor speed */
void DoPid(SetPointInfo * p){
  int Perror, output;
  
  Perror = p->Velocity - (p->Encoder-p->PrevEnc);
          
  // Derivative error is the delta Perror
  output = (Kp*Perror + Kd*(Perror - p->PrevErr) + Ki*p->Ierror)/Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;
  
  output += p->output;   
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAXOUTPUT)
    output = MAXOUTPUT;
  else if (output <= -MAXOUTPUT)
    output = -MAXOUTPUT;
  else
    p->Ierror += Perror;
  
  p->output = output;
}

/* This is called by the main loop, does a X HZ PID loop. */
void updatePID(){
  if((moving > 0) && (PIDmode > 0)){  // otherwise, just return
    unsigned long j = millis();
    if(j > f_time){
      // update encoders
      left.Encoder = Encoders.left;
      right.Encoder = Encoders.right;
      // check endpoints (do we need to start ramping down)
      if(PIDmode & PID_DISTANCE){
        if( (abs(left.Endpoint) - abs(left.Encoder)) < left.rampdown ){
          left.VelocitySetpoint = 0;
          if(abs(left.Velocity) < 50 && abs(right.Velocity) < 50)
            PIDmode = 0;  
        }
        if( (abs(right.Endpoint) - abs(right.Encoder)) < right.rampdown ){
          right.VelocitySetpoint = 0; 
          if(abs(left.Velocity) < 50 && abs(right.Velocity) < 50)
            PIDmode = 0;  
        }
      }
      // update velocity setpoints (ramping to velocity)
      Ramp(&left);
      Ramp(&right);      
      // do PID update on PWM
      DoPid(&left);
      DoPid(&right);
      // set updated motor outputs
      if(PIDmode > 0){
        drive.set(left.output, right.output);
      }else{
        drive.set(0,0);
        moving = 0;
      }
      // update timing
      f_time = j + FRAME_RATE;
    }
  }
}

void clearAll(){
  PIDmode = 0;
  left.Encoder = 0;
  right.Encoder = 0;
  left.PrevEnc = 0;
  right.PrevEnc = 0;
  left.output = 0;
  right.output = 0;
  Encoders.Reset();  
}

int doResetBase(){ 
  moving = 0; 
  paused = 0;
  PIDmode = 0;
  left.Encoder = 0;
  right.Encoder = 0;
  left.PrevEnc = 0;
  right.PrevEnc = 0;
  left.output = 0;
  right.output = 0;
  left.Velocity = 0;
  right.Velocity = 0;
  Encoders.Reset();  
  return 0;
}   
  
int doMoveBase(){
  doResetBase();
    
  left.VelocitySetpoint = left_speed;
  right.VelocitySetpoint = right_speed;
  left.Endpoint = left_endpoint;
  right.Endpoint = right_endpoint;
  
  left.rampdown = abs(left.VelocitySetpoint/maxAccel * left.VelocitySetpoint/2);
  right.rampdown = abs(right.VelocitySetpoint/maxAccel * right.VelocitySetpoint/2);

  if(left_endpoint == 0 && right_endpoint == 0)
    PIDmode = PID_SPEED;
  else
    PIDmode = PID_BOTH;
  moving = 1;
  return 0;
}

int doPauseBase(){
  if(moving > 0){
    moving = 0;
    drive.set(0,0);
    paused = 1;
    return 0;
  }
  // instruction error, we aren't moving 
  return INSTRUCTION_ERROR;
}

int doResumeBase(){
  if(paused > 0){
    moving = 1;
    return 0;
  }
  // instruction error, we aren't paused 
  return INSTRUCTION_ERROR;  
}


