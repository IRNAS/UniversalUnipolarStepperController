/* to be used with http://energia.nu/ or http://arduino.cc/
This firmware periodically reads the analog value of the 

Universal Unipolar Sttepper Controller for use with 28BYJ48 stepper motors

MSP45430G2955 suppored in Energia with modification:
http://forum.43oh.com/topic/3594-msp430g2955-launchpad-development/?p=47184

This support further modified in \energia-0101E0014\hardware\msp430\variants\spir_g2955\pins_energia.h:

line 106 to: static const uint8_t P2_6 = 29;
line 377 to: 	P2,  	/* 29 */
line 421 to: 	BV(6),  	/* 29 */

Copyright Institute IRNAS Rače 2015 - info@irnas.eu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

// http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>
// https://github.com/madsci1016/Arduino-EasyTransfer
#include <EasyTransfer.h>

// already in Energia
#include <msp430.h>
#include "MspFlash.h"
#define flash SEGMENT_D

//serial buffer size has been changed in
//energia-0101E0016\hardware\msp430\cores\msp430

/* Functions implemented:
- move motors to next coordinate specified
- report current coordinate of the motors
- stop when moving in negative X and Y directions and negative endswitch is pressed
- limit movement to positive range only
- movement in negative only with homing command, both X and Y simoultaneously
- 30s after all movement has stopped current location will be stored in flash for reboot persistentance
*/

/* Functions implemented:
- proper implementation of all endswitches
- homing functions per axis
- encoder support
- general debugging
*/


#define DEBUG

// communication objects
EasyTransfer ETin, ETout; 

#define PULSEWIDTH 20

//Motors
#define M_X1 P3_6
#define M_X2 P2_3
#define M_X3 P3_7
#define M_X4 P2_4

#define M_Y1 P1_0
#define M_Y2 P1_2
#define M_Y3 P1_1
#define M_Y4 P1_3

#define M_F1 P1_4
#define M_F2 P1_6
#define M_F3 P1_5
#define M_F4 P1_7

#define X_MIN P2_6
#define Y_MIN P2_5
#define F_MIN P4_1

#define LED_RED P4_5
#define LED_GREEN P4_6

// Creating object for Accel Stepper library
AccelStepper stepperX;
AccelStepper stepperY;
AccelStepper stepperF;

typedef __attribute__ ((packed)) struct struct_motor_t{
  int32_t current_x;
  int32_t next_x;
  int32_t current_y;
  int32_t next_y;
  int32_t current_f;
  int32_t next_f;
  uint16_t motor_speed;
  uint16_t motor_accel;
  uint8_t status_x;
  uint8_t status_y;
  uint8_t status_f;
  uint8_t command;
  uint8_t flash_status;
  uint8_t flash_write_count;
  uint8_t empty2;
  uint8_t empty3;
};

//Making a union so reading as struct or as char array is possible
union data_t{
 struct_motor_t u;
 unsigned char c[sizeof(struct_motor_t)];
 uint8_t i[sizeof(struct_motor_t)];
};


//Create data structure isntance
data_t motor;
data_t motor_rx;

//timing
long timeout_write_flash=0;
long timeout=0;

boolean write_enable = LOW;



void setup(){
  Serial.begin(115200);
  Serial.print("Init, size ");
  Serial.println(sizeof(struct_motor_t),DEC);
  ETin.begin(details(motor_rx.c), &Serial);
  ETout.begin(details(motor.c), &Serial);
  
  // Stepper motors in half step mode, for 24BYJ-48 4000steps per revolution
  stepperX.begin(8,M_X1,M_X2,M_X3,M_X4);
  stepperY.begin(8,M_Y1,M_Y2,M_Y3,M_Y4);
  stepperF.begin(8,M_F1,M_F2,M_F3,M_F4);
  
    // Load current position and the rest from flash
  flash_read();

  if(motor.u.flash_status!=0xab){
    // configure variables
    motor.u.current_x=0;
    motor.u.current_y=0;
    motor.u.current_f=0;
    motor.u.next_x=0;
    motor.u.next_y=0;
    motor.u.next_f=0;
    motor.u.status_x=0;
    motor.u.status_y=0;
    motor.u.status_f=0;
    motor.u.motor_speed=1000;
    motor.u.motor_accel=1000;
    motor.u.command=0;
    motor.u.flash_status=0xab;
    motor.u.flash_write_count=0;
    
    flash_erase();
    flash_write();
  }
  
  flash_read();
  
  // Initialize motors
  stepperX.setCurrentPosition(motor.u.current_x);
  stepperX.setMaxSpeed(motor.u.motor_speed);
  stepperX.setAcceleration(motor.u.motor_accel);
  stepperY.setCurrentPosition(motor.u.current_y);
  stepperY.setMaxSpeed(motor.u.motor_speed);
  stepperY.setAcceleration(motor.u.motor_accel);
  stepperF.setCurrentPosition(motor.u.current_f);
  stepperF.setMaxSpeed(motor.u.motor_speed);
  stepperF.setAcceleration(motor.u.motor_accel);

  // configure endswitches
  pinMode(X_MIN, INPUT_PULLUP);
  pinMode(Y_MIN, INPUT_PULLUP);
  pinMode(F_MIN, INPUT_PULLUP);
  
  // configure LEDs
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, LOW);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, LOW);

}

void home_motors(){
      stepperX.move(-100000);
      stepperY.move(-100000);
}

void loop(){

  //receive data and reply if received
  if(ETin.receiveData()){
    //process commands
    motor.u.command=motor_rx.u.command;
    process_command(motor_rx.u.command);
    
    // if command is 0 for normal operation
    if(motor_rx.u.command==0){      
      if(motor_rx.u.next_x<0){
        motor_rx.u.next_x=0;
      }
      else{
        motor.u.next_x=motor_rx.u.next_x;
      }
      
      if(motor_rx.u.next_y<0){
        motor_rx.u.next_y=0;
      }
      else{
        motor.u.next_y=motor_rx.u.next_y;
      }
      
      if(motor_rx.u.next_f<0){
        motor_rx.u.next_f=0;
      }
      else{
        motor.u.next_f=motor_rx.u.next_f;
      }
      stepperX.moveTo(motor_rx.u.next_x);
      stepperY.moveTo(motor_rx.u.next_y);
      stepperF.moveTo(motor_rx.u.next_f);
   }
    
    //reply
    digitalWrite(LED_RED, HIGH);
    ETout.sendData();
  }
  digitalWrite(LED_RED, LOW);
  
  motor.u.next_x=motor_rx.u.next_x;
 
  //run motors
  motor.u.status_x=runm(&stepperX,&motor.u.current_x,X_MIN);
  motor.u.status_y=runm(&stepperY,&motor.u.current_y,Y_MIN);
  motor.u.status_f=runm(&stepperF,&motor.u.current_f,F_MIN);
  
  // this should not be needed
  motor.u.current_x=stepperX.currentPosition();
  motor.u.current_y=stepperY.currentPosition();
  motor.u.current_f=stepperF.currentPosition();
  
  if((timeout_write_flash<millis())&(write_enable==HIGH)){
    flash_erase();
    flash_write();
    write_enable=LOW;
  }
}

void process_command(uint8_t command){
    switch (command) {
    case 1: // stop all movement
      stepperX.moveTo(motor.u.current_x);
      stepperY.moveTo(motor.u.current_y);
      stepperF.moveTo(motor.u.current_f);
      break;
    case 2: // home
      stepperX.move(-100000);
      stepperY.move(-100000);
      break;
    default: 
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

uint8_t runm(AccelStepper *stepper, int32_t *location, int min_pin){
  // this function runs the stepper motor
  // returns status of limits reached and motion stopped
  // 0x00 - idle
  // 0x01 - moving
  // 0x1* - maximum reached
  // 0x2* - maximum reached
  // 0xff - error
  
  uint8_t return_data = 0x00;
  
  //stop movement if endswitch pressed
  if(digitalRead(min_pin)==LOW){
    //stop only if moving in the negative direction
    if(stepper->targetPosition()<stepper->currentPosition()){
      stepper->stop();
      stepper->setCurrentPosition(0);
    }
    digitalWrite(LED_RED, HIGH);
  }
  else{
    digitalWrite(LED_RED, LOW);
  }
  
  // motor movement
  if(stepper->currentPosition()!=stepper->targetPosition()){
    stepper->enableOutputs();
    stepper->run();
    *location=stepper->currentPosition();
    return_data=(return_data|0x01);
    digitalWrite(LED_GREEN, HIGH);
    //reset timeout write
    timeout_write_flash=millis()+30000;
    write_enable=HIGH;
  }
  else{
    stepper->stop();
    stepper->disableOutputs();
    return_data=(return_data|0x00);
    digitalWrite(LED_GREEN, LOW);	
  }
  return return_data;
}

//flash for storing position
void flash_read(){
  Flash.read(flash, motor.c,sizeof(struct_motor_t));
}

void flash_erase(){
  Flash.erase(flash); 
}

void flash_write(){
  motor.u.flash_write_count++;
  Flash.write(flash, motor.c,sizeof(struct_motor_t)); 
}
