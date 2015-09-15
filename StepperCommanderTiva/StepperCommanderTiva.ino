/* to be used with http://energia.nu/ or http://arduino.cc/
This firmware periodically reads the analog value of the 

UART device communicating with Universal Unipolar Stepper Controller implemented on Tiva C launchpad, adjust UART ports for use with others

MSP45430G2955 suppored in Energia with modification:
http://forum.43oh.com/topic/3594-msp430g2955-launchpad-development/?p=47184

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

// https://github.com/madsci1016/Arduino-EasyTransfer
#include <EasyTransfer.h>

#define LED PF_2
#define LED_GREEN PF_3

#pragma PACK

//create object
// communication objects
EasyTransfer ETin, ETout; 


struct __attribute__ ((packed)) struct_motor_t{
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
  uint8_t gpio;
  uint8_t empty3;
};
// Big warning, struct must be packed for use cross-platform between 8bit, 16bit and 32bit devices

//Making a <union so reading as struct or as char array is possible
typedef union data_t{
 struct_motor_t u;
 char c[sizeof(struct_motor_t)];
 uint8_t i[sizeof(struct_motor_t)];
};

//Create data structure isntance
data_t motor_tx;
data_t motor_rx;


long time_stored;


void setup(){
  Serial.begin(115200);
  Serial.print("Size: ");
   Serial.println(sizeof(motor_rx));
  Serial1.setPins(UART1_PORTB); //trick to active USART1 without overlap USART4
  Serial1.begin(115200);
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(motor_rx.c), &Serial1);
  ETout.begin(details(motor_tx.c), &Serial1);
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  randomSeed(analogRead(0));
  motor_tx.u.command=0;
  time_stored=0;
  motor_rx.u.current_x=1;
  motor_rx.u.current_y=1;
}

char state=0;

void loop(){
  // this test code performs the following operations
  // sets the acceleration and speed
  // moves motors to a position
  // homes X and Y in negative direction
  // creates repeated squares movemenet
  switch (state) {
    case 0:
      motor_tx.u.motor_accel=5000;
      motor_tx.u.motor_speed=300;
      motor_tx.u.command=4;
      state=1;
      break;
    case 1:
      motor_tx.u.next_x=10000;
      motor_tx.u.next_y=10000;
      motor_tx.u.command=0;
      if((motor_rx.u.current_x==10000)&(motor_rx.u.current_y==10000)){
        state=2;
      }
      break;
    case 2:
      //home
      motor_tx.u.command=2;
      if((motor_rx.u.current_x==0)&(motor_rx.u.current_y==0)){
        state=3;
        motor_tx.u.command=0;
      }
      break;
    case 3:
      //move square
      motor_tx.u.next_x=10000;
      motor_tx.u.next_y=10000;
      motor_tx.u.command=0;
      motor_tx.u.gpio=1;
      if((motor_rx.u.current_x==motor_tx.u.next_x)&&(motor_rx.u.current_y==motor_tx.u.next_y)){
        state++;
      }
      break;
    case 4:
      //move square
      motor_tx.u.next_x=20000;
      motor_tx.u.next_y=10000;
      motor_tx.u.command=0;
      if((motor_rx.u.current_x==motor_tx.u.next_x)&&(motor_rx.u.current_y==motor_tx.u.next_y)){
        state++;
      }
      break;
    case 5:
      //move square
      motor_tx.u.next_x=20000;
      motor_tx.u.next_y=20000;
      motor_tx.u.command=0;
      if((motor_rx.u.current_x==motor_tx.u.next_x)&&(motor_rx.u.current_y==motor_tx.u.next_y)){
        state++;
      }
      break;
    case 6:
      //move square
      motor_tx.u.next_x=10000;
      motor_tx.u.next_y=20000;
      motor_tx.u.command=0;
      if((motor_rx.u.current_x==motor_tx.u.next_x)&&(motor_rx.u.current_y==motor_tx.u.next_y)){
        state=3;
      }
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
    break;
  }


/*
  if(motor_rx.u.current_x>45000){
    motor_tx.u.gpio=1;
  }
  else{
    motor_tx.u.gpio=1;
  }
  
  if(motor_rx.u.current_x<5000){
    motor_tx.u.gpio=1;
  }
  else{
    motor_tx.u.gpio=0;
  }

  // here sending some commands for testing each axis
  if(motor_rx.u.current_x==0){
    motor_tx.u.next_x=50000;
    time_stored=millis()+120000;
  }
  else if(motor_rx.u.current_x==50000 & time_stored<millis()){
    motor_tx.u.next_x=0;
  }
  
  if(motor_rx.u.current_y==0){
    motor_tx.u.next_y=50000;
    time_stored=millis()+120000;
  }
  else if(motor_rx.u.current_y==50000 & time_stored<millis()){
    motor_tx.u.next_y=0;
  }
  
  if(motor_rx.u.current_f==0& time_stored<millis()){
    motor_tx.u.next_f=10000;
  }
  else if(motor_rx.u.current_f==10000){
    motor_tx.u.next_f=0;
  }
*/
  //send the data
  digitalWrite(LED, HIGH);
  ETout.sendData();
  digitalWrite(LED, LOW);
  
  //check and see if a data packet has come in. 
  if(ETin.receiveData()){
    digitalWrite(LED_GREEN, HIGH);
    dump_motor_variables();
    //motor_tx.u.command=motor_rx.u.command;//command disables itself
  }
  
  delay(500);
  digitalWrite(LED_GREEN, LOW);
}

void dump_motor_variables(){
  Serial.println("");
  Serial.print("Current x: ");
  Serial.println(motor_rx.u.current_x);
  Serial.print("Next x: ");
  Serial.println(motor_tx.u.next_x);
  Serial.print("Status x: 0x");
  Serial.println(motor_rx.u.status_x,HEX);
  Serial.print("Current y: ");
  Serial.println(motor_rx.u.current_y);
  Serial.print("Next y: ");
  Serial.println(motor_tx.u.next_y);
  Serial.print("Status y: 0x");
  Serial.println(motor_rx.u.status_y,HEX);
  Serial.print("Current f: ");
  Serial.println(motor_rx.u.current_f);
  Serial.print("Next f: ");
  Serial.println(motor_rx.u.next_f);
  Serial.print("Status f: ");
  Serial.println(motor_rx.u.status_f);
  Serial.print("Speed: ");
  Serial.println(motor_rx.u.motor_speed);
  Serial.print("Accel: ");
  Serial.println(motor_rx.u.motor_accel);
  Serial.print("Command: ");
  Serial.println(motor_rx.u.command);
  Serial.print("Command tx: ");
  Serial.println(motor_tx.u.command);
  Serial.print("Flash: ");
  Serial.println(motor_rx.u.flash_status,HEX);
  Serial.print("Flash write count: ");
  Serial.println(motor_rx.u.flash_write_count,DEC);
  Serial.print("State: ");
  Serial.println(state,DEC);
}
