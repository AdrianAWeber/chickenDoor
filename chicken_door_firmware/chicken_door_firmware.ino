/*
 * CHICKEN DOOR AUTOMATISATION
 * 
 *  Pin 0 -> Interrupt Button_complete
 *  Pin 1 -> Interrupt Buttpn_steps
 *  Pin 2 -> Interrupt DS3231
 *  
 *  Pin 3 -> Switch opened
 *  Pin 4 -> Switch closed
 *  
 *  Pin 5 -> Motor Enable
 *  Pin 6 -> Motor connection A
 *  Pin 7 -> Motor connection B
 */

//TODO: Check if Pin 0 can be used for pin 2; otherwise fix it on PCB
//      PCB has no MOTOR Conector ...

#include <EEPROM.h>
#include <avr/sleep.h>
#include "LowPower.h"

#define STOP_BY_SENSOR 1

/**
 * Digital Pins
 */
 
#define MOVE_COMPLETE 1
#define MOVE_STEP 0
#define BTN_step_read 2
#define Photo_Pwr 4
#define Motor_En 5
#define Motor_B 6
#define Motor_A 7
#define End_down 8
#define End_up 9
#define DOWN_5V 14
#define UP_5V   16
#define MOTOR_OUT_5V 10

/**
 * Analog Pins
 */
#define PR 0 // analog Pin0  -> A0 | PhotoSensor Measurement


#define TIME_OPEN  17000
#define TIME_CLOSE 14200

#define LightThreshold_close 18
#define LightThreshold_open 25

byte position_door = 0;
byte dir           = 0;  // 0: down; 1: up
byte pressed       = 0;

byte move_complete_interr = 0;
byte move_step_interr     = 1;

unsigned int light         = 1000;
unsigned int light_pre     = 1000;

//Measure not permamntly: interval can be set here 
unsigned int timecnt       = 0 ; 
unsigned int TimeIntervall = 5;//225; // Multiple of 8 sec   

void setup() {
  // Read position of door  -> Address 0
  position_door = EEPROM.read(0);  // 0 -> closed ; 1-> open
  pinMode(Motor_En, OUTPUT);
  pinMode(Motor_A, OUTPUT);
  pinMode(Motor_B, OUTPUT);
  pinMode(Photo_Pwr, OUTPUT);
  pinMode(DOWN_5V, OUTPUT);
  pinMode(UP_5V, OUTPUT);
  pinMode(MOTOR_OUT_5V, OUTPUT);

  digitalWrite(Motor_En,LOW); // Stop
  digitalWrite(Motor_A,LOW);   // Save power
  digitalWrite(Motor_B,LOW);
  digitalWrite(Photo_Pwr,LOW);
  digitalWrite(DOWN_5V,LOW);
  digitalWrite(UP_5V,LOW);
  digitalWrite(MOTOR_OUT_5V,LOW);
     
  attachInterrupt(digitalPinToInterrupt(MOVE_COMPLETE), move_complete_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(MOVE_STEP), move_step_ISR, RISING);
  //Serial.begin(9600);
}

void loop() {

//LowPower.idle(SLEEP_8S, ADC_OFF, TIMER4_OFF, TIMER3_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART1_OFF, TWI_OFF, USB_OFF);
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  timecnt++;
  
  if (move_complete_interr == 1){ 
        if (position_door == 1){ // open
          close_door(TIME_CLOSE);
        } else {
          open_door(TIME_OPEN);
        }
     move_complete_interr = 0;
     move_step_interr = 0;
     if (timecnt >= TimeIntervall) {timecnt = 0;}
  } else if(move_step_interr == 1) {

     if (dir == 0){
        down();
        dir = 1;
     } else {
        up();
        dir = 0;
     }
     digitalWrite(Motor_En, HIGH);
     while (move_step_interr == 1) {
        delay(200);
        move_step_interr = digitalRead(MOVE_STEP);//digitalRead(BTN_step_read);
        
        if ((move_step_interr == 1) && (STOP_BY_SENSOR == 1)) {
           int endPos = 0;
           if (dir == 1) endPos = digitalRead(End_down); // moving down
           if (dir == 0) endPos = digitalRead(End_up);
           if (endPos == 1) move_step_interr = 0; // break loop due to End of door
        }
     }

     // Stop Motor again
     digitalWrite(Motor_En,LOW); // Stop
     digitalWrite(Motor_A,LOW);   // Save power
     digitalWrite(Motor_B,LOW);
     digitalWrite(MOTOR_OUT_5V,LOW);

     //Unpower the Reeds
     if (STOP_BY_SENSOR == 1) { 
       digitalWrite(DOWN_5V,LOW);
       digitalWrite(UP_5V,LOW);
     }
     
     move_complete_interr = 0;
     move_step_interr = 0;
     if (timecnt >= TimeIntervall) {timecnt = 0;}
  
  } else{
        if (timecnt >= TimeIntervall) {
            digitalWrite(Photo_Pwr,HIGH); delay(100);
            light = analogRead(PR); delay (100);
            digitalWrite(Photo_Pwr,LOW);
            //Serial.println(light);
            // evening
            if (light <= LightThreshold_close && light_pre > LightThreshold_close && position_door == 1){
               close_door(TIME_CLOSE);
            } else if (light > LightThreshold_open && light_pre <= LightThreshold_open && position_door == 0) {
              open_door(TIME_OPEN);
            }
            light_pre = light;
     
            move_complete_interr = 0;
            move_step_interr = 0;
            timecnt = 0;
         }
  }
}//loop


// open the door
void open_door(unsigned int open_time){
  //Motor Settings for upwards
  up();

  // enable Motor
  digitalWrite(Motor_En, HIGH);

  if (STOP_BY_SENSOR == 1) {
    bool endPos = 0;
    while (endPos == 0){
      endPos = digitalRead(End_up);
    }
  } else { // stop by time
    delay(open_time);
  }
  
  digitalWrite(Motor_En,LOW); // Stop
  digitalWrite(Motor_A,LOW);   // Save power
  digitalWrite(Motor_B,LOW);
  digitalWrite(MOTOR_OUT_5V,LOW);

  //Unpower the Reeds
  if (STOP_BY_SENSOR == 1) { 
    digitalWrite(DOWN_5V,LOW);
    digitalWrite(UP_5V,LOW);
  }

  // Save status to EEPROM
  EEPROM.write(0,1); // open
  position_door = 1;
}



//close the door
void close_door(unsigned int closeTime){
  // Motor Setting to cloase door
  down();

  // enable Motor
  digitalWrite(Motor_En, HIGH);

  if (STOP_BY_SENSOR == 1) {
    bool endPos = 0;
    while (endPos == 0){
      endPos = digitalRead(End_down);
    }
  } else { // stop by time
    delay(closeTime);
  }
  // closed!
  
  // Stop Motor again
  digitalWrite(Motor_En,LOW); // Stop
  digitalWrite(Motor_A,LOW);   // Save power
  digitalWrite(Motor_B,LOW);
  digitalWrite(MOTOR_OUT_5V,LOW);

  //Unpower the Reeds
  if (STOP_BY_SENSOR == 1) { 
    digitalWrite(DOWN_5V,LOW);
    digitalWrite(UP_5V,LOW);
  }  

  // Save status to EEPROM
  EEPROM.write(0,0); // closed
  position_door = 0;
}

void up(){
  digitalWrite(MOTOR_OUT_5V,HIGH);
   // Motor will turn right
  digitalWrite(Motor_A,HIGH);   
  digitalWrite(Motor_B,LOW);

  //power the upwards Reed 
  if (STOP_BY_SENSOR == 1) digitalWrite(UP_5V,HIGH);  
}

void down(){
  digitalWrite(MOTOR_OUT_5V,HIGH);
   // Motor will turn left
  digitalWrite(Motor_A,LOW);   
  digitalWrite(Motor_B,HIGH);

  //power the downwards Reed 
  if (STOP_BY_SENSOR == 1) digitalWrite(DOWN_5V,HIGH);
}

void move_complete_ISR(){
  move_complete_interr = 1;
}

void move_step_ISR(){
  move_step_interr = 1;
}
