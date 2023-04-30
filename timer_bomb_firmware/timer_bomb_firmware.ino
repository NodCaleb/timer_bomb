#include "GyverTM1637.h"
#define output_pin 8
#define rt_clk_pin 3
#define rt_dt_pin 4
#define rt_sw_pin 5
#define disp_clk_pin 6
#define disp_dio_pin 7
#define interrupt_pin 2

int rotation;
int value;
bool skip = false;
bool active = false;

int seconds = 0;
int new_seconds = 0;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 300;    // the debounce time; increase if the output flickers

GyverTM1637 disp(disp_clk_pin, disp_dio_pin);

void setup() {
  Serial.begin (9600);
  pinMode(output_pin, OUTPUT);
  pinMode (rt_clk_pin,INPUT);
  pinMode (rt_dt_pin,INPUT);
  pinMode (rt_sw_pin,INPUT);
  pinMode (interrupt_pin,INPUT);
  rotation = digitalRead(rt_clk_pin);

  disp.clear();
  disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
  displayTime(seconds);

  attachInterrupt(digitalPinToInterrupt(interrupt_pin), process_interrupt_debounced, HIGH);
}

void loop() {

  if (active){
    delay(1000);
    if(active) { //Check for the case interrupt has been triggered
      new_seconds--;
      if(new_seconds == 0){
        deactivate();
      }
    } 
  }  
  else {    
    value = digitalRead(rt_clk_pin);
     if (value != rotation){ // we use the DT pin to find out which way we turning.

      if(skip){ //Skipping every other signal, cause that fucking selector sends 2 in 1 click
        skip = false;
      }
      else{
        skip = true;
        if (digitalRead(rt_dt_pin) != value) {  // Clockwise
          new_seconds += secondsStep(seconds);
        } else { //Counterclockwise
          new_seconds -= secondsStep(seconds-1);
          if(new_seconds < 0) new_seconds = 0;
        }
      }          
   } 
   rotation = value;

   if(!digitalRead(rt_sw_pin)){
     new_seconds = 0;
   }    
  }

  if (seconds != new_seconds){
    seconds = new_seconds;
    displayTime(seconds);
  }
}

int secondsStep(int value){
  if (value < 0) return 0; //To prevent negative values
  if (value < 30) return 5;
  if (value < 120) return 10;
  if (value < 300) return 30;
  return 60;
}

void displayTime(int value){
  int minutesToDisplay = value / 60;
  int secondsToDisplay = value % 60;

  int digit0 = minutesToDisplay / 10;
  int digit1 = minutesToDisplay % 10;
  int digit2 = secondsToDisplay / 10;
  int digit3 = secondsToDisplay % 10;

  disp.clear();
  disp.display(digit0, digit1, digit2, digit3);
  disp.point(true);
}

void activate(){
  active = true;
  digitalWrite(output_pin, 1);
}
void deactivate(){
  active = false;
  digitalWrite(output_pin, 0);
}

void switch_state(){
  if(!active && seconds > 0) activate();
  else if (active) deactivate();
}

void process_interrupt_debounced(){
  if(millis() - lastDebounceTime > debounceDelay){
    switch_state();
    lastDebounceTime = millis();    
  }  
}
