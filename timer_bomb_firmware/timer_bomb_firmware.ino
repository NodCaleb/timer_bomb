#include "GyverTM1637.h"
#define output_pin 9
#define rt_clk_pin 4
#define rt_dt_pin 5
#define rt_sw_pin 6
#define disp_clk_pin 7
#define disp_dio_pin 8
#define play_pause_pin 2
#define stop_pin 3
#define time_up_sound_pin 10
#define warning_sound_pin 11

int rotation;
int value;
bool skip = false;
bool armed = false;
bool update_display = false;
bool warning = false;
bool time_up = false;

int initial_seconds = 0;
int seconds = 0;
int new_seconds = 0;

unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 300;    // the debounce time; increase if the output flickers

GyverTM1637 disp(disp_clk_pin, disp_dio_pin);

void setup() {

  cli();//stop interrupts

  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  sei();//allow interrupts

  Serial.begin (9600);
  pinMode(output_pin, OUTPUT);
  pinMode (rt_clk_pin,INPUT);
  pinMode (rt_dt_pin,INPUT);
  pinMode (rt_sw_pin,INPUT);
  pinMode (play_pause_pin,INPUT);
  pinMode (stop_pin,INPUT);
  pinMode (time_up_sound_pin,INPUT);
  pinMode (warning_sound_pin,INPUT);
  rotation = digitalRead(rt_clk_pin);
  Serial.println("Start");

  disp.clear();
  disp.brightness(7);  // яркость, 0 - 7 (минимум - максимум)
  displayTime(seconds);

  attachInterrupt(digitalPinToInterrupt(play_pause_pin), process_play_pause, RISING);
  attachInterrupt(digitalPinToInterrupt(stop_pin), process_stop, RISING);
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1H
  if (armed){
    seconds--;
    update_display = true;
    if (seconds == 0){
      armed = false;
      time_up = true;
    }
    else if (seconds <= 5){
      warning = true;
    }
  }
}

void loop() {

  if (warning){
    warning = false;
    if (digitalRead(warning_sound_pin)) beep(100);
  }

  if (!armed){
    value = digitalRead(rt_clk_pin);
     if (value != rotation){ // we use the DT pin to find out which way we turning.

      if(skip){ //Skipping every other signal, cause that fucking selector sends 2 in 1 click
        skip = false;
      }
      else{
        skip = true;
        if (digitalRead(rt_dt_pin) != value) {  // Clockwise
          seconds += secondsStep(seconds);
        } else { //Counterclockwise
          seconds -= secondsStep(seconds-1);
          if(seconds < 0) seconds = 0;
        }
        update_display = true;
        initial_seconds = seconds;
      }          
   } 
   rotation = value;

   if(!digitalRead(rt_sw_pin)){
     seconds = 0;
     update_display = true;
     initial_seconds = seconds;
   } 
  }

  if (update_display){
    displayTime(seconds);
    update_display = false;
  }

  if (time_up){
    time_up = false;
    if (digitalRead(time_up_sound_pin)) beep(1500);
    else if (digitalRead(warning_sound_pin)) beep(100); //To avoid silent final in case of warning sounds
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

void process_play_pause(){
  if(millis() - lastDebounceTime > debounceDelay){
    
    if(seconds > 0) {
      armed = !armed;
      warning = true; 
    }
    
    lastDebounceTime = millis();    
  }  
}

void process_stop(){
  if(millis() - lastDebounceTime > debounceDelay){
    
    armed = false;
    seconds = initial_seconds;
    update_display = true;
    warning = true;
    
    lastDebounceTime = millis();    
  }  
}

void beep(int time){
  digitalWrite(output_pin, 1);
  delay(time);
  digitalWrite(output_pin, 0);
}
