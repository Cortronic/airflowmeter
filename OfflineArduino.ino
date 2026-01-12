#include "Pid.h"
#include "Globals.h"


/*
===============
Global vars
===============
*/
volatile unsigned long pulseCount;
volatile unsigned long avgCount;
volatile unsigned int interruptCounts; //Accumulator used to reduce timer interrupt rate

volatile unsigned int pidOutput;
volatile unsigned int pidSetPoint;

volatile unsigned int ledStatus;

Pid controller(KP, KI, KD, MOTOR_MIN, MOTOR_MAX);

/*
===============
Function prototypes
===============
*/

void Timer2Setup(void);
unsigned int getSetPointValue(void);




void setup(){
    Serial.begin(115200);

    //Change default PWM frequency (Timer 1)
    //TCCR0B = TCCR0B & B11111000 | B00000001;
    
    pinMode(PIN_FLOW_SENSOR_INPUT, INPUT_PULLUP);
    pinMode(PIN_WATER_PUMP, OUTPUT);

    pinMode(PIN_SETPOINT_ADJ_BUTTON, INPUT_PULLUP);
    pinMode(PIN_SETPOINT_ADJ_INPUT, INPUT);
    
    pinMode(LED_BUILTIN, OUTPUT);

    

    digitalWrite(PIN_WATER_PUMP, 0); //Disable water pump by default
    
    digitalWrite(LED_BUILTIN, 0);

    
    ledStatus = 0;
    pulseCount = 0;
    avgCount = 0;
    interruptCounts = 0;
    

    pidOutput = 0;
    pidSetPoint = 0;

    while(digitalRead(PIN_SETPOINT_ADJ_BUTTON)){
      pidSetPoint = getSetPointValue();
      Serial.print("Set point: ");
      Serial.println(pidSetPoint);
    }

    controller.resetIDvalues();
    controller.pidSetPoint(pidSetPoint);

    Timer2Setup();
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR_INPUT), countISR, CHANGE);
}

void loop(){
  delay(MAX_TIMER_INTERRUPTS*100/8);
  Serial.print(avgCount);

  pidOutput = (int)controller.pidUpdate(avgCount);

  analogWrite(PIN_WATER_PUMP, pidOutput);

  Serial.print(", ");
  Serial.println(pidOutput);

  //If a new set point is meant to be set, press push button
  while(!digitalRead(PIN_SETPOINT_ADJ_BUTTON)){
      pidSetPoint = getSetPointValue();
      controller.resetIDvalues();
      controller.pidSetPoint(pidSetPoint);
      Serial.print("Set point: ");
      Serial.println(pidSetPoint);
  }

  
  
}


void Timer2Setup(void){
  /*
  =============
  TIMER 2 SETUP (System Tick)
  =============
  */

  //General interrupts enable bit clear (Status Register)
  SREG &= ~0x80;

  //Disable power-saving feature (enables Timer 2)
  PRR &= ~0x40;

  //Compare A interrupt enable
  TIMSK2 |= 0x02; 

  //Clear timer value on compare match mode (CTC mode)
  TCCR2A = 0x02;
  TCCR2B &= ~0x08;

  //7.8125 KHz Clk Source for Timer 2
  //195 counts compare match generates interrupt
  //Interrupt rate = 40.064 Hz
  
  //Prescaler /1024 clk source //31.25 Hz (T = 32 ms)
  TCCR2B |= 0x07;

  //Comparator A register limit (overflow value)
  OCR2A = 194; //195 - 1

  //General interrupt enable bit set (Status Register)
  SREG |= 0x80; 
}


unsigned int getSetPointValue(void){
    unsigned int x;
    x = analogRead(PIN_SETPOINT_ADJ_INPUT);
    x = map(x, 0, 1020, SETPOINT_MIN, SETPOINT_MAX);
    return x;
}


void countISR(void){
    ++pulseCount;
}


ISR(TIMER2_COMPA_vect){ //Timer comparison interrupt
  if(++interruptCounts >= MAX_TIMER_INTERRUPTS - 1){
    ledStatus ^= 1;
    digitalWrite(LED_BUILTIN, ledStatus);
    avgCount = pulseCount;
    pulseCount = 0;
    interruptCounts = 0;
  }
}
