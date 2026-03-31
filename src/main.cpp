#include <Arduino.h>

#include <Preferences.h>
#include <driver/ledc.h>
#include <driver/dac.h>
#include <Wire.h>
#include <AiEsp32RotaryEncoder.h>
#include <AiEsp32RotaryEncoderNumberSelector.h>
#include <SensirionCore.h>
#include <SensirionI2CSdp.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PID_v1.h>
#include <Smoothed.h>
#include "main.h"

const float Csupply = 0.90;
const float Cextract = 1.03;
const float flowFactor = 260.28;

// Pin definitions
const int PWM_FAN_PIN = 27; 

// 12C pins
const int I2C_SDA0_PIN = 21;
const int I2C_SCL0_PIN = 22;
const int I2C_SDA1_PIN = 25;
const int I2C_SCL1_PIN = 26;

// Rotary encoder
const int ROTARY_ENCODER_A_PIN = 2; // 18;
const int ROTARY_ENCODER_B_PIN = 4; //19;
const int ROTARY_ENCODER_BUTTON_PIN = 15; //5;
const int ROTARY_ENCODER_STEPS = 4;

// Display
const int I2C_ADDRESS_DISPLAY = 0x3C;
const int I2C_ADDRESS_BME280  = 0x76;
const int I2C_ADDRESS_SDP810  = 0x25;

// PWM settings
const int PWM_FAN_CHAN = LEDC_CHANNEL_0;
const int PWM_FREQ = 25000;    // 25 kHz frequency for computer fans
const int PWM_RESOLUTION = 10; // 10-bit resolution (0-1023)

//paramaters for button
const unsigned long shortPressAfterMiliseconds = 50;   // how long short press shoud be.
                                                       // Do not set too low to avoid bouncing (false press events).
const unsigned long longPressAfterMiliseconds = 1000;  // how long long press shoud be.

AiEsp32RotaryEncoder *rotaryEncoder = new AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN,
  ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS, false);
AiEsp32RotaryEncoderNumberSelector numberSelector = AiEsp32RotaryEncoderNumberSelector();

Preferences preferences;

Smoothed<float> zeroPressure;
Smoothed<float> flow;
Smoothed<float> flowPressure;
Smoothed<float> calibration;
Smoothed<float> pressureAmbient;
Smoothed<float> temperatureAmbient;
Smoothed<float> humidityAmbient;

/**
 * Two hardware I2C busses, one for the display, the BME280 and one SPD810 sensor.
 * The other for the second SDP810 sensor, to avoid interference between the two sensors.
 * The display and BME280 are on the same bus as they are not time critical and can handle the occasional delay 
 * and do not interfere with the timing of the SDP810 sensors.
 */
TwoWire I2C_A = TwoWire(0);
TwoWire I2C_B = TwoWire(1);

SensirionI2CSdp sdpFlow;
SensirionI2CSdp sdpZero;

// Initialize the OLED display using Wire library
Adafruit_SH1106G display(128, 64, &I2C_A);
Adafruit_BME280  bme280;     
hw_timer_t *timer0 = nullptr;
volatile bool ms10_passed = false;

ModeType modeType = MT_MEASURE;
ValveType valveType = VT_EXTRACT_RADIAL;
PidTuneType pidTuneType = PID_TUNE_NONE;

bool direction = false; // false for return

// lookup tabel
int lookupTable[4096];
int calibrateTable[4096];

// Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

// Specify the links and initial tuning parameters
double Kp=6, Ki=3, Kd=0;
PID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, REVERSE);

float offsetZeroPressure;
float offsetFlowPressure;

float compensationFactorEA = 0.0;
float compensationFactorER = 0.0;
float compensationFactorSA = 0.0;
float compensationFactorSR = 0.0;

float flowFactorExtractAxial  = Cextract * flowFactor;
float flowFactorExtractRadial = Cextract * flowFactor;
float flowFactorSupplyAxial   = Csupply * flowFactor;
float flowFactorSupplyRadial  = Csupply * flowFactor;

static void  initDisplay(void);
static void  displayAdjustSensorOffsetsProgress(int16_t progress);
static void  displayAdjustSensorOffsets(const char *sensor);
static void  displayTextNumber(const char *txt, float);
static void  displayMeasurements();
static void  displaySelectMode(ModeType type);
static void  displaySelectTunePID(PidTuneType type);
static void  displaySelectValveMode(ValveType type);
static void  displayCoefficientFlow();
static void  displayCoefficientZeroCompensation();
static void  initBME280();
static void  initSDP(SensirionI2CSdp&, TwoWire&);
static float calculateFlow(float dP);
static float calculateFlowCompensated(float dP);
static void  drawString(int16_t x, int16_t y, const String &text);
//////////////////////////////////////////////////////////////////////////

// Interrupt Service Routine (ISR)
// IRAM_ATTR places the function in RAM for faster execution
void IRAM_ATTR Timer0_ISR() {
  ms10_passed = true;
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR readEncoder_ISR() {
    rotaryEncoder->readEncoder_ISR();
}
//////////////////////////////////////////////////////////////////////////

void IRAM_ATTR readButton_ISR() {
    rotaryEncoder->readButton_ISR();
}
//////////////////////////////////////////////////////////////////////////

void setupTimer0() {
  // 1. Initialize timer
  //timerBegin(timer_number, divider, countUp)
  // Prescaler 80: 80MHz / 80 = 1MHz (1 microsecond per tick)
  timer0 = timerBegin(0, 80, true);

  // 2. Attach the ISR function
  timerAttachInterrupt(timer0, &Timer0_ISR, true);

  // 3. Set alarm to fire every 10000 ticks (10000 us = 10ms)
  // timerAlarmWrite(timer, microseconds, autoreload)
  timerAlarmWrite(timer0, 10000, true);

  // 4. Enable the alarm
  timerAlarmEnable(timer0);
}
//////////////////////////////////////////////////////////////////////////

void setupRotaryEncoder() {
  rotaryEncoder->begin();
  rotaryEncoder->setup(&readEncoder_ISR, &readButton_ISR);
  numberSelector.attachEncoder(rotaryEncoder);

  // numberSelector.setRange parameters:
  float minValue = -20.0;      // set minimum value for example -12.0
  float maxValue = 20.0;       // set maxinum value for example 12.0
  float step = 0.1;            // set step increment, default 1, can be smaller steps like 0.5 or 10
  bool cycleValues = false;    // set true only if you want going to miminum value after maximum 
  unsigned int decimals = 1;   // set precision - how many decimal places you want, default is 0
  numberSelector.setRange(minValue, maxValue,  step, cycleValues, decimals);
  numberSelector.setValue(0.0); // sets initial value
}
//////////////////////////////////////////////////////////////////////////

void setValveType(ValveType type) {
  valveType = type;
  
  switch(type) {
    case VT_EXTRACT_AXIAL:
    case VT_EXTRACT_RADIAL:
      direction = false;
      break;
    
    case VT_SUPPLY_AXIAL:  
    case VT_SUPPLY_RADIAL:
      direction = true;
      break;
  }
  pid.SetControllerDirection(direction ? REVERSE : DIRECT);

}
//////////////////////////////////////////////////////////////////////////

float getFloat(const char* key, float value = NAN) {
  float f;
  preferences.begin("airflow", false);
  f = preferences.getFloat(key, value);
  preferences.end();
  return f == NAN ? 0.0 : f;
}
//////////////////////////////////////////////////////////////////////////

void saveFloat(const char* key, float value) {
  preferences.begin("airflow", false);
  preferences.putFloat(key, value);
  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

void loadPreferences() {
  preferences.begin("airflow", true);

  offsetZeroPressure = preferences.getFloat("offsetZero", 0.0);
  offsetFlowPressure = preferences.getFloat("offsetFlow", 0.0);

  float temp = preferences.getFloat("coefExtractAx", 0.0);
  if (temp >= 0.8 && temp <= 1.2) {
    flowFactorExtractAxial = flowFactor * temp;
  }
  temp = preferences.getFloat("coefExtractRd", 0.0);
  if (temp >= 0.8 && temp <= 1.2) {
    flowFactorExtractRadial = flowFactor * temp;
  }
  temp = preferences.getFloat("coefSupplyAx", 0.0);
  if (temp >= 0.8 && temp <= 1.2) {
    flowFactorSupplyAxial = flowFactor * temp;
  }
  temp = preferences.getFloat("coefSupplyRd", 0.0);
  if (temp >= 0.8 && temp <= 1.2) {
    flowFactorSupplyRadial = flowFactor * temp;
  }
  compensationFactorEA = preferences.getFloat("compFactRA", 0.0);
  compensationFactorER = preferences.getFloat("compFactRB", 0.0);
  compensationFactorSA = preferences.getFloat("compFactSA", 0.0);
  compensationFactorSR = preferences.getFloat("compFactSB", 0.0);
  preferences.end();
}
//////////////////////////////////////////////////////////////////////////

void adjustSensorOffsetFlowPressure() {
  Serial.println("Adjusting offset flowsensor."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpFlow.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from flowsensor");
       break;
    }
    calibration.add(differentialPressure);
    if (i % 12 == 0) {
      displayAdjustSensorOffsetsProgress(i * 120 / 200);
    }
    delay(15);
  }

  offsetFlowPressure = calibration.get();
  saveFloat("offsetFlow", offsetFlowPressure);
  Serial.printf("offset flow %f\n", offsetFlowPressure);
  delay(500); 
}
//////////////////////////////////////////////////////////////////////////

void adjustSensorOffsetZeroPressure() {
  Serial.println("Adjusting offset zeropressuresensor."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpZero.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from zeropressuresensor");
       break;
    }
    calibration.add(differentialPressure);
    if (i % 12 == 0) {
      displayAdjustSensorOffsetsProgress(i * 120 / 200);
    }
    delay(15);
  }
  
  offsetZeroPressure = calibration.get();
  saveFloat("offsetZero", offsetZeroPressure);
  Serial.printf("offset zero %f\n", offsetZeroPressure);
  delay(500); 
}
//////////////////////////////////////////////////////////////////////////

void adjustSensorOffsets() {
  displayAdjustSensorOffsets("Zero");
  adjustSensorOffsetZeroPressure();
  displayAdjustSensorOffsets("Flow");
  adjustSensorOffsetFlowPressure();
}
//////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(115200);

  Serial.println("\nAirflowmeter is starting up...");

  loadPreferences();

  // setup RotaryEncoder
  setupRotaryEncoder();

  // Configure PWM
  Serial.println("Setup PWM");
  ledcSetup(PWM_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_FAN_PIN, PWM_FAN_CHAN);
  // Set initial fan speed to zero
  ledcWrite(PWM_FAN_CHAN, 0);
  delay(1000);

  analogReadResolution(12);

  // Start both busses
  I2C_A.begin(I2C_SDA0_PIN, I2C_SCL0_PIN, 700000);   // SDA, SCL
  I2C_B.begin(I2C_SDA1_PIN, I2C_SCL1_PIN, 100000);

  Serial.println("Setup Display");
  initDisplay();
  delay(500);
  Serial.println("Setup BME280");
  initBME280();
  delay(500);
  Serial.println("Setup SDP810 Flow");
  initSDP(sdpFlow, I2C_A);
  delay(500);
  Serial.println("Setup SDP810 Zero");
  initSDP(sdpZero, I2C_B);
  delay(500);

  Serial.println("setup smoothed average.");
  zeroPressure.begin(SMOOTHED_EXPONENTIAL, 2);
  flow.begin(SMOOTHED_AVERAGE, 50);
  flowPressure.begin(SMOOTHED_AVERAGE, 100);
  calibration.begin(SMOOTHED_AVERAGE, 200);
  pressureAmbient.begin(SMOOTHED_AVERAGE, 40);
  humidityAmbient.begin(SMOOTHED_AVERAGE, 40);
  temperatureAmbient.begin(SMOOTHED_AVERAGE, 40);
  delay(500);

  // initialize the PID variables
  Kp = getFloat("Kp", Kp);
  Ki = getFloat("Ki", Ki );
  Kd = getFloat("Kd", Kd);
  pidSetpoint = 0.0;
  pidInput = 0.0;

  // turn the PID on
  Serial.println("turn the PID on.");
  pid.SetTunings(Kp, Ki, Kd);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, 1023);
  pid.SetControllerDirection(direction ? REVERSE : DIRECT);
  
  delay(500);

  Serial.print("Setup timer interrupt\n");
  setupTimer0();
  delay(500);

  Serial.print("\n Setup done...\n\n");
  delay(500);
}
//////////////////////////////////////////////////////////////////////////

float getZeroCompensationFactor() {
  switch (valveType) {
    case VT_EXTRACT_AXIAL:
      return compensationFactorEA;
    case VT_EXTRACT_RADIAL:
      return compensationFactorER;
    case VT_SUPPLY_AXIAL:
      return compensationFactorSA;
    case VT_SUPPLY_RADIAL:
      return compensationFactorSR; 
  }
  return 0.0;
}
//////////////////////////////////////////////////////////////////////////

void setZeroCompensationFactor(float factor) {
  switch (valveType) {
    case VT_EXTRACT_AXIAL:
      compensationFactorEA = factor;
      break;
    case VT_EXTRACT_RADIAL:
      compensationFactorER = factor;
      break;
    case VT_SUPPLY_AXIAL:
      compensationFactorSA = factor;
      break;
    case VT_SUPPLY_RADIAL:
      compensationFactorSR = factor; 
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void saveDischargeCoefficient(float coef) {
  switch (valveType) {
    case VT_EXTRACT_AXIAL:
      flowFactorExtractAxial = flowFactor * coef;
      saveFloat("coefExtractAx", coef);
      break;
    case VT_EXTRACT_RADIAL:
      flowFactorExtractRadial = flowFactor * coef;
      saveFloat("coefExtractRd", coef);
      break;
    case VT_SUPPLY_AXIAL:
      flowFactorSupplyAxial = flowFactor * coef;
      saveFloat("coefSupplyAx", coef);
      break;
    case VT_SUPPLY_RADIAL:
      flowFactorSupplyRadial = flowFactor * coef; 
      saveFloat("coefSupplyRd", coef);
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void saveZeroCompensationFactor(float factor) {
  
  switch (valveType) {
    case VT_EXTRACT_AXIAL:
      compensationFactorEA = factor;
      saveFloat("compFactRA", compensationFactorEA);
      break;
    case VT_EXTRACT_RADIAL:
      compensationFactorER = factor;
      saveFloat("compFactRB", compensationFactorER);
      break;
    case VT_SUPPLY_AXIAL:
      compensationFactorSA = factor;
      saveFloat("compFactSA", compensationFactorSA);
      break;
    case VT_SUPPLY_RADIAL:
      compensationFactorSR = factor; 
      saveFloat("compFactSB", compensationFactorSR);
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void restoreZeroCompensationFactors() {
  switch(valveType) {
    case VT_EXTRACT_AXIAL:
      compensationFactorEA = getFloat("compFactRA", 0.0);
      pidSetpoint = compensationFactorEA * flowPressure.get();
      break;
    case VT_EXTRACT_RADIAL:
      compensationFactorER = getFloat("compFactRB", 0.0);
      pidSetpoint = compensationFactorER * flowPressure.get();
      break;
    case VT_SUPPLY_AXIAL:
      compensationFactorSA = getFloat("compFactSA", 0.0);
      pidSetpoint = compensationFactorSA * flowPressure.get();
      break;
    case VT_SUPPLY_RADIAL:
      compensationFactorSR = getFloat("compFactSB", 0.0); 
      pidSetpoint = compensationFactorSR * flowPressure.get();
      break;
  }
}
////////////////////////////////////////////////////////////////////////// 

float calculateZeroCompensationPressure() {
  return getZeroCompensationFactor() * flowPressure.get();
}
//////////////////////////////////////////////////////////////////////////

void initNextMode(ModeType type) {

  modeType = type;

  switch (type) {
    
    case MT_SELECT:
      numberSelector.setRange(1, 6,  1, true, 0);
      numberSelector.setValue(MT_SELECT_VALVE);
      displaySelectMode(MT_SELECT_VALVE);
      break;

    case MT_SELECT_VALVE:
      numberSelector.setRange(0, 3,  1, true, 0);
      numberSelector.setValue(valveType);
      displaySelectValveMode(valveType);
      break;

    case MT_MEASURE:
      break; 

    case MT_TUNE_ZERO_COMPENSATION:
      numberSelector.setRange(ZERO_COMP_MIN, ZERO_COMP_MAX, ZERO_COMP_STEP, false, 3);
      numberSelector.setValue(getZeroCompensationFactor());
      displayMeasurements();
      break;

    case MT_CALIBRATE_FLOW:
      numberSelector.setRange(0.8, 1.2, 0.001, false, 3);
      switch (valveType) {
        case VT_EXTRACT_AXIAL:
        case VT_EXTRACT_RADIAL:
          numberSelector.setValue(getFloat("coefReturn", 1.0));
          break;
        case VT_SUPPLY_AXIAL:
        case VT_SUPPLY_RADIAL:
          numberSelector.setValue(getFloat("coefSupply", 1.0));
          break;
      }
      break;

    case MT_TUNE_PID:
      pidTuneType = PID_TUNE_NONE;
      numberSelector.setRange(0, 3, 1, true, 0);
      numberSelector.setValue(PID_TUNE_NONE);
      displaySelectTunePID(PID_TUNE_NONE);
      break;

    case MT_ADJUST_OFFSET:
      adjustSensorOffsets();
      modeType = MT_MEASURE;
      displayMeasurements();
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void on_button_short_click() {
 
  switch (modeType) {
    
    case MT_MEASURE:
      modeType = MT_SELECT_VALVE;
      numberSelector.setRange(0, 3,  1, true, 0);
      numberSelector.setValue(valveType);
      displaySelectValveMode(valveType);
      break;

    case MT_SELECT:
      initNextMode((ModeType)(uint8_t)numberSelector.getValue()); 
      break;

    case MT_SELECT_VALVE:
      modeType = MT_MEASURE;
      setValveType((ValveType)(uint8_t)numberSelector.getValue());
      displayMeasurements();
      break;

    case MT_TUNE_ZERO_COMPENSATION:
      modeType = MT_MEASURE;
      saveZeroCompensationFactor(numberSelector.getValue());
      displayMeasurements();
      break;

    case MT_CALIBRATE_FLOW:
      modeType = MT_MEASURE;
      saveDischargeCoefficient(numberSelector.getValue());
      break;

    case MT_TUNE_PID:
      switch (pidTuneType) {
        case PID_TUNE_NONE:
          switch((PidTuneType)(uint8_t)numberSelector.getValue()) {
            case PID_TUNE_NONE:
              initNextMode(MT_SELECT);
              break;
            case PID_TUNE_P:
              pidTuneType = PID_TUNE_P;
              numberSelector.setRange(0, 20, 0.1, false, 1);
              numberSelector.setValue(Kp);
              displayMeasurements();
              break;
            case PID_TUNE_I:
              pidTuneType = PID_TUNE_I;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(Ki);
              displayMeasurements();
              break;
            case PID_TUNE_D:
              pidTuneType = PID_TUNE_D;
              numberSelector.setRange(0, 10, 0.1, false, 1);
              numberSelector.setValue(Kd);
              displayMeasurements();
              break;
          }
          break;

        case PID_TUNE_P:
          saveFloat("Kp", numberSelector.getValue());
          initNextMode(MT_TUNE_PID);
          break;

        case PID_TUNE_I:
          saveFloat("Ki", numberSelector.getValue());
          initNextMode(MT_TUNE_PID);
          break;
          
        case PID_TUNE_D:
          saveFloat("Kd", numberSelector.getValue());
          initNextMode(MT_TUNE_PID);
          break; 
      }
      break;           
  }
}
//////////////////////////////////////////////////////////////////////////

void on_button_long_click() {
  // toggle direction
  // direction = !direction;
  // myPID.SetControllerDirection(direction ?  REVERSE : DIRECT);
  
  switch(modeType) {
    case MT_MEASURE:
      initNextMode(MT_SELECT);
      break;

    case MT_TUNE_ZERO_COMPENSATION:
      restoreZeroCompensationFactors();
      modeType = MT_MEASURE;
      displayMeasurements();
      break;

    default:
      break;
  }
}
//////////////////////////////////////////////////////////////////////////

void handle_rotary_button() {
  static unsigned long lastTimeButtonDown = 0;
  static bool wasButtonDown = false;

  bool isEncoderButtonDown = rotaryEncoder->isEncoderButtonDown();
  
  if (isEncoderButtonDown) {
    if (!wasButtonDown) {
      wasButtonDown = true;
      // start measuring
      lastTimeButtonDown = millis();
    }
  // button is up
  } else if (wasButtonDown) {
    wasButtonDown = false;
    // click happened, lets see if it was short click, long click or just too short
    if (millis() - lastTimeButtonDown >= longPressAfterMiliseconds) {
      on_button_long_click();
    } else if (millis() - lastTimeButtonDown >= shortPressAfterMiliseconds) {
      on_button_short_click();
    }
  } 
}
//////////////////////////////////////////////////////////////////////////

void loopRotaryEncoder() {
  
  if (rotaryEncoder->encoderChanged()) {
    switch (modeType) {
      
      case MT_SELECT:
        displaySelectMode((ModeType)(uint8_t)numberSelector.getValue()); 
        break;
      
      case MT_SELECT_VALVE:
        displaySelectValveMode((ValveType)(uint8_t)numberSelector.getValue());
        break;
      
      case MT_MEASURE:
        break;
      
      case MT_TUNE_ZERO_COMPENSATION:
        setZeroCompensationFactor(numberSelector.getValue());
        displayCoefficientZeroCompensation();
        break;

      case MT_CALIBRATE_FLOW:
        switch (valveType) {
          case VT_EXTRACT_AXIAL:
            flowFactorExtractAxial = flowFactor * numberSelector.getValue();
            break;
          case VT_EXTRACT_RADIAL:
            flowFactorExtractRadial = flowFactor * numberSelector.getValue();
            break;
          case VT_SUPPLY_AXIAL:
            flowFactorSupplyAxial = flowFactor * numberSelector.getValue();
            break;
          case VT_SUPPLY_RADIAL:
            flowFactorSupplyRadial = flowFactor * numberSelector.getValue();
            break;
        }
        displayCoefficientFlow();
        break;

      case MT_TUNE_PID:
        switch (pidTuneType) {

          case PID_TUNE_NONE:
            displaySelectTunePID((PidTuneType)(uint8_t)numberSelector.getValue());
            break;

          case PID_TUNE_P:
            Kp = numberSelector.getValue();
            pid.SetTunings(Kp, Ki, Kd);
            displayTextNumber("Kp: %.2f", Kp);
            break;

          case PID_TUNE_I:
            Ki = numberSelector.getValue();
            pid.SetTunings(Kp, Ki, Kd);
            displayTextNumber("Ki: %.2f", Ki);
            break;
          
          case PID_TUNE_D:
            Kd = numberSelector.getValue();
            pid.SetTunings(Kp, Ki, Kd);
            displayTextNumber("Kd: %.2f", Kd);
            break;
        }
        break;    
    }
  } 
  handle_rotary_button();
}
//////////////////////////////////////////////////////////////////////////

void readPressureSensors() {
  
  if (ms10_passed == true) {
    ms10_passed = false;
    float differentialPressure, temperature;

    // 2ms time to read
    uint16_t error = sdpZero.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from ZeroPressure");
    } else {
      zeroPressure.add(differentialPressure - offsetZeroPressure);
    }

    // 2 ms time to read
    error = sdpFlow.readMeasurement(differentialPressure, temperature);
    if (error) {
      Serial.print("Error trying to execute readMeasurement from FlowPressure");
    } else {
      flowPressure.add(differentialPressure - offsetFlowPressure);
    }
  }
}
//////////////////////////////////////////////////////////////////////////

void loop() {
  static uint32_t loopcnt = 0;
  static uint32_t loopCount = 0;
  static uint32_t last_millis = 0;
  uint32_t ms = millis();
  
  ++loopCount;
  
  readPressureSensors();

  loopRotaryEncoder();

  // every 100ms
  if (ms >= last_millis + 100) {
    last_millis = ms;

    flow.add(calculateFlowCompensated(flowPressure.get()));

    pidSetpoint = calculateZeroCompensationPressure(); 
    pidInput = zeroPressure.get();
    
    pid.Compute();
    // Apply PWM to fan
    ledcWrite(PWM_FAN_CHAN, pidOutput);

    readPressureSensors();
    
    // every second
    if (loopcnt % 10 == 0) {

      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme280.takeForcedMeasurement(); // has no effect in normal mode
      readPressureSensors();

      // get the measurements from the BME280
      pressureAmbient.add(bme280.readPressure() / 100.0); // convert from Pa to hPa
      readPressureSensors();
      temperatureAmbient.add(bme280.readTemperature());
      readPressureSensors();
      humidityAmbient.add(bme280.readHumidity());
      readPressureSensors();

      Serial.printf("delay: %u\n", millis() - ms);
      Serial.printf("Loop count: %u\n", loopCount); // 168 loops/second
      Serial.printf("Flow pressure: %.1f Pa\n", flowPressure.get());
      Serial.printf("Zero presssure: %.1f Pa\n", zeroPressure.get());
      Serial.printf("PWM fan dutycycle: %.1f%%\n", pidOutput/10.23);
      loopCount = 0;
      // 13ms
      // Serial.printf("Loop duaration: %u\n", millis()- last_millis);
    }
    
    // every 2 seconds
    if (loopcnt++ % 20 == 0) {
      if (modeType == MT_MEASURE 
          || modeType == MT_TUNE_ZERO_COMPENSATION
          || modeType == MT_CALIBRATE_FLOW
          || pidTuneType == PID_TUNE_P
          || pidTuneType == PID_TUNE_I
          || pidTuneType == PID_TUNE_D
        ) {
        displayMeasurements(); // takes 42ms
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////

void initBME280(void) {
  
  if (bme280.begin(I2C_ADDRESS_BME280, &I2C_A)) {

    //weather monitoring scenario
    bme280.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF);
                    
    drawString(0, 12, "Setup BME280 ");                
    delay(1000);

  } else {
    char message[32];
    drawString(5, 10, "No valid BME280");
    delay(5000);
    snprintf(message, sizeof(message), "SensorID: 0x%02X", bme280.sensorID());
    drawString(5, 10, message);
    delay(5000);
  }
}
//////////////////////////////////////////////////////////////////////////

static void initSDP(SensirionI2CSdp& sdp, TwoWire& wire) {
  drawString(0, 0, "Setup SDP810 ");                
  delay(1000);
  
  sdp.begin(wire, SDP8XX_I2C_ADDRESS_0);

  uint16_t error;
  char errorMessage[256];

  uint32_t productNumber;
  uint8_t serialNumber[8];
  uint8_t serialNumberSize = 8;

  sdp.stopContinuousMeasurement();

  error = sdp.readProductIdentifier(productNumber, serialNumber,
    serialNumberSize);
  if (error) {
    drawString(0, 12, "Error ");                
    Serial.print("Error trying to execute readProductIdentifier(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    delay(5000);
  } else {
    Serial.print("ProductNumber:");
    Serial.print(productNumber);
    Serial.print("\t");
    Serial.print("SerialNumber:");
    Serial.print("0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
            Serial.print(serialNumber[i], HEX);
     }
     Serial.println();
  }

  //error = sdp.startContinuousMeasurementWithDiffPressureTCompAndAveraging();
  error = sdp.startContinuousMeasurementWithDiffPressureTComp();

  if (error) {
    Serial.print(
      "Error trying to execute "
      "startContinuousMeasurementWithDiffPressureTComp(): "
    );
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    delay(500);
  }
}
//////////////////////////////////////////////////////////////////////////

static void setOledContrast(uint8_t contrast, uint8_t precharge, uint8_t comdetect) {
  display.oled_command(SH110X_SETPRECHARGE); //0xD9
  display.oled_command(precharge); //0xF1 default, to lower the contrast, put 1-1F
  display.oled_command(SH110X_SETCONTRAST);
  display.oled_command(contrast);  // 0-255
  display.oled_command(SH110X_SETVCOMDETECT); //0xDB, (additionally needed to lower the contrast)
  display.oled_command(comdetect); //0x40 default, to lower the contrast, put 0
  display.oled_command(SH110X_DISPLAYALLON_RESUME);
  display.oled_command(SH110X_NORMALDISPLAY);
  display.oled_command(SH110X_DISPLAYON);
}
//////////////////////////////////////////////////////////////////////////

static void setOledBrightness(uint8_t brightness) {
  uint8_t contrast = brightness;
  if (brightness < 128) {
    // Magic values to get a smooth/ step-free transition
    contrast = brightness * 1.171;
  } else {
    contrast = brightness * 1.171 - 43;
  }

  uint8_t precharge = 241;
  if (brightness == 0) {
    precharge = 0;
  }
  uint8_t comdetect = brightness / 8;

  setOledContrast(contrast, precharge, comdetect);
}
//////////////////////////////////////////////////////////////////////////

static void initDisplay(void) {
  display.begin(0x3C, true);
  //display.setBrightness(0x60);
  setOledBrightness(0x60);
  //display.flipScreenVertically(); // turn the display upside down;
  // ArialMT_Plain_10, ArialMT_Plain_16, ArialMT_Plain_24
  //display.setFont(ArialMT_Plain_16);
  //display.setTextSize(16);
  //display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setTextSize(1);          // schaalfactor
  //display.drawString(0, 24, "Display ready");
  display.setCursor(0,24);
  display.print("Display ready");
  display.display();
  delay(1000);
}
//////////////////////////////////////////////////////////////////////////

static void printAlignCenter(const char *text, int16_t x, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor(x-(w>>1), y);
  display.print(text);
}
//////////////////////////////////////////////////////////////////////////

static void printAlignRight(const char *text, int16_t x, int16_t y) {
  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  display.setCursor(x-w, y);
  display.print(text);
}
//////////////////////////////////////////////////////////////////////////

// Draws a string at the given location
static void drawString(int16_t x, int16_t y, const String &text) {
  display.clearDisplay();
  // display.setTextAlignment(TEXT_ALIGN_LEFT);
  // display.setFont(ArialMT_Plain_10);
  display.setTextSize(1);
  // display.drawString(x, y, text);
  display.setCursor(x, y);
  display.print(text);
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displayCoefficientZeroCompensation() {
  // display setpoint zero pressure
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Cz: %.3f", numberSelector.getValue());
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayCoefficientFlow() {
  // display setpoint zero pressure
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Cd: %.3f", numberSelector.getValue());
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static const char* getValveTypeText(ValveType type) {

  switch (type) {
    case VT_EXTRACT_AXIAL:
      return "EXTRACT_AX";
    case VT_EXTRACT_RADIAL:
      return "EXTRACT_RD";
    case VT_SUPPLY_AXIAL:
      return "SUPPLY_AX";
    case VT_SUPPLY_RADIAL:
      return "SUPPLY_RD";
  }
  return "        ";
}
//////////////////////////////////////////////////////////////////////////

static void displayTextNumber(const char *txt, float number) {
  // display setpoint zero pressure
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("          ");
  readPressureSensors();
  display.setCursor(0, 0);
  display.printf(txt, number);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayAdjustSensorOffsets(const char *sensor) {
  display.clearDisplay();
  readPressureSensors();
  display.setTextSize(1);
  display.setCursor(0, 28);
  display.printf("Offset %s...", sensor);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayAdjustSensorOffsetsProgress(int16_t progress) {
  display.fillRect(0, 40, progress, 10, SH110X_WHITE);
  readPressureSensors();
  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displayMeasurements() {
  char message[32];
  
  display.clearDisplay();

  readPressureSensors();

  display.setTextSize(1);
  display.setCursor(0, 0);

  if (modeType == MT_TUNE_ZERO_COMPENSATION) {
    display.printf("Cz: %.3f", numberSelector.getValue());
  } else if (modeType == MT_CALIBRATE_FLOW) {
    display.printf("Cd: %.3f", numberSelector.getValue());
  } else if (pidTuneType == PID_TUNE_P) {
    // display proportional gain PID controller
    display.printf("Kp: %.2f", numberSelector.getValue());
  } else if (pidTuneType == PID_TUNE_I) {
    // display integral gain PID controller
    display.printf("Ki: %.2f", numberSelector.getValue());
  } else if (pidTuneType == PID_TUNE_D) {
    // display derivative gain PID controller
    display.printf("Kd: %.2f", numberSelector.getValue());     
  } else {
    // display setpoint zero pressure
    display.printf("Sz: %.1fPa", pidSetpoint);
  }
  readPressureSensors();

  // display current hood valve type
  printAlignRight(getValveTypeText(valveType), 127, 0);
  readPressureSensors();

  // display flow
  snprintf(message, sizeof(message),"%.1f", flow.get());
  display.setFont(); // standaard font
  display.setTextSize(2);
  printAlignCenter(message, 63, 18);
  readPressureSensors();
  display.setTextSize(1);
  display.setCursor(0, 21);
  display.print("Flow");
  readPressureSensors();
  printAlignRight("m3/h", 127, 21);
  readPressureSensors();

  // display zero pressure
  display.setCursor(0, 41);
  display.printf("Pz %.1fPa", zeroPressure.get());
  readPressureSensors();

  // display temperature
  snprintf(message, sizeof(message),"%.1f C", temperatureAmbient.get());
  printAlignRight(message, 127, 41);
  readPressureSensors();

  // display absolute pressure
  display.setCursor(0, 57);
  display.printf("%.1f hPa", pressureAmbient.get());
  readPressureSensors();

  // display humidity
  snprintf(message, sizeof(message),"%.1f %%RH", humidityAmbient.get());
  printAlignRight(message, 127, 57);
  readPressureSensors();

  display.display();
  readPressureSensors();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectMode(ModeType mtype) {
  
  display.clearDisplay();
  display.setTextSize(1);

  if (mtype == MT_SELECT_VALVE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Select Valve");
  if (mtype == MT_SELECT_VALVE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_MEASURE)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Measure Mode");
  if (mtype == MT_MEASURE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_ZERO_COMPENSATION) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Tune Zero Comp");
  if (mtype == MT_TUNE_ZERO_COMPENSATION) display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  
  if (mtype == MT_CALIBRATE_FLOW) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Tune Flow");
  if (mtype == MT_CALIBRATE_FLOW) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_TUNE_PID) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 37);
  display.print("Tune PID");
  if (mtype == MT_TUNE_PID) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (mtype == MT_ADJUST_OFFSET) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 46);
  display.print("Adjust Offset");
  if (mtype == MT_ADJUST_OFFSET) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void displaySelectTunePID(PidTuneType type) {
  display.clearDisplay();
  display.setTextSize(1);

  if (type == PID_TUNE_NONE) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 1);
  display.print("Back");
  if (type == PID_TUNE_NONE) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_P) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 10);
  display.print("Tune Kp");
  if (type == PID_TUNE_P) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_I)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 19);
  display.print("Tune Ki");
  if (type == PID_TUNE_I) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == PID_TUNE_D) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 28);
  display.print("Tune Kd");
  if (type == PID_TUNE_D) display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  
  display.display();
}
//////////////////////////////////////////////////////////////////////////

static void  displaySelectValveMode(ValveType type) {
  display.clearDisplay();
  display.setTextSize(1);

  if (type == VT_EXTRACT_AXIAL) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 0);
  display.print("Extract Axial");
  if (type == VT_EXTRACT_AXIAL) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VT_EXTRACT_RADIAL)  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 8);
  display.print("Extraxt Radial");
  if (type == VT_EXTRACT_RADIAL) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VT_SUPPLY_AXIAL) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 16);
  display.print("Supply Axial");
  if (type == VT_SUPPLY_AXIAL) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  if (type == VT_SUPPLY_RADIAL) display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 24);
  display.print("Supply Radial");
  if (type == VT_SUPPLY_RADIAL) display.setTextColor(SH110X_WHITE, SH110X_BLACK);

  display.display();

}
//////////////////////////////////////////////////////////////////////////

/**
  *                    ____________
  *                  /  dP * T
  * Q  = 0.0723  \  / ____________  (m³/s)
  *               \/      Pabs
  *
  *                                  ____________ 
  *                                 /  dP * T  
  * Q = 3600 x 0.0723 = 260.28  \  / ____________  (m³/h)
  *                              \/     Pabs
  * 
*/

static float getValveCoefficient() {

  switch (valveType) {
    case VT_EXTRACT_AXIAL:
      return flowFactorExtractAxial;
    case VT_EXTRACT_RADIAL:
      return flowFactorExtractRadial;
    case VT_SUPPLY_AXIAL:
      return flowFactorSupplyAxial;
    case VT_SUPPLY_RADIAL:
      return flowFactorSupplyRadial;
  }
  return 0.0;
}
//////////////////////////////////////////////////////////////////////////

static float calculateFlowCompensated(float dP) {
  float Pa = pressureAmbient.get() * 100.0;
  float Ta = temperatureAmbient.get() + 273.15;
  //float C = direction ? flowFactorSupply : flowFactorReturn;
  float C = getValveCoefficient();

  return dP > 0.0 ? C * sqrt(dP * Ta / Pa) : 0.0; // (m³/h)
}
//////////////////////////////////////////////////////////////////////////

static float calculateFlow(float dP) {

  //float v1 = sqrt(dP/1.509); // (m/s)
  //float v1 = 0.814 * sqrt(dP); // (m/s)
  //float Q = 0.00478 * v1; // (m³/s)
  //return Q * 3600.0; // (m³/h)
  //return 17.208 * v1; // (m³/h)
  return dP > 0.0 ? 14.00732 * sqrt(dP) : 0.0; // (m³/h)
}
//////////////////////////////////////////////////////////////////////////
