#include <Arduino.h>

#include <driver/ledc.h>
#include <driver/dac.h>
#include <Wire.h>
#include "SensirionCore.h"
#include "SensirionI2CSdp.h"
#include <Adafruit_SH110X.h>
#include <Fonts/Picopixel.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <PID_v1.h>
#include <Smoothed.h>
#include <LittleFS.h>

// Pin definitions
const int PWM_FAN_PIN = 27;  // 12 = GPIO 32
const int PWM_CAL_PIN = 13;
const int CAL_PIN = A0;      // GPIO 36 = A0 = VP
const int ZERO_PIN = A6;    // GPIO 34 = A6
const int FLOW_PIN = A7;    // GPIO 35 = A7, uses any valid Ax pin as you wish

const int I2C_SDA0_PIN = 21;
const int I2C_SCL0_PIN = 22;
const int I2C_SDA1_PIN = 25;
const int I2C_SCL1_PIN = 26;

// Display
const int I2C_ADDRESS_DISPLAY = 0x3C;
const int I2C_ADDRESS_BME280  = 0x76;
const int I2C_ADDRESS_SDP810  = 0x25;

// PWM settings
const int PWM_FAN_CHAN = LEDC_CHANNEL_0;
const int PWM_CAL_CHAN = LEDC_CHANNEL_4;
const int PWM_FREQ = 25000;    // 25 kHz frequency for computer fans
const int CAL_FREQ = 5000;     // 5 KHz
const int PWM_RESOLUTION = 10; // 10-bit resolution (0-1023)
const int CAL_RESOLUTION = 12; // 12-bit resolution (0-4095)


Smoothed<float> zeroPressure;
Smoothed<float> flowPressure;
Smoothed<float> adcCalibration;
Smoothed<float> pressureAmbient;
Smoothed<float> temperatureAmbient;
Smoothed<float> humidityAmbient;

// Twee hardware I2C bussen
TwoWire I2C_A = TwoWire(0);
TwoWire I2C_B = TwoWire(1);

SensirionI2CSdp sdpZero;
SensirionI2CSdp sdpFlow;

// Initialize the OLED display using Wire library
// ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically
// based on your board's pins_arduino.h 
// e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
// SH1106Wire      display(I2C_ADDRESS_DISPLAY, I2C_SDA0_PIN, I2C_SCL0_PIN, GEOMETRY_128_64);
Adafruit_SH1106G display(128, 64, &I2C_A);
Adafruit_BME280  bme280;     // I2C

// lookup tabel
int lookupTable[4096];
int calibrateTable[4096];

// Define Variables we'll be connecting to
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
double Kp=10, Ki=3, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);

float offsetZero;
float offsetFlow;

static void  initDisplay(void);
static void  displayMeasurements();
static void  initBME280();
static void  initSDP(SensirionI2CSdp&, TwoWire&);
static float calculateFlow(float dP);
static float calculateFlowCompensated(float dP);
static void  drawString(int16_t x, int16_t y, const String &text);
void         calibrateAdc();
const char*  readLine(File file);
int          readFile(const char* filename);

void setup() {
  Serial.begin(115200);

  Serial.println("\nAirflowmeter is starting up...");

  // Configure PWM
  Serial.println("Setup PWM");
  ledcSetup(PWM_FAN_CHAN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_FAN_PIN, PWM_FAN_CHAN);
  ledcSetup(PWM_CAL_CHAN, CAL_FREQ, CAL_RESOLUTION);
  ledcAttachPin(PWM_CAL_PIN, PWM_CAL_CHAN);
 
  // Set initial fan speed to zero
  ledcWrite(PWM_FAN_CHAN, 0);
  ledcWrite(PWM_CAL_CHAN, 0);
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
  Serial.println("Setup SDP810 Zero");
  initSDP(sdpZero, I2C_A);
  delay(500);
  Serial.println("Setup SDP810 Flow");
  initSDP(sdpFlow, I2C_B);
  delay(500);

  Serial.println("setup smoothed average.");
  zeroPressure.begin(SMOOTHED_AVERAGE, 20);
  flowPressure.begin(SMOOTHED_AVERAGE, 20);
  adcCalibration.begin(SMOOTHED_AVERAGE, 100);
  pressureAmbient.begin(SMOOTHED_AVERAGE, 40);
  humidityAmbient.begin(SMOOTHED_AVERAGE, 40);
  temperatureAmbient.begin(SMOOTHED_AVERAGE, 40);
  delay(500);
  /*
  Serial.println("setup LitteFS.");
  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    delay(1000);
    Serial.println("starting calibration procedure.");
    calibrateAdc();
  } else {
    Serial.printf("LittleFS: totalbytes: %u, usedbytes: %u.\n", LittleFS.totalBytes(), LittleFS.usedBytes());
    Serial.println("Reading file: /luptable.txt");
    if (readFile("/luptable.txt") < 0) {
      Serial.println("Reading file failed");
      delay(1000);
      Serial.println("starting calibration procedure.");
      calibrateAdc();
    }
  }//*/

  //
  Serial.println("Getting  offset zeropressure."); 
  for (size_t i = 0; i < 200; i++) {
    float differentialPressure;
    float temperature;
    uint16_t error = sdpZero.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from zeropresuresensor");
       break;
    }
    zeroPressure.add(differentialPressure);
    delay(10);
  }
  delay(500);
  // initialize the variables we're linked to
  Setpoint = Input = offsetZero = zeroPressure.get();
  Serial.printf("offset zero %.1f\n", offsetZero);

  //
  Serial.println("Getting offset flowsensor."); 
  for (size_t i = 0; i < 200; i++) {
    //flowPressure.add(lookupTable[analogRead(FLOW_PIN)]);
    //delay(2);
    float differentialPressure;
    float temperature;
    uint16_t error = sdpFlow.readMeasurement(differentialPressure, temperature);
    if (error) {
       Serial.print("Error trying to execute readMeasurement() from flowsensor");
       break;
    }
    flowPressure.add(differentialPressure);
    delay(10);
  }
  delay(500);
  offsetFlow = flowPressure.get();
  Serial.printf("offset flow %.1f\n", offsetFlow);

  // turn the PID on
  Serial.println("turn the PID on.");
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 1023);
  delay(500);

  Serial.print("\n Setup done...\n\n");
  delay(500);
}

void loop() {
  static uint32_t loopcnt = 0;
  static uint32_t last_millis = 0;
  float differentialPressure;
  float temperature;
  uint16_t error;

  error = sdpZero.readMeasurement(differentialPressure, temperature);
  if (error) {
    Serial.print("Error trying to execute readMeasurement from ZeroPressure");
  } else {
    zeroPressure.add(differentialPressure);
  }

  error = sdpFlow.readMeasurement(differentialPressure, temperature);
  if (error) {
    Serial.print("Error trying to execute readMeasurement from FlowPressure");
  } else {
    flowPressure.add(differentialPressure);
  }

  if (millis() >= last_millis + 100) {
    last_millis = millis();
    
    Input = zeroPressure.get();
    myPID.Compute();
    // Apply PWM to fan
    ledcWrite(PWM_FAN_CHAN, Output);
    
    // every second
    if (loopcnt % 10 == 0) {

      // Only needed in forced mode! In normal mode, you can remove the next line.
      bme280.takeForcedMeasurement(); // has no effect in normal mode

      // get the measurements from the BME280
      pressureAmbient.add(bme280.readPressure() / 100.0); // convert from Pa to hPa
      temperatureAmbient.add(bme280.readTemperature());
      humidityAmbient.add(bme280.readHumidity());

      Serial.printf("Flow pressure: %.1f Pa\n", flowPressure.get() - offsetFlow);
      Serial.printf("Zero presssure: %.1f Pa\n", zeroPressure.get() - offsetZero);
      Serial.printf("PWM fan dutycycle: %.1f%%\n", Output/1023 * 100.0);
    }
    
    // every 2 seconds
    if (loopcnt++ % 20 == 0) {
      displayMeasurements();
    }
  } else {
    delay(2);
  }
}

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
  display.setTextColor(SH110X_WHITE);
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

static void displayMeasurements() {
  char message[32];
  float pz = zeroPressure.get() - offsetZero;
  float pf = flowPressure.get() - offsetFlow;
  float flow = calculateFlowCompensated(pf);
  float temperature = temperatureAmbient.get();
  float humidity = humidityAmbient.get();
  float pressure = pressureAmbient.get();
  
  display.clearDisplay();

  // display P_flow and P_zero
  // snprintf(message, sizeof(message),"Pf %.1f Pa", pf);
  // display.setFont(ArialMT_Plain_10)
  // display.setTextAlignment(TEXT_ALIGN_LEFT);
  // display.drawString(8, 0, message);
  // display.setFont(&TomThumb);
  // display.setTextColor(SH110X_WHITE);
  // display.setFont(&Picopixel);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("f %.1fPa", pf);
  snprintf(message, sizeof(message),"z %.1fPa", pz);
  // display.setTextAlignment(TEXT_ALIGN_RIGHT);
  // display.drawString(119, 0, message);
  printAlignRight(message, 127,0);
  
  // display Flow
  snprintf(message, sizeof(message),"%.1f", flow);
  // display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.setFont(); // standaard font
  display.setTextSize(2);
  // display.drawString(64, 14, message);
  printAlignCenter(message, 63,18);
  // display.setFont(ArialMT_Plain_10);
  display.setTextSize(1);
  // display.setTextAlignment(TEXT_ALIGN_LEFT);
  // display.drawString(8, 21, "FLow");
  display.setCursor(8, 21);
  display.print("FLow");
  // display.setTextAlignment(TEXT_ALIGN_RIGHT);
  // display.drawString(119,21, "m³/h");
  printAlignRight("m3/h", 119, 21);

  // display temperature
  // display.setFont(ArialMT_Plain_10);
  display.setTextSize(1);
  // display.setTextAlignment(TEXT_ALIGN_LEFT);
  snprintf(message, sizeof(message),"%.1f C", temperature);
  // display.drawString(8, 41, message);
  display.setCursor(8, 41);
  display.print(message);

  // display humidity
  // display.setFont(ArialMT_Plain_10);
  display.setTextSize(1);
  // display.setTextAlignment(TEXT_ALIGN_RIGHT);
  snprintf(message, sizeof(message),"%.1f %%RH", humidity);
  // display.drawString(119, 41, message);
  printAlignRight(message, 119, 41);

  // display pressure
  // display.setFont(ArialMT_Plain_10);
  display.setTextSize(1);
  // display.setTextAlignment(TEXT_ALIGN_CENTER);
  snprintf(message, sizeof(message),"%.1f hPa", pressure);
  // display.drawString(64, 54, message);
  printAlignCenter(message, 63, 56);
 
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
  *                             \/      Pabs
  * 
*/

static float calculateFlowCompensated(float dP) {
  float Pabs = pressureAmbient.get() * 100.0;
  float Tamb = temperatureAmbient.get() + 273.15;

  return dP > 0.0 ? 260.28 * sqrt(dP * Tamb / Pabs) : 0.0; // (m³/h)
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

// generate LUT - reverse array
void createLUT() {
  int min_idx = 0;
  int last_pos = 0;
  int diff = 1;
  int last_val = calibrateTable[0];

  Serial.println("Reversing data / Generating LUT...");

  for (size_t i = 0; i < sizeof(lookupTable) / sizeof(int); i++) {
    if (--diff <= 0) {
      while (calibrateTable[last_pos] <= last_val) {
        last_pos++;
      }
      diff = calibrateTable[last_pos] - last_val;
      last_val = calibrateTable[last_pos];
    }
    lookupTable[i] = min_idx;
    min_idx = last_pos;
  }
}

void calibrateAdc() {
  
  delay(1000);
  for (size_t i = 0; i < sizeof(calibrateTable) / sizeof(int); i++) {
    ledcWrite(PWM_CAL_CHAN, i);
    // Serial.printf("iteration i: %u\n", i);
    delay(10);
    for (size_t j = 0; j < 100; j++) {
      delayMicroseconds(200);
      adcCalibration.add(analogRead(CAL_PIN));
    }
    calibrateTable[i] = adcCalibration.get() + 0.5;
    Serial.printf("Cal[%u]: %d (%0.3fV)\n", i, calibrateTable[i], 3.3 * calibrateTable[i] / 4096);
    delay(10);
  }

  // 3. check if values are ascending and fix if not
  Serial.println("Checking if ADC values are ascending and fixing them if not...");
  for (size_t i = 1; i <  sizeof(calibrateTable) / sizeof(int); i++) {
    if (calibrateTable[i] < calibrateTable[i - 1]) {
      Serial.printf("Warning: values not ascending, index: %u\n", i);
      calibrateTable[i] = calibrateTable[i-1];
    }
  }

  // 5. generate LUT - reverse array
  createLUT();

  // print the look-up table
  Serial.print("Priting the ADC correction look-up table, to be used in your code.\n\n");
  Serial.print("const float ADC_LUT[4096] = {");
  for (size_t i = 0; i < sizeof(lookupTable) / sizeof(int); i++) {
    if (i % 16 == 0)
      Serial.println();
    Serial.printf("%d,", lookupTable[i]);
  }
  Serial.print("\n};\n\n");
}

int readFile(const char* filename) {
  const char *buf, *p;
  size_t i = 0, j = 0;

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.printf("Failed to open file %s for reading\n", filename);
    return -1;
  }
  
  Serial.println("File Content:");
  delay(1000
  );

  while ((buf = readLine(file)) != nullptr) {
    p = buf;
    Serial.println(buf);
    while (sscanf(p, "%d,", &lookupTable[i]) == 1) {
      i++;
      p = strchr(p, ',');
      if (i >= 4096 || p == nullptr)
        break;
      p++;
    }
    do {
      Serial.printf("%d,", lookupTable[j++]);
    } while ((j % 16) != 0);
    Serial.println("");
    if (i >= 4096)
      break;
    // memset((void*)buf, 0, 128);
  }
  file.close();
  return 1; 
}

const char* readLine(File file) {
  static char buf[128];
  size_t i = 0;
  char c;
  while (file.available()) {
    if ((c = file.read()) != '\n') {
       buf[i++] = c;
       if (i >= sizeof(buf)) {
          // bufferoverlow
          return nullptr;
       }
    } else {
       break;
    }
  }
  if (i) {
    buf[i] = '\0';
    return buf;
  }
  return nullptr;
}
