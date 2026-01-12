#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN   48   // Pin LED internal ESP32 S3 DevKitC1
#define NUMPIXELS 1  

#define DOUT_1 18  // DOUT HX710B pertama (MPS20N0040D-S)
#define SCK_1  17  // SCK HX710B pertama

#define DOUT_2 11  // DOUT HX710B kedua (Venturi)
#define SCK_2  10  // SCK HX710B kedua

#define BUTTON_PIN 9  // Tombol untuk kalibrasi
#define D1 0.025  // Diameter inlet (25 mm)
#define D2 0.010  // Diameter throat (10 mm)
#define RHO 1.225 // Massa jenis udara (kg/m³) pada suhu ruangan

volatile bool calibrate = false; // Flag untuk interrupt
long offset1 = 0, offset2 = 0;   // Offset masing-masing sensor

Adafruit_NeoPixel strip(NUMPIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

float lastP1 = 0, lastP2 = 0;
bool inhaling = false;

float totalVolume = 0.0;  // Total akumulasi volume udara yang dihirup (liter)
int inhaleCount = 0;  // Jumlah siklus inhale
float accumulatedVolume = 0.0;
unsigned long lastTime = 0;
float lastAirflow = 0.0;

// HX710B Reading function with validation
long readHX710B(int doutPin, int sckPin) {
  long value = 0;
  while (digitalRead(doutPin)); // Tunggu sampai data siap

  for (int i = 0; i < 24; i++) {
    digitalWrite(sckPin, HIGH);
    delayMicroseconds(1);
    value = (value << 1) | digitalRead(doutPin);
    digitalWrite(sckPin, LOW);
    delayMicroseconds(1);
  }

  // Untuk High-Speed Mode, tambahkan 3 pulse clock
  for (int i = 0; i < 3; i++) {
    digitalWrite(sckPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(sckPin, LOW);
    delayMicroseconds(1);
  }

  // Validasi nilai pembacaan
  return (value < 0 || value > 16777216) ? 0 : value;
}

// Fungsi menghitung airflow berdasarkan tekanan dengan validasi
float calculateAirflow(float P1, float P2) {
  float dP = P1 - P2;
  if (dP <= 0) return 0;

  float A2 = (PI * pow(D2, 2)) / 4.0;
  float term = 1 - pow(D2 / D1, 4);
  
  return (term > 0) ? A2 * sqrt((2 * dP) / (RHO * term)) : 0;
}

// ISR untuk tombol kalibrasi
void IRAM_ATTR handleCalibration() {
  calibrate = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(SCK_1, OUTPUT);
  pinMode(DOUT_1, INPUT);
  pinMode(SCK_2, OUTPUT);
  pinMode(DOUT_2, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleCalibration, FALLING);

  strip.begin();
  strip.show();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0; // Konversi ms ke s
  lastTime = currentTime;

  // Jika tombol ditekan, lakukan kalibrasi
  if (calibrate) {
    calibrate = false;

    Serial.println("Kalibrasi dimulai...");
    strip.setPixelColor(0, strip.Color(255, 255, 0)); // Warna kuning saat kalibrasi
    strip.show();

    long sum1 = 0, sum2 = 0;
    int samples = 100;

    for (int i = 0; i < samples; i++) {
      sum1 += readHX710B(DOUT_1, SCK_1);
      sum2 += readHX710B(DOUT_2, SCK_2);
      delay(5);
    }

    long avg = (sum1 + sum2) / (2 * samples);
    offset1 = avg - (sum1 / samples);
    offset2 = avg - (sum2 / samples);

    Serial.println("Kalibrasi selesai!");
    Serial.print("Offset Sensor 1: "); Serial.println(offset1);
    Serial.print("Offset Sensor 2: "); Serial.println(offset2);

    strip.setPixelColor(0, strip.Color(0, 255, 0)); // Hijau setelah kalibrasi
    strip.show();
  }

  // Baca nilai tekanan dan sesuaikan dengan offset
  long pressure1 = readHX710B(DOUT_1, SCK_1) + offset1;
  long pressure2 = readHX710B(DOUT_2, SCK_2) + offset2;

  float P1 = (pressure1 / 16777216.0) * 40000;
  float P2 = (pressure2 / 16777216.0) * 40000;
  float airflow = calculateAirflow(P1, P2);

  // Deteksi inhale dan akumulasi volume udara dengan metode trapezoidal
  if (airflow > 0.0005) {
    inhaling = true;
    accumulatedVolume += ((lastAirflow + airflow) / 2.0) * dt;
    strip.setPixelColor(0, strip.Color(0, 0, 255)); // Biru saat inhale
    strip.show();
  } else if (inhaling) {
    inhaling = false;
    inhaleCount++;
    totalVolume += accumulatedVolume * 1000; // Konversi ke liter

    Serial.print("Volume Inhale: ");
    Serial.print(accumulatedVolume * 1000);
    Serial.println(" L");

    accumulatedVolume = 0.0;
    strip.setPixelColor(0, strip.Color(122, 122, 122)); // Abu-abu saat idle
    strip.show();
  }
  
  lastAirflow = airflow;

  // Cek perubahan nilai untuk logging
  if (P1 != lastP1 || P2 != lastP2 || airflow != lastAirflow) {
    Serial.print("Airflow: ");
    Serial.print(airflow * 1000);
    Serial.println(" L/s");
    Serial.print("Total Volume: "); Serial.print(totalVolume); Serial.println(" L");
    Serial.print("Jumlah Inhalasi: "); Serial.println(inhaleCount);
    
    lastP1 = P1;
    lastP2 = P2;
  }

  delay(1000);
}
