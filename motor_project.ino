// Motor test stand: thrust (HX711 load cell) vs electrical power (INA260)
//
// Serial commands (115200 baud):
//   t        zero the load cell
//   k<grams> calibrate with a known weight on the cell, e.g. k100 (saved to EEPROM)
//   s        automatic sweep: one averaged row per throttle step
//   p<pct>   manual throttle, e.g. p50
//   x        stop the motor
// t, k and s only work while the motor is stopped. While it is spinning,
// any key except p stops it.

#include <Wire.h> //I2C communication
#include <EEPROM.h>
#include <Adafruit_INA260.h>
#include <HX711.h>
#include <Servo.h>

// Pins (same as the original sketch)
const uint8_t ESC_PIN = 9;
const uint8_t HX_DAT_PIN = 2; // HX711 DAT -> D2
const uint8_t HX_CLK_PIN = 3; // HX711 CLK -> D3

// Sweep settings
const int SWEEP_START_PCT = 10;
const int SWEEP_END_PCT = 100;
const int SWEEP_STEP_PCT = 10;
const unsigned long SETTLE_MS = 2000;  // ignore readings while thrust settles
const unsigned long AVERAGE_MS = 3000; // then average for this long

// Safety limits
const float MAX_CURRENT_A = 14.0; // INA260 is rated for 15 A
const unsigned long LOADCELL_TIMEOUT_MS = 1000;

// Calibration storage
const uint32_t CAL_MAGIC = 0xCA1B0001;

// Define sensors
Adafruit_INA260 ina;
HX711 loadcell;

// ESC
Servo esc;

// Variables
float calFactor = 1.0;
float pct = 0;
long tareRaw = 0;
unsigned long lastSampleMs = 0;

// Sweep state
bool sweeping = false;
int sweepPct = 0;
unsigned long stepStartMs = 0;
int64_t sumRaw = 0;
float sumV = 0, sumI = 0, sumP = 0;
unsigned int sampleCount = 0;

// throttle function
void setThrottle(float p) {
  pct = constrain(p, 0, 100);

  int pulseWidth = 1000 + 10 * pct; //1000 off, 2000 = max throttle

  esc.writeMicroseconds(pulseWidth);
}

void stopMotor(const char *reason) {
  setThrottle(0);
  sweeping = false;
  Serial.print("# stopped: ");
  Serial.println(reason);
}

void tare() {
  tareRaw = loadcell.read_average(10);
  lastSampleMs = millis();
}

void startStep(int p) {
  sweepPct = p;
  setThrottle(p);
  stepStartMs = millis();
  sumRaw = 0;
  sumV = sumI = sumP = 0;
  sampleCount = 0;
}

void printRow(float throttle, float busV, float currentA, float powerW, long raw) {
  float thrust = (raw - tareRaw) / calFactor;
  float eff = powerW > 0.05 ? thrust / powerW : 0;

  Serial.print(millis());
  Serial.print(',');
  Serial.print(throttle, 1);
  Serial.print(',');
  Serial.print(busV, 3);
  Serial.print(',');
  Serial.print(currentA, 3);
  Serial.print(',');
  Serial.print(powerW, 3);
  Serial.print(',');
  Serial.print(raw);
  Serial.print(',');
  Serial.print(thrust, 2);
  Serial.print(',');
  Serial.println(eff, 3);
}

void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  // Any key during a sweep stops it
  if (sweeping) {
    stopMotor("key pressed");
    return;
  }

  if (c == 'p') {
    setThrottle(Serial.parseFloat());
  } else if (c == 't' && pct == 0) {
    tare();
  } else if (c == 'k' && pct == 0) {
    float g = Serial.parseFloat();
    if (g > 0) {
      float k = (loadcell.read_average(10) - tareRaw) / g;
      lastSampleMs = millis();
      if (k != 0) {
        calFactor = k;
        EEPROM.put(0, CAL_MAGIC);
        EEPROM.put(4, calFactor);
        Serial.print("# calFactor=");
        Serial.println(calFactor, 4);
      }
    }
  } else if (c == 's' && pct == 0) {
    tare();
    Serial.println("# sweep start");
    sweeping = true;
    startStep(SWEEP_START_PCT);
  } else {
    stopMotor("stop command");
  }
}

void setup() {
  // Initialise ESC
  esc.attach(ESC_PIN);
  setThrottle(0);

  // serial communication
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Start sensors
  if (!ina.begin()) {
    Serial.println("# INA260 not found - check Vcc/GND/SDA/SCL");
    while (true) {}
  }
  // Average 16 samples (~35 ms) so ESC switching ripple doesn't land in single readings
  ina.setAveragingCount(INA260_COUNT_16);

  loadcell.begin(HX_DAT_PIN, HX_CLK_PIN);

  // Load saved calibration
  uint32_t magic;
  EEPROM.get(0, magic);
  if (magic == CAL_MAGIC) {
    float k;
    EEPROM.get(4, k);
    if (k != 0 && !isnan(k) && !isinf(k)) calFactor = k;
  }

  delay(3000);

  Serial.print("# calFactor=");
  Serial.println(calFactor, 4);
  Serial.println("t_ms,throttle_pct,bus_V,current_A,power_W,load_raw,thrust_gf,eff_gf_per_W");
  lastSampleMs = millis();
}

void loop() {
  handleSerial();

  // Only act when the load cell has a new sample, so a dead HX711
  // can't block the loop and lock out the stop command
  if (!loadcell.is_ready()) {
    if (pct > 0 && millis() - lastSampleMs > LOADCELL_TIMEOUT_MS) {
      stopMotor("load cell timeout");
    }
    return;
  }
  lastSampleMs = millis();

  // Read sensors
  long raw = loadcell.read();
  float busV = ina.readBusVoltage() / 1000;
  float currentA = ina.readCurrent() / 1000;
  float powerW = ina.readPower() / 1000;

  if (fabs(currentA) > MAX_CURRENT_A) {
    stopMotor("over current");
    return;
  }

  // Manual mode: log every sample
  if (!sweeping) {
    printRow(pct, busV, currentA, powerW, raw);
    return;
  }

  // Sweep mode: skip the settling time, then average, then print one row
  unsigned long elapsed = millis() - stepStartMs;
  if (elapsed < SETTLE_MS) return;

  sumRaw += raw;
  sumV += busV;
  sumI += currentA;
  sumP += powerW;
  sampleCount++;

  if (elapsed >= SETTLE_MS + AVERAGE_MS) {
    printRow(sweepPct, sumV / sampleCount, sumI / sampleCount, sumP / sampleCount,
             (long)(sumRaw / sampleCount));

    if (sweepPct + SWEEP_STEP_PCT > SWEEP_END_PCT) {
      setThrottle(0);
      sweeping = false;
      Serial.println("# sweep done");
    } else {
      startStep(sweepPct + SWEEP_STEP_PCT);
    }
  }
}
