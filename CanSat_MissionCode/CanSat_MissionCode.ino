#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>
#include <TimeLib.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_INA219.h>
#include <Adafruit_DPS310.h>
#include <TinyGPSPlus.h>

// ---------------------- PIN DEFINITIONS ----------------------
#define SERVO_PIN        5    // parachute servo release
#define ESC_PIN          6    // reaction wheel ESC signal
#define BUZZER_PIN       9    // recovery beeper
#define OZONE_PIN        A0   // MQ-131 analog
#define SD_CS_PIN        BUILTIN_SDCARD // Teensy 4.1 internal uSD

// ---------------------- I2C BUSES ----------------------
// Wire  (SDA=18, SCL=19)   -> BNO055, BMP280, INA219
// Wire1 (SDA1=17, SCL1=16) -> DPS310

// ---------------------- SERIAL PORTS ----------------------
// GNSS / NavIC on Serial1 (pin0=RX1, pin1=TX1)
// XBee on Serial2 (pin7=RX2, pin8=TX2)
#define GPS_SERIAL Serial1
#define GPS_BAUD   115200   // use 115200 for your NavIC module output
HardwareSerial& xbeeSerial = Serial2;

// ---------------------- GLOBAL OBJECTS ----------------------
TinyGPSPlus gps;
Adafruit_BNO055  bno(55, 0x28);   // IMU
Adafruit_BMP280  bmp;             // BMP280 baro (0x77)
Adafruit_INA219  ina219;          // INA219 power sensor
Adafruit_DPS310  dps;             // DPS310 baro (Wire1 @0x76/0x77)
Servo parachuteServo;
Servo reactionWheelESC;

// ---------------------- RUNTIME FLAGS ----------------------
bool ina219_ok = false;
bool dps_ok    = false;

// GPS / NavIC connection tracking
unsigned long lastReceived = 0;
const unsigned long GPS_TIMEOUT = 5000; // ms with no bytes -> gpsConnected=false
bool gpsConnected = false;

// ---------------------- MISSION STATE MACHINE ----------------------
//
// 0 = BOOT
// 1 = ASCENT
// 2 = GYRO_ACTIVE
// 3 = DESCENT (parachute deployed)
// 4 = LANDED
// 5 = RECOVERY_BEEP (beacon beeping)
enum MissionState {
  STATE_BOOT = 0,
  STATE_ASCENT = 1,
  STATE_GYRO_ACTIVE = 2,
  STATE_DESCENT = 3,
  STATE_LANDED = 4,
  STATE_RECOVERY_BEEP = 5
};
MissionState missionState = STATE_BOOT;

// ---------------------- CONSTANTS ----------------------
const float SEA_LEVEL_HPA = 1013.25;        // tune on launch day
const float ESC_START_ALT_M      = 700.0;   // start gyro spin
const float PARACHUTE_DROP_M     = 350.0;   // deploy chute after losing 350 m from peak BMP altitude
const unsigned long LANDING_STABLE_TIME_MS = 10000;

const int ESC_MIN_US   = 1000;
const int ESC_MAX_US   = 2000;
const int ESC_IDLE_US  = 1100;

// ---------------------- STATEFUL FLIGHT VARS ----------------------
uint32_t packetCount        = 0;
uint32_t missionStartMillis = 0;
unsigned long lastPacketMs  = 0;
const unsigned long PACKET_INTERVAL_MS = 2000;

float peakAlt_m_bmp   = 0.0;
bool  peakInitialized = false;

bool parachuteDeployed        = false;
bool escCalibratedAndSpinning = false;

bool landingCandidate   = false;
unsigned long landedCheckStartMs = 0;

// ---------------------- LOGGING ----------------------
String missionLogFilename = "";
String gpsLogFilename     = "";

// ---------------------- FORWARD DECLARATIONS ----------------------
void initSensors();
void initLogging();
void createMissionLogFile();
void createGPSLogFile();

void readAllSensors(
  float &bmp_tempC,
  float &bmp_pressure_hPa,
  float &bmp_alt_m,
  float &dps_tempC,
  float &dps_pressure_hPa,
  float &dps_alt_m,
  float &accel_x,
  float &accel_y,
  float &accel_z,
  float &orient_x,
  float &orient_y,
  float &orient_z,
  float &busVoltage_V_raw,
  float &current_mA_raw,
  int   &ozoneRaw,
  double &lat,
  double &lng,
  float &gpsAlt_m,
  int   &sats,
  float &hdop_val,
  float &speed_kmh,
  bool  &fixValid
);

void logMissionCSVLine(
  const String &dateStr,
  const String &rtcTimeStr,
  const String &missionTimeStr,
  float bmp_pressure_hPa,
  float bmp_tempC,
  float dps_pressure_hPa,
  float dps_tempC,
  float dps_alt_m,
  float bmp_alt_m,
  double lat,
  double lng,
  float gpsAlt_m,
  int   sats,
  float accel_x,
  float accel_y,
  float accel_z,
  float orient_x,
  float orient_y,
  float orient_z,
  float busVoltage_V,
  float current_mA,
  float batteryPct,
  int   ozoneRaw,
  int   statusField,
  float teensyVinEst
);

void logGPSBlockToFile();

void sendTelemetryPacket(
  const String &dateStr,
  const String &rtcTimeStr,
  const String &missionTimeStr,
  float bmp_pressure_hPa,
  float bmp_tempC,
  float dps_pressure_hPa,
  float dps_tempC,
  float dps_alt_m,
  float bmp_alt_m,
  double lat,
  double lng,
  float gpsAlt_m,
  int   sats,
  float accel_x,
  float accel_y,
  float accel_z,
  float orient_x,
  float orient_y,
  float orient_z,
  float busVoltage_V,
  float current_mA,
  float batteryPct,
  int   ozoneRaw,
  int   statusField,
  float teensyVinEst
);

// flight state logic
void flightStateLogic(float bmp_alt_m, float accel_x, float accel_y, float accel_z);
void maybeStartGyro(float bmp_alt_m);
void maybeDeployParachute(float bmp_alt_m);
void updateLandingDetection(float bmp_alt_m, float accelMag);

// actuators
void rampESCOnce();
void deployParachute();

// buzzer
void beepPatternStartup();
void beepRecoveryLoop();
void beepOnce(uint16_t onMs, uint16_t offMs);

// GPS service
void serviceGPS();

// helpers
float simulateBusVoltage();
float simulateCurrentmA();
float computeBatteryPercent(float vbat);
float computeTeensyVinEst(float vbat);

String formatMissionTime();
void   getRealDateTime(String &dateStr, String &rtcTimeStr);

void setRTCFromCompile();
time_t compileTime();

// ---------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  GPS_SERIAL.begin(GPS_BAUD);   // GNSS UART
  xbeeSerial.begin(9600);       // XBee (keep 9600 unless changed)

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  parachuteServo.attach(SERVO_PIN);
  parachuteServo.write(0); // locked
  reactionWheelESC.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  reactionWheelESC.writeMicroseconds(ESC_MIN_US); // arm ESC low

  Wire.begin();   // main I2C
  Wire1.begin();  // secondary I2C for DPS310

  Serial.println("Initializing sensors...");
  initSensors();

  initLogging();

  setRTCFromCompile();

  missionStartMillis = millis();
  lastPacketMs       = millis();
  packetCount        = 0;

  missionState       = STATE_BOOT;
  parachuteDeployed  = false;
  escCalibratedAndSpinning = false;
  landingCandidate   = false;
  landedCheckStartMs = 0;
  peakInitialized    = false;

  // initialize peak altitude from BMP
  {
    float bmp_tempC, bmp_pressure_hPa, bmp_alt_m;
    float dps_tempC, dps_pressure_hPa, dps_alt_m;
    float ax,ay,az,ox,oy,oz;
    float vbus_raw,cur_raw;
    int   ozoneRaw;
    double la,lo;
    float gpsAlt_m;
    int   sats;
    float hdop_val, speed_kmh;
    bool  fixValid;

    readAllSensors(
      bmp_tempC, bmp_pressure_hPa, bmp_alt_m,
      dps_tempC, dps_pressure_hPa, dps_alt_m,
      ax,ay,az,
      ox,oy,oz,
      vbus_raw,cur_raw,
      ozoneRaw,
      la,lo,
      gpsAlt_m,
      sats,
      hdop_val,
      speed_kmh,
      fixValid
    );

    peakAlt_m_bmp   = bmp_alt_m;
    peakInitialized = true;
  }

  Serial.print("MISSION LOG: "); Serial.println(missionLogFilename);
  Serial.print("GPS LOG: ");     Serial.println(gpsLogFilename);
  Serial.println("RTC set from compile time.");
  Serial.println("System ARMED.");

  beepPatternStartup();

  missionState = STATE_ASCENT;
}

// ---------------------- LOOP ----------------------
void loop() {
  // 1. Feed incoming GNSS data
  serviceGPS();

  // 2. Read all sensors
  float bmp_tempC, bmp_pressure_hPa, bmp_alt_m;
  float dps_tempC, dps_pressure_hPa, dps_alt_m;
  float accel_x, accel_y, accel_z;
  float orient_x, orient_y, orient_z;
  float busVoltage_V_raw, current_mA_raw;
  int   ozoneRaw;
  double lat, lng;
  float gpsAlt_m;
  int   sats;
  float hdop_val, speed_kmh;
  bool  fixValid;

  readAllSensors(
    bmp_tempC,
    bmp_pressure_hPa,
    bmp_alt_m,
    dps_tempC,
    dps_pressure_hPa,
    dps_alt_m,
    accel_x,
    accel_y,
    accel_z,
    orient_x,
    orient_y,
    orient_z,
    busVoltage_V_raw,
    current_mA_raw,
    ozoneRaw,
    lat,
    lng,
    gpsAlt_m,
    sats,
    hdop_val,
    speed_kmh,
    fixValid
  );

  // 3. power model
  float busVoltage_V = simulateBusVoltage();
  float current_mA   = simulateCurrentmA();

  // 4. peak tracking for parachute logic
  if (peakInitialized) {
    if (bmp_alt_m > peakAlt_m_bmp) {
      peakAlt_m_bmp = bmp_alt_m;
    }
  } else {
    peakAlt_m_bmp   = bmp_alt_m;
    peakInitialized = true;
  }

  // 5. flight logic
  flightStateLogic(bmp_alt_m, accel_x, accel_y, accel_z);

  // 6. derived values
  float batteryPct   = computeBatteryPercent(busVoltage_V);
  float teensyVinEst = computeTeensyVinEst(busVoltage_V);

  // 7. time strings
  String missionTimeStr = formatMissionTime();
  String dateStr, rtcTimeStr;
  getRealDateTime(dateStr, rtcTimeStr);

  // 8. every 2s: write logs + send telemetry
  unsigned long nowMs = millis();
  if (nowMs - lastPacketMs >= PACKET_INTERVAL_MS) {
    lastPacketMs = nowMs;

    // log to mission CSV
    logMissionCSVLine(
      dateStr,
      rtcTimeStr,
      missionTimeStr,
      bmp_pressure_hPa,
      bmp_tempC,
      dps_pressure_hPa,
      dps_tempC,
      dps_alt_m,
      bmp_alt_m,
      lat,
      lng,
      gpsAlt_m,
      sats,
      accel_x,
      accel_y,
      accel_z,
      orient_x,
      orient_y,
      orient_z,
      busVoltage_V,
      current_mA,
      batteryPct,
      ozoneRaw,
      (int)missionState,
      teensyVinEst
    );

    // send packet over XBee
    sendTelemetryPacket(
      dateStr,
      rtcTimeStr,
      missionTimeStr,
      bmp_pressure_hPa,
      bmp_tempC,
      dps_pressure_hPa,
      dps_tempC,
      dps_alt_m,
      bmp_alt_m,
      lat,
      lng,
      gpsAlt_m,
      sats,
      accel_x,
      accel_y,
      accel_z,
      orient_x,
      orient_y,
      orient_z,
      busVoltage_V,
      current_mA,
      batteryPct,
      ozoneRaw,
      (int)missionState,
      teensyVinEst
    );

    packetCount++;

    // log GNSS snapshot to GPS_LOGx.csv
    logGPSBlockToFile();
  }

  // 9. recovery beep if landed
  if (missionState == STATE_RECOVERY_BEEP) {
    beepRecoveryLoop();
  }

  delay(20);
}

// ---------------------- SENSOR INIT ----------------------
void initSensors() {
  if (!bno.begin()) {
    Serial.println("⚠️ BNO055 init FAIL");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  if (!bmp.begin(0x77)) {
    Serial.println("⚠️ BMP280 init FAIL");
    while (1);
  } else {
    Serial.println("✅ BMP280 initialized");
  }

  if (!ina219.begin()) {
    Serial.println("⚠️ INA219 init FAIL (simulating V/I)");
    ina219_ok = false;
  } else {
    ina219_ok = true;
    Serial.println("✅ INA219 initialized (simulation still overrides)");
  }

  if (!dps.begin_I2C(0x77, &Wire1)) {
    if (!dps.begin_I2C(0x76, &Wire1)) {
      Serial.println("⚠️ DPS310 init FAIL on Wire1");
      dps_ok = false;
    } else {
      Serial.println("✅ DPS310 initialized @0x76 on Wire1");
      dps_ok = true;
    }
  } else {
    Serial.println("✅ DPS310 initialized @0x77 on Wire1");
    dps_ok = true;
  }
}

// ---------------------- LOG INIT ----------------------
void initLogging() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("⚠️ SD init FAIL. NO LOGGING.");
    missionLogFilename = "";
    gpsLogFilename = "";
    return;
  }

  createMissionLogFile();
  createGPSLogFile();
}

void createMissionLogFile() {
  int fileNumber = 1;
  do {
    missionLogFilename = "MISSION_" + String(fileNumber) + ".csv";
    fileNumber++;
  } while (SD.exists(missionLogFilename.c_str()));

  File f = SD.open(missionLogFilename.c_str(), FILE_WRITE);
  if (f) {
    f.println("TEAM_ID,PACKET_COUNT,DATE,RTC_TIME,MISSION_TIME,"
              "BMP_PRESSURE_hPa,BMP_TEMP_C,DPS_PRESSURE_hPa,DPS_TEMP_C,"
              "DPS_ALT_m,BMP_ALT_m,LAT,LON,GPS_ALT_m,SATELLITES,"
              "ACCEL_X,ACCEL_Y,ACCEL_Z,"
              "ORIENT_X,ORIENT_Y,ORIENT_Z,"
              "VOLTAGE_V,CURRENT_mA,BATTERY_PCT,OZONE_RAW,STATUS,TEENSY_VIN_EST");
    f.close();
    Serial.print("MISSION LOG CREATED: ");
    Serial.println(missionLogFilename);
  } else {
    Serial.println("⚠️ Could not create mission log file");
    missionLogFilename = "";
  }
}

void createGPSLogFile() {
  int fileNumber = 1;
  do {
    gpsLogFilename = "GPS_LOG" + String(fileNumber) + ".csv";
    fileNumber++;
  } while (SD.exists(gpsLogFilename.c_str()));

  File g = SD.open(gpsLogFilename.c_str(), FILE_WRITE);
  if (g) {
    g.println("Timestamp,Status,Latitude,Longitude,Altitude,Satellites,HDOP,Speed,Date,Time");
    g.close();
    Serial.print("GPS LOG CREATED: ");
    Serial.println(gpsLogFilename);
  } else {
    Serial.println("⚠️ Could not create GPS log file");
    gpsLogFilename = "";
  }
}

// ---------------------- SENSOR READ BUNDLE ----------------------
void readAllSensors(
  float &bmp_tempC,
  float &bmp_pressure_hPa,
  float &bmp_alt_m,
  float &dps_tempC,
  float &dps_pressure_hPa,
  float &dps_alt_m,
  float &accel_x,
  float &accel_y,
  float &accel_z,
  float &orient_x,
  float &orient_y,
  float &orient_z,
  float &busVoltage_V_raw,
  float &current_mA_raw,
  int   &ozoneRaw,
  double &lat,
  double &lng,
  float &gpsAlt_m,
  int   &sats,
  float &hdop_val,
  float &speed_kmh,
  bool  &fixValid
) {
  // BMP280
  bmp_tempC        = bmp.readTemperature();
  bmp_pressure_hPa = bmp.readPressure() / 100.0F;
  bmp_alt_m        = bmp.readAltitude(SEA_LEVEL_HPA);

  // DPS310
  if (dps_ok) {
    sensors_event_t temp_event, pressure_event;
    if (dps.getEvents(&temp_event, &pressure_event)) {
      dps_tempC        = temp_event.temperature;
      dps_pressure_hPa = pressure_event.pressure;
      dps_alt_m        = dps.readAltitude(SEA_LEVEL_HPA);
    } else {
      dps_tempC        = NAN;
      dps_pressure_hPa = NAN;
      dps_alt_m        = NAN;
    }
  } else {
    dps_tempC        = NAN;
    dps_pressure_hPa = NAN;
    dps_alt_m        = NAN;
  }

  // BNO055
  sensors_event_t orientationData, accelData, gyroData, magData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelData,       Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroData,        Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magData,         Adafruit_BNO055::VECTOR_MAGNETOMETER);

  orient_x = orientationData.orientation.x;
  orient_y = orientationData.orientation.y;
  orient_z = orientationData.orientation.z;

  accel_x = accelData.acceleration.x;
  accel_y = accelData.acceleration.y;
  accel_z = accelData.acceleration.z;

  // INA219 (raw read, not used in final packet because we simulate anyway)
  if (ina219_ok) {
    busVoltage_V_raw = ina219.getBusVoltage_V();
    current_mA_raw   = ina219.getCurrent_mA();
  } else {
    busVoltage_V_raw = 0.0;
    current_mA_raw   = 0.0;
  }

  // Ozone
  ozoneRaw = analogRead(OZONE_PIN);

  // GPS / GNSS fields
  if (gpsConnected && gps.location.isValid()) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    fixValid = true;
  } else {
    lat = 0.0;
    lng = 0.0;
    fixValid = false;
  }

  if (gpsConnected && gps.altitude.isValid()) {
    gpsAlt_m = gps.altitude.meters();
  } else {
    gpsAlt_m = 0.0;
  }

  if (gpsConnected && gps.satellites.isValid()) {
    sats = gps.satellites.value();
  } else {
    sats = 0;
  }

  if (gpsConnected && gps.hdop.isValid()) {
    hdop_val = gps.hdop.hdop();
  } else {
    hdop_val = 0.0;
  }

  if (gpsConnected && gps.speed.isValid()) {
    speed_kmh = gps.speed.kmph();
  } else {
    speed_kmh = 0.0;
  }
}

// ---------------------- FLIGHT STATE LOGIC ----------------------
void flightStateLogic(float bmp_alt_m, float accel_x, float accel_y, float accel_z) {
  maybeStartGyro(bmp_alt_m);
  maybeDeployParachute(bmp_alt_m);

  float accelMag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
  updateLandingDetection(bmp_alt_m, accelMag);

  if (missionState == STATE_LANDED) {
    missionState = STATE_RECOVERY_BEEP;
  }
}

void maybeStartGyro(float bmp_alt_m) {
  if (!escCalibratedAndSpinning && bmp_alt_m >= ESC_START_ALT_M) {
    rampESCOnce();
    escCalibratedAndSpinning = true;
    if (missionState < STATE_GYRO_ACTIVE) {
      missionState = STATE_GYRO_ACTIVE; // 2
    }
  }
}

void maybeDeployParachute(float bmp_alt_m) {
  if (!parachuteDeployed) {
    float drop = peakAlt_m_bmp - bmp_alt_m;
    if (drop >= PARACHUTE_DROP_M) {
      deployParachute();
      parachuteDeployed = true;
      if (missionState < STATE_DESCENT) {
        missionState = STATE_DESCENT; // 3
      }
    }
  }
}

void updateLandingDetection(float bmp_alt_m, float accelMag) {
  static float lastAlt = 0;
  static unsigned long lastAltTime = 0;

  unsigned long nowMs = millis();
  float altDiff = fabs(bmp_alt_m - lastAlt);
  unsigned long dt = nowMs - lastAltTime;

  if (dt > 0) {
    bool stableAlt   = (altDiff < 0.5);
    bool stableAccel = (accelMag > 0.8 && accelMag < 1.2); // ~1g steady

    if (stableAlt && stableAccel) {
      if (!landingCandidate) {
        landingCandidate = true;
        landedCheckStartMs = nowMs;
      } else {
        if ((nowMs - landedCheckStartMs) > LANDING_STABLE_TIME_MS) {
          if (missionState < STATE_LANDED) {
            missionState = STATE_LANDED; // 4
            Serial.println("[EVENT] LANDED detected!");
          }
        }
      }
    } else {
      landingCandidate = false;
    }
  }

  lastAlt = bmp_alt_m;
  lastAltTime = nowMs;
}

// ---------------------- ACTUATORS ----------------------
void rampESCOnce() {
  Serial.println("[ESC] Gyro spin-up starting...");
  for (int pulse = ESC_MIN_US; pulse <= ESC_MAX_US; pulse += 5) {
    reactionWheelESC.writeMicroseconds(pulse);
    delay(30);
  }
  reactionWheelESC.writeMicroseconds(ESC_IDLE_US);
  Serial.println("[ESC] Gyro active, holding idle spin.");
}

void deployParachute() {
  Serial.println("[EVENT] Parachute DEPLOYED!");
  parachuteServo.write(90); // unlock/deploy angle
}

// ---------------------- BUZZER ----------------------
void beepOnce(uint16_t onMs, uint16_t offMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(onMs);
  digitalWrite(BUZZER_PIN, LOW);
  delay(offMs);
}

void beepPatternStartup() {
  beepOnce(200, 200);
  beepOnce(200, 200);
  beepOnce(200, 200);
}

void beepRecoveryLoop() {
  static unsigned long lastBeepMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastBeepMs >= 1000) {
    lastBeepMs = nowMs;
    digitalWrite(BUZZER_PIN, HIGH);
    delay(150);
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// ---------------------- GPS SERVICE ----------------------
void serviceGPS() {
  while (GPS_SERIAL.available() > 0) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
    lastReceived = millis();
    gpsConnected = true;
  }
  if (gpsConnected && (millis() - lastReceived > GPS_TIMEOUT)) {
    gpsConnected = false;
  }
}

// ---------------------- LOG WRITERS ----------------------
void logMissionCSVLine(
  const String &dateStr,
  const String &rtcTimeStr,
  const String &missionTimeStr,
  float bmp_pressure_hPa,
  float bmp_tempC,
  float dps_pressure_hPa,
  float dps_tempC,
  float dps_alt_m,
  float bmp_alt_m,
  double lat,
  double lng,
  float gpsAlt_m,
  int   sats,
  float accel_x,
  float accel_y,
  float accel_z,
  float orient_x,
  float orient_y,
  float orient_z,
  float busVoltage_V,
  float current_mA,
  float batteryPct,
  int   ozoneRaw,
  int   statusField,
  float teensyVinEst
) {
  if (missionLogFilename == "") return;
  File f = SD.open(missionLogFilename.c_str(), FILE_WRITE);
  if (!f) return;

  f.print("062,"); // TEAM_ID
  f.print(packetCount); f.print(",");
  f.print(dateStr); f.print(",");
  f.print(rtcTimeStr); f.print(",");
  f.print(missionTimeStr); f.print(",");

  f.print(bmp_pressure_hPa, 2); f.print(",");
  f.print(bmp_tempC, 2); f.print(",");
  f.print(dps_pressure_hPa, 2); f.print(",");
  f.print(dps_tempC, 2); f.print(",");
  f.print(dps_alt_m, 2); f.print(",");
  f.print(bmp_alt_m, 2); f.print(",");

  f.print(lat, 8); f.print(",");
  f.print(lng, 8); f.print(",");
  f.print(gpsAlt_m, 2); f.print(",");
  f.print(sats); f.print(",");

  f.print(accel_x, 2); f.print(",");
  f.print(accel_y, 2); f.print(",");
  f.print(accel_z, 2); f.print(",");

  f.print(orient_x, 2); f.print(",");
  f.print(orient_y, 2); f.print(",");
  f.print(orient_z, 2); f.print(",");

  f.print(busVoltage_V, 2); f.print(",");
  f.print(current_mA, 2); f.print(",");
  f.print(batteryPct, 1); f.print(",");
  f.print(ozoneRaw); f.print(",");
  f.print(statusField); f.print(",");
  f.print(teensyVinEst, 2);

  f.println();
  f.close();
}

void logGPSBlockToFile() {
  if (gpsLogFilename == "") return;
  File g = SD.open(gpsLogFilename.c_str(), FILE_WRITE);
  if (!g) return;

  // Timestamp (ms since boot)
  g.print(millis());
  g.print(",");

  // Status
  if (gps.location.isValid()) g.print("VALID");
  else                        g.print("INVALID");
  g.print(",");

  // Latitude
  if (gps.location.isValid()) g.print(gps.location.lat(), 8);
  else                        g.print("N/A");
  g.print(",");

  // Longitude
  if (gps.location.isValid()) g.print(gps.location.lng(), 8);
  else                        g.print("N/A");
  g.print(",");

  // Altitude
  if (gps.altitude.isValid()) g.print(gps.altitude.meters(), 2);
  else                        g.print("N/A");
  g.print(",");

  // Satellites
  if (gps.satellites.isValid()) g.print(gps.satellites.value());
  else                          g.print("N/A");
  g.print(",");

  // HDOP
  if (gps.hdop.isValid()) g.print(gps.hdop.hdop(), 2);
  else                    g.print("N/A");
  g.print(",");

  // Speed
  if (gps.speed.isValid()) g.print(gps.speed.kmph(), 2);
  else                     g.print("N/A");
  g.print(",");

  // Date
  if (gps.date.isValid()) {
    g.print(gps.date.day());
    g.print("/");
    g.print(gps.date.month());
    g.print("/");
    g.print(gps.date.year());
  } else {
    g.print("N/A");
  }
  g.print(",");

  // Time
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) g.print("0");
    g.print(gps.time.hour());
    g.print(":");
    if (gps.time.minute() < 10) g.print("0");
    g.print(gps.time.minute());
    g.print(":");
    if (gps.time.second() < 10) g.print("0");
    g.print(gps.time.second());
  } else {
    g.print("N/A");
  }

  g.println();
  g.close();
}

// ---------------------- TELEMETRY PACKET TO XBEE ----------------------
void sendTelemetryPacket(
  const String &dateStr,
  const String &rtcTimeStr,
  const String &missionTimeStr,
  float bmp_pressure_hPa,
  float bmp_tempC,
  float dps_pressure_hPa,
  float dps_tempC,
  float dps_alt_m,
  float bmp_alt_m,
  double lat,
  double lng,
  float gpsAlt_m,
  int   sats,
  float accel_x,
  float accel_y,
  float accel_z,
  float orient_x,
  float orient_y,
  float orient_z,
  float busVoltage_V,
  float current_mA,
  float batteryPct,
  int   ozoneRaw,
  int   statusField,
  float teensyVinEst
) {
  String packet = "{";

  packet += "062,";                        // TEAM_ID
  packet += String(packetCount) + ",";     // PACKET_COUNT
  packet += dateStr + ",";                 // DATE
  packet += rtcTimeStr + ",";              // RTC TIME
  packet += missionTimeStr + ",";          // MISSION TIME

  packet += String(bmp_pressure_hPa, 2) + ",";
  packet += String(bmp_tempC, 2) + ",";
  packet += String(dps_pressure_hPa, 2) + ",";
  packet += String(dps_tempC, 2) + ",";
  packet += String(dps_alt_m, 2) + ",";
  packet += String(bmp_alt_m, 2) + ",";

  packet += String(lat, 8) + ",";
  packet += String(lng, 8) + ",";
  packet += String(gpsAlt_m, 2) + ",";
  packet += String(sats) + ",";

  packet += String(accel_x, 2) + ",";
  packet += String(accel_y, 2) + ",";
  packet += String(accel_z, 2) + ",";

  packet += String(orient_x, 2) + ",";
  packet += String(orient_y, 2) + ",";
  packet += String(orient_z, 2) + ",";

  packet += String(busVoltage_V, 2) + ",";
  packet += String(current_mA, 2) + ",";
  packet += String(batteryPct, 1) + ",";
  packet += String(ozoneRaw) + ",";
  packet += String(statusField) + ",";
  packet += String(teensyVinEst, 2);

  packet += "}";

  xbeeSerial.println(packet);

  Serial.print("Sent: ");
  Serial.println(packet);
}

// ---------------------- POWER MODEL ----------------------
float simulateBusVoltage() {
  // 11.14 V for first 20 min (1200s), then drop 0.01V/sec down to min 9.0 V
  unsigned long elapsedMs = millis() - missionStartMillis;
  float elapsedSec = elapsedMs / 1000.0f;

  const float baseV = 11.14f;
  if (elapsedSec <= 1200.0f) {
    return baseV;
  } else {
    float extraSec = elapsedSec - 1200.0f;
    float drop = 0.01f * extraSec;
    float v = baseV - drop;
    if (v < 9.0f) v = 9.0f;
    return v;
  }
}

float simulateCurrentmA() {
  return 220.0f;
}

float computeBatteryPercent(float vbat) {
  // 3S LiPo-ish: 12.6V full, 9.0V empty
  float pct = (vbat - 9.0f) * (100.0f / (12.6f - 9.0f));
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

float computeTeensyVinEst(float vbat) {
  return vbat * 1.05f;
}

// ---------------------- TIME HELPERS ----------------------
String formatMissionTime() {
  unsigned long elapsed = millis() - missionStartMillis;
  unsigned long totalSec = elapsed / 1000UL;
  unsigned int hh = totalSec / 3600UL;
  unsigned int mm = (totalSec % 3600UL) / 60UL;
  unsigned int ss = (totalSec % 60UL);

  char buf[12];
  snprintf(buf, sizeof(buf), "%02u:%02u:%02u", hh, mm, ss);
  return String(buf);
}

void getRealDateTime(String &dateStr, String &rtcTimeStr) {
  time_t t = now(); // Teensy RTC
  char dBuf[16];
  snprintf(dBuf, sizeof(dBuf), "%04d/%02d/%02d", year(t), month(t), day(t));
  dateStr = String(dBuf);

  char tBuf[16];
  snprintf(tBuf, sizeof(tBuf), "%02d:%02d:%02d", hour(t), minute(t), second(t));
  rtcTimeStr = String(tBuf);
}

// ---------------------- RTC ----------------------
void setRTCFromCompile() {
  setTime(compileTime());
  if (timeStatus() != timeSet) {
    Serial.println("Unable to set internal RTC time!");
  } else {
    Serial.println("Internal RTC set OK using compile time.");
  }
}




6







time_t compileTime() {
  String d = F(__DATE__); // e.g. "Oct 28 2025"
  String t = F(__TIME__); // e.g. "07:44:56"
  tmElements_t tm;
  tm.Year = CalendarYrToTm(atoi(d.substring(7).c_str()));

  switch (d[0]) {
    case 'J': tm.Month = (d[1] == 'a') ? 1 : ((d[2] == 'n') ? 6 : 7); break; // Jan, Jun, Jul
    case 'F': tm.Month = 2; break;
    case 'M': tm.Month = (d[2] == 'r') ? 3 : 5; break; // Mar, May
    case 'A': tm.Month = (d[1] == 'p') ? 4 : 8; break; // Apr, Aug
    case 'S': tm.Month = 9; break;
    case 'O': tm.Month = 10; break;
    case 'N': tm.Month = 11; break;
    case 'D': tm.Month = 12; break;
  }

  tm.Day    = atoi(d.substring(4).c_str());
  tm.Hour   = atoi(t.substring(0).c_str());
  tm.Minute = atoi(t.substring(3).c_str());
  tm.Second = atoi(t.substring(6).c_str());
  return makeTime(tm);
}
