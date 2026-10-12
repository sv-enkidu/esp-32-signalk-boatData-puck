/*
 * version_4v7 - Replaces 4.6, which is working code but lacks compass offset.   
 * 
 * offsetCompass (offsetCompass, degrees, NVS key "compass"), lost when
 *   the old QMC5883P compass offset was removed. This is distinct from offsetBnoYaw: offsetBnoYaw
 *   is a mechanical mounting correction applied pre-fusion-output (affects both $IIXDR YAW and
 *   $IIHDG). offsetCompass is a magnetic deviation correction applied ONLY in sendBNOHeadingData(),
 *   as the very last step before NMEA conversion, so it corrects $IIHDG only and never touches
 *   currentIMU.yaw, the BNO085's fusion math, or the $IIXDR attitude output.
 * 
 * 
 * sensing added via the BME280, output on a new $WIXDR humidity field.
 */

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h> 
#include <Adafruit_BNO08x.h> 
#include <Preferences.h> // ESP32 NVS Storage Library

// Pin assigments 
// Define GPS Pins (UART 2)
#define RXD2 16  
#define TXD2 17  

// Define BNO085 UART Pins (UART 1 - chosen safe GPIOs for ESP32-S3)
#define BNO_RX 4
#define BNO_TX 5

// Define Custom I2C Pins for Lonely Binary ESP32-S3 (BME280 only)
#define I2C_SDA 8
#define I2C_SCL 9

// Define BNO085 Control Pins
#define BNO_INT 11 
#define BNO_RST 10 

HardwareSerial gpsSerial(2);
HardwareSerial bnoSerial(1); // Dedicated UART for BNO085

Adafruit_BME280 bme; 
Adafruit_BNO08x bno08x(BNO_RST); // Pass reset pin to constructor
Preferences preferences;

// HARDCODED DEFAULT OFFSETS (in degrees - code will convert to radians and apply as such)
float offsetPitch  = 0.0; // BNO085 Pitch Offset
float offsetRoll   = 0.0; // BNO085 Roll Offset
float offsetBnoYaw = 0.0; // BNO085 Compass/Yaw Offset - mechanical mounting correction, applied
                            // pre-fusion-output (affects both $IIXDR YAW and $IIHDG equally)

// Magnetic compass correction (in degrees - NOT converted to radians, since it is applied
// directly to the already-computed heading in degrees, as the LAST step before NMEA output).
// Use this to correct for local magnetic deviation/anomaly (e.g. nearby metal, wiring, etc.)
// without touching the BNO085's own fused pitch/roll/yaw math or the $IIXDR YAW attitude value.
// Example: if the sensor reports magnetic north 10 degrees west of actual magnetic north,
// set this to +10.0 to correct it.

float offsetCompass = 0.0; // Magnetic Compass Offset (degrees, heading output only)

// Hardware Presence Tracking Flags
bool hasBME     = false;
bool hasBNO     = false;

// Struct to store parsed IMU values securely between reports
struct IMUData {
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
} currentIMU;

// Timing variables
unsigned long lastIMUTime = 0;
const unsigned long imuInterval = 100; // 10 Hz

unsigned long lastWeatherTime = 0;
const unsigned long weatherInterval = 2000; // 0.5 Hz

// ===================== WATCHGUARD (DISABLED) =====================
// The BNO085 firmware watchdog is disabled for now so the UART-connected
// sensor's behavior can be observed without any automatic reset intervening.
// It was originally added to recover from I2C clock-stretching hangs; it may
// not be needed now that the BNO085 is on UART.
//
// TO RE-ENABLE: uncomment every block tagged "WATCHGUARD" throughout this
// file (variables, recoverBNO085(), the setup() kicks, the loop() assessment
// and kick, and the isResettingBNO guard on the IMU send block).
//
// unsigned long lastSuccessfulDataTime = 0;
// const unsigned long bnoWatchdogTimeout = 3000; // Reset if dead for 3 seconds
// bool isResettingBNO = false;
// ===================================================================

void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { 
    Serial.println("$WARNING,Failed to enable BNO085 Rotation Vector report*00");
  }
}

// ===================== WATCHGUARD (DISABLED) =====================
// void recoverBNO085() {
//   isResettingBNO = true;
//   hasBNO = false;
//   Serial.println("$WARNING,BNO085 Freeze detected. Executing UART hardware reset pipeline...*00");
//
//   // 1. Force physical hardware reset line low to reset the sensor
//   pinMode(BNO_RST, OUTPUT);
//   digitalWrite(BNO_RST, LOW);
//   delay(50); 
//   digitalWrite(BNO_RST, HIGH);
//   pinMode(BNO_RST, INPUT); // Return to high-impedance
//   delay(1000); // Pause for reboot
//
//   // 2. Re-initialize UART serial port for BNO085
//   bnoSerial.begin(3000000, SERIAL_8N1, BNO_RX, BNO_TX); // Standard BNO085 UART baud rate is often high (3M) or configurable; check your sensor configuration. Default Adafruit UART uses 3000000 or 115200 depending on PS0/PS1 jumpers.
//   delay(150);
//
//   // 3. Re-attempt UART handshake
//   if (bno08x.begin_UART(&bnoSerial)) {
//     setReports();
//     hasBNO = true;
//     lastSuccessfulDataTime = millis(); 
//     Serial.println("$NOTIFICATION,BNO085 UART Watchdog recovery completed successfully*00");
//   } else {
//     Serial.println("$WARNING,BNO085 Hard reset failed to bind to UART bus. Will retry...*00");
//     lastSuccessfulDataTime = millis(); 
//   }
//   isResettingBNO = false;
// }
// ===================================================================

void initNvsOffsets() {
  preferences.begin("offsets", false);

  preferences.putFloat("pitch",   offsetPitch);
  preferences.putFloat("roll",    offsetRoll);
  preferences.putFloat("bnoyaw",  offsetBnoYaw);
  preferences.putFloat("compass", offsetCompass);

  offsetPitch   = preferences.getFloat("pitch",   0.0);
  offsetRoll    = preferences.getFloat("roll",    0.0);
  offsetBnoYaw  = preferences.getFloat("bnoyaw",  0.0);
  offsetCompass = preferences.getFloat("compass", 0.0);

  offsetPitch  = offsetPitch  * (M_PI / 180.0);
  offsetRoll   = offsetRoll   * (M_PI / 180.0);
  offsetBnoYaw = offsetBnoYaw * (M_PI / 180.0);
  // offsetCompass is intentionally left in degrees (not radians) - see declaration comment above.

  preferences.end();
  Serial.println("$NOTIFICATION,NVS Sensor offsets linked and synchronized*00");
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins (For BME280)
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200);

  // 2. Initialize BME280 (0x77 or 0x76) // CONFIRMED ADDRESS IS 0x76
  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    hasBME = true;
    // Adafruit_BME280 uses a single setSampling() call rather than the
    // BME680 library's separate setXOversampling()/setIIRFilterSize() calls.
    // Oversampling values chosen to mirror the previous BME680 settings
    // (temp 8X, pressure 4X); humidity is new. Filter set to X4, the
    // closest available step to the old FILTER_SIZE_3 setting.
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                     Adafruit_BME280::SAMPLING_X8,   // temperature
                     Adafruit_BME280::SAMPLING_X4,   // pressure
                     Adafruit_BME280::SAMPLING_X2,   // humidity
                     Adafruit_BME280::FILTER_X4,
                     Adafruit_BME280::STANDBY_MS_0_5);
    Serial.println("$NOTIFICATION,BME280 Weather Sensor initialized successfully*00");
  } else {
    Serial.println("$WARNING,BME280 Weather Sensor not found at 0x77 or 0x76*00");
  }

  // 3. Initialize BNO085 IMU over UART
  // Note: Ensure your BNO085 physical jumpers (PS0/PS1) are configured for UART mode.
  bnoSerial.begin(3000000, SERIAL_8N1, BNO_RX, BNO_TX); 
  delay(100);
  
  if (bno08x.begin_UART(&bnoSerial)) {
    hasBNO = true;
    setReports(); 
    // lastSuccessfulDataTime = millis(); // WATCHGUARD (DISABLED)
    Serial.println("$NOTIFICATION,BNO085 IMU initialized successfully via UART*00");
  } else {
    Serial.println("$WARNING,BNO085 IMU not found on UART port*00");
    // lastSuccessfulDataTime = millis(); // WATCHGUARD (DISABLED)
  }

  // 4. Initialize NVS offsets
  initNvsOffsets();  
}

void loop() {
  // 1. Process GPS Streams (Always Active)
  static String gpsBuffer = "";
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsBuffer += c;
    if (c == '\n') { 
      Serial.print(gpsBuffer); 
      gpsBuffer = ""; 
    }
  }

// 2. Poll BNO085 buffer ONLY if it exists physically
if (hasBNO) {
  if (bno08x.wasReset()) {
    Serial.println("$NOTIFICATION,BNO085 reset detected, re-enabling reports*00");
    setReports();
  }

  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float i = sensorValue.un.rotationVector.real;
      float j = sensorValue.un.rotationVector.i;
      float k = sensorValue.un.rotationVector.j;
      float l = sensorValue.un.rotationVector.k;
      
      float r = atan2(2.0 * (i * j + k * l), 1.0 - 2.0 * (j * j + k * k));
      float p = asin(2.0 * (i * k - l * j));
      float y = atan2(2.0 * (i * l + j * k), 1.0 - 2.0 * (k * k + l * l));

      p += offsetPitch;
      r += offsetRoll;
      y += offsetBnoYaw;

      currentIMU.roll  = r * 180.0 / M_PI;
      currentIMU.pitch = p * 180.0 / M_PI;
      currentIMU.yaw   = y * 180.0 / M_PI;
      
      while (currentIMU.yaw < 0.0)   currentIMU.yaw += 360.0;
      while (currentIMU.yaw >= 360.0) currentIMU.yaw -= 360.0;

      // lastSuccessfulDataTime = millis(); // WATCHGUARD (DISABLED) - Kick the watchdog
    }
  }
}

  // 3. Firmware Watchdog Assessment
  // ===================== WATCHGUARD (DISABLED) =====================
  // if (millis() - lastSuccessfulDataTime >= bnoWatchdogTimeout) {
  //   recoverBNO085();
  // }
  // ===================================================================

  // 4. Transmit IMU attitude ($IIXDR) and BNO085 heading ($IIHDG)
  // WATCHGUARD (DISABLED): original condition also required "!isResettingBNO" here.
  // If the watchguard is re-enabled, restore it: hasBNO && !isResettingBNO && (...)
  if (hasBNO && (millis() - lastIMUTime >= imuInterval)) {
    lastIMUTime = millis();
    sendIMUAttitudeData();
    sendBNOHeadingData();
  }

  // 5. Transmit weather data ($WIXDR)
  if (hasBME && (millis() - lastWeatherTime >= weatherInterval)) {
    lastWeatherTime = millis();
    sendWeatherData();
  }
} 

void sendIMUAttitudeData() {
  String attitudeBody = "IIXDR,"
                        "A," + String(currentIMU.pitch, 1) + ",D,PITCH,"
                        "A," + String(currentIMU.roll, 1) + ",D,ROLL,"
                        "A," + String(currentIMU.yaw, 1) + ",D,YAW";
  outputNmeasentence(attitudeBody);
}

void sendBNOHeadingData() {
  // Magnetic compass correction applied here, and ONLY here - as the last step before
  // NMEA conversion. currentIMU.yaw itself is left untouched, so this does not feed back
  // into the BNO085's fusion math and does not affect the $IIXDR YAW attitude value above.
  float correctedHeading = currentIMU.yaw + offsetCompass;
  while (correctedHeading < 0.0)    correctedHeading += 360.0;
  while (correctedHeading >= 360.0) correctedHeading -= 360.0;

  String nmeaBody = "IIHDG," + String(correctedHeading, 1) + ",,,,";
  outputNmeasentence(nmeaBody);
}

void sendWeatherData() {
  // Unlike the BME680 library's single performReading() + cached-member
  // pattern, Adafruit_BME280 reads each value independently over I2C.
  float tempC = bme.readTemperature();
  float pressurePa = bme.readPressure();
  float humidityPct = bme.readHumidity();

  // Adafruit_BME280 returns NAN on a failed read rather than a bool status,
  // so validate all three before transmitting.
  if (isnan(tempC) || isnan(pressurePa) || isnan(humidityPct)) return;

  float pressureBar = pressurePa / 100000.0;

  String nmeaBody = "WIXDR,"
                    "P," + String(pressureBar, 5) + ",B,BARO,"
                    "C," + String(tempC, 1) + ",C,TEMP,"
                    "H," + String(humidityPct, 1) + ",P,HUMID";

  outputNmeasentence(nmeaBody);
}

void outputNmeasentence(String body) {
  byte checksum = 0;
  for (int i = 0; i < body.length(); i++) {
    checksum ^= body[i];
  }
  
  String hexChecksum = String(checksum, HEX);
  hexChecksum.toUpperCase();
  if (hexChecksum.length() < 2) {
    hexChecksum = "0" + hexChecksum; 
  }
  
  Serial.print("$");
  Serial.print(body);
  Serial.print("*");
  Serial.println(hexChecksum);
}
