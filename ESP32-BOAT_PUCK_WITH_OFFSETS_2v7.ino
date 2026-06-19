/*
 * VERSION - Modifies 2v6,  which functions perfectly except that the offsets, because they are in NMEA0183
 * which requires human readable data in DEGREES (vs radians), were not user friendly (user would have to 
 * manually calculate the offset values for radians when the output in signalk - ie., the path values
 * in the data browser, were in degrees.  Version adds simple conversion math so that user now enters
 * offset in degrees.  
 * 
 * - Keeps sensor data processing in its native radians
 * - allows user engagement to align with data as surfaced by SK (NMEA plugin converts the degrees to radians, and
 *  vice versa)
 * 
 * flip this logic: the sender writes to NVS, and the firmware only reads — never writes from code. That's the 
 * right model for user-defined offsets that should survive reflashing.  That is, THE SENDER CHANNEL ONLY
 * WRITES TO NVS, the code does not apply values not sent by the sender channel (ENSURE sender channel 
 * CODE SENDS ZERO VALUES for any field that does not include explicitly defines offsets by user), eg:
 * 
 * // Phase 2 version — firmware never writes, only reads
* void initNvsOffsets() {
* preferences.begin("offsets", true);  // true = read-only
*  offsetPitch  = preferences.getFloat("pitch",  0.0);
* offsetRoll   = preferences.getFloat("roll",   0.0);
* offsetBnoYaw = preferences.getFloat("bnoyaw", 0.0);
* offsetQmcYaw = preferences.getFloat("qmcyaw", 0.0);
* preferences.end();
* }
 * 
 *
 * 
 * Phases:
 * 1. (THIS PHASE) Hardcoded (in firmware) offsets saved in NVS by firmware.  
 * 2. Sender channel created for user-defined, off-firmware offsets
 * 3. Java (signalk webapp) and/or node red UI for user defined setting of offsets
 * 
 *  
 * Things to look out for:
 * 1. The Catch: In SignalK and NMEA0183, a heading sentence (HDG) can carry both magnetic heading and 
 * variation/deviation fields. By hardcoding a raw offset, you are essentially baking mounting error directly 
 * into the heading.
 * 
 * 2. Current code On boot, it checks if (!preferences.isKey(...)), meaning it only writes once on the very 
 * first boot, and thereafter only reads. This has zero risk of wearing out the flash.  BUT in future sender
 * channel versions make sure node red or java sends only a new JSON packet when the user explicitly clicks a 
 * "Save Calibration" button. If you accidentally program Node-RED to stream slider values in real-time to the 
 * ESP32 while a user is dragging a UI slider, you could write to the flash thousands of times in a couple of 
 * minutes and degrade the memory (so as not to exceed NVS limits on ESP32)
 * 
 * 3. Compass adjustments should work perfectly at a flat dock. but there may be issues due to X and Y 
 * compensated (by sensors) three or 9-axis algorythmic adjustments based on the offsetted values.  POSSIBLE
 * FIX: Since you have a brilliant BNO085 IMU sitting right next to it on the same I2C bus, the BNO085 already 
 * outputs a highly accurate, tilt-compensated rotation vector. For your secondary compass (QMC), if you 
 * notice its heading swinging wildly while the boat is rolling heavily, you may eventually want to use the 
 * BNO's roll/pitch variables to mathematically "tilt-compensate" the raw QMC $X/Y$ values before calculating 
 * the heading.
 * 
 */

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_BME680.h> 
#include <Adafruit_BNO08x.h> 
#include <Preferences.h> // ESP32 NVS Storage Library

// Define GPS Pins
#define RXD2 16  
#define TXD2 17  

// Define Custom I2C Pins for Lonely Binary ESP32-S3
#define I2C_SDA 9
#define I2C_SCL 8

// Define BNO085 Control Pins
#define BNO_INT 10
#define BNO_RST 11

HardwareSerial gpsSerial(2);
Adafruit_QMC5883P compass;
Adafruit_BME680 bme; 
Adafruit_BNO08x bno08x(BNO_RST); 
Preferences preferences;

// HARDCODED DEFAULT OFFSETS (in degrees - code will convert to radians and apply as such)
// Modify these variables for your baseline hardware installation.
float offsetPitch  = 2.1; // BNO085 Pitch Offset
float offsetRoll   = 0.8; // BNO085 Roll Offset
float offsetBnoYaw = 0.0; // BNO085 Compass/Yaw Offset
float offsetQmcYaw = 0.0; // QMC5883P Compass Heading Offset

// Hardware Presence Tracking Flags
bool hasCompass = false;
bool hasBME     = false;
bool hasBNO     = false;

// Struct to store parsed IMU values securely between reports
struct IMUData {
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
} currentIMU;

// Timing variables
unsigned long lastCompassTime = 0;
const unsigned long compassInterval = 100; // 10 Hz

unsigned long lastIMUTime = 0;
const unsigned long imuInterval = 100; // 10 Hz

unsigned long lastWeatherTime = 0;
const unsigned long weatherInterval = 2000; // 0.5 Hz

// WATCHDOG VARIABLES FOR BNO085
unsigned long lastSuccessfulDataTime = 0;
const unsigned long bnoWatchdogTimeout = 3000; // Reset if dead for 3 seconds
bool isResettingBNO = false;

void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { 
    Serial.println("$WARNING,Failed to enable BNO085 Rotation Vector report*00");
  }
}

// Dedicated hardware reset routine for marine environment resilience
void recoverBNO085() {
  isResettingBNO = true;
  hasBNO = false;
  Serial.println("$WARNING,BNO085 Freeze detected. Executing hardware reset pipeline...*00");

  // Force physical hardware reset line low
  pinMode(BNO_RST, OUTPUT);
  digitalWrite(BNO_RST, LOW);
  delay(50); 
  digitalWrite(BNO_RST, HIGH);
  pinMode(BNO_RST, INPUT); // Return to high-impedance if required by breakout
  delay(200); // Give chip bootloader time to stabilize

  // Re-attempt I2C handshakes safely
  if (bno08x.begin_I2C(0x4A, &Wire, BNO_INT) || bno08x.begin_I2C(0x4B, &Wire, BNO_INT)) {
    setReports();
    hasBNO = true;
    lastSuccessfulDataTime = millis(); // Refresh watchdog clock
    Serial.println("$NOTIFICATION,BNO085 Watchdog recovery completed successfully*00");
  } else {
    Serial.println("$WARNING,BNO085 Hard reset failed to bind to I2C bus*00");
  }
  isResettingBNO = false;
}

void initNvsOffsets() {
  preferences.begin("offsets", false);

  // Always write hardcoded defaults — NVS only physically writes if value changed
  preferences.putFloat("pitch",  offsetPitch);
  preferences.putFloat("roll",   offsetRoll);
  preferences.putFloat("bnoyaw", offsetBnoYaw);
  preferences.putFloat("qmcyaw", offsetQmcYaw);

  // Then read back (confirms what's stored)
  offsetPitch  = preferences.getFloat("pitch",  0.0);
  offsetRoll   = preferences.getFloat("roll",   0.0);
  offsetBnoYaw = preferences.getFloat("bnoyaw", 0.0);
  offsetQmcYaw = preferences.getFloat("qmcyaw", 0.0);

  // ADDED (vs. 2v6) Convert degrees to radians for use in sensor math
  // (NMEA0183 uses degrees, but sensor quaternion math is in radians)
  offsetPitch  = offsetPitch  * (M_PI / 180.0);
  offsetRoll   = offsetRoll   * (M_PI / 180.0);
  offsetBnoYaw = offsetBnoYaw * (M_PI / 180.0);
  offsetQmcYaw = offsetQmcYaw * (M_PI / 180.0);

  preferences.end();
  Serial.println("$NOTIFICATION,NVS Sensor offsets linked and synchronized*00");
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200);
  
  // 2. Initialize Compass (0x2C)
  if (compass.begin(0x2C, &Wire)) {
    hasCompass = true;
    compass.setRange(QMC5883P_RANGE_30G); 
    compass.setODR(QMC5883P_ODR_10HZ);   
    compass.setMode(QMC5883P_MODE_CONTINUOUS); 
    Serial.println("$NOTIFICATION,QMC5883P Compass initialized successfully at 0x2C*00");
  } else {
    Serial.println("$WARNING,QMC5883P Compass not found at 0x2C*00");
  }

  // 3. Initialize BME680 (0x77 or 0x76)
  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    hasBME = true;
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
    Serial.println("$NOTIFICATION,BME680 Weather Sensor initialized successfully*00");
  } else {
    Serial.println("$WARNING,BME680 Weather Sensor not found at 0x77 or 0x76*00");
  }

  // 4. Initialize BNO085 IMU (0x4A or 0x4B)
  if (bno08x.begin_I2C(0x4A, &Wire, BNO_INT) || bno08x.begin_I2C(0x4B, &Wire, BNO_INT)) {
    hasBNO = true;
    if (bno08x.wasReset()) {
      setReports();
    }
    lastSuccessfulDataTime = millis();
    Serial.println("$NOTIFICATION,BNO085 IMU initialized successfully*00");
  } else {
    Serial.println("$WARNING,BNO085 IMU not found at 0x4A or 0x4B*00");
  }

  // 5. Initialize NVS offsets (after all I2C sensors are up)
  initNvsOffsets();  

  // 6. Final Bus Re-assertion verification
  Wire.begin(I2C_SDA, I2C_SCL);
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
    sh2_SensorValue_t sensorValue;
    // Non-blocking transaction check
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float i = sensorValue.un.rotationVector.real;
        float j = sensorValue.un.rotationVector.i;
        float k = sensorValue.un.rotationVector.j;
        float l = sensorValue.un.rotationVector.k;
        
        float r = atan2(2.0 * (i * j + k * l), 1.0 - 2.0 * (j * j + k * k));
        float p = asin(2.0 * (i * k - l * j));
        float y = atan2(2.0 * (i * l + j * k), 1.0 - 2.0 * (k * k + l * l));

        // Apply physical mounting offsets in Radians
        p += offsetPitch;
        r += offsetRoll;
        y += offsetBnoYaw;

        currentIMU.roll  = r * 180.0 / M_PI;
        currentIMU.pitch = p * 180.0 / M_PI;
        currentIMU.yaw   = y * 180.0 / M_PI;
        
        // Normalize degrees output to standard 0-360 range
        while (currentIMU.yaw < 0.0)   currentIMU.yaw += 360.0;
        while (currentIMU.yaw >= 360.0) currentIMU.yaw -= 360.0;

        // Kick the watchdog timer on explicit data acquisition
        lastSuccessfulDataTime = millis();
      }
    }
  }

  // 3. Firmware Watchdog Assessment
  if (hasBNO && (millis() - lastSuccessfulDataTime >= bnoWatchdogTimeout)) {
    recoverBNO085();
  }

  // 4. Transmit QMC5883P stand-alone compass data ($HCHDG)
  if (hasCompass && (millis() - lastCompassTime >= compassInterval)) {
    lastCompassTime = millis();
    sendSafeCompassData();
  }

  // 5. Transmit IMU attitude ($IIXDR) and BNO085 heading ($IIHDG)
  if (hasBNO && !isResettingBNO && (millis() - lastIMUTime >= imuInterval)) {
    lastIMUTime = millis();
    sendIMUAttitudeData();
    sendBNOHeadingData();
  }

  // 6. Transmit weather data ($WIXDR)
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
  String nmeaBody = "IIHDG," + String(currentIMU.yaw, 1) + ",,,,";
  outputNmeasentence(nmeaBody);
}

void sendSafeCompassData() {
  float x = 0.0, y = 0.0, z = 0.0; 

  if (!compass.getGaussField(&x, &y, &z)) return; 
  if (x == 0.0 && y == 0.0) return; 

  float heading_rad = atan2(y, x);
  
  // Apply raw radian offset alignment
  heading_rad += offsetQmcYaw;

  float heading = heading_rad * (180.0 / M_PI);
  
  // Normalize compass degrees output to 0-360 range
  while (heading < 0.0)   heading += 360.0;
  while (heading >= 360.0) heading -= 360.0;

  String nmeaBody = "HCHDG," + String(heading, 1) + ",,,,";
  outputNmeasentence(nmeaBody);
}

void sendWeatherData() {
  if (!bme.performReading()) return;

  float tempC = bme.temperature;
  float pressureBar = bme.pressure / 100000.0; 
  float vocKOhms = bme.gas_resistance / 1000.0; 

  String nmeaBody = "WIXDR,"
                    "P," + String(pressureBar, 5) + ",B,BARO,"
                    "C," + String(tempC, 1) + ",C,TEMP,"
                    "G," + String(vocKOhms, 1) + ",,VOC";

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
