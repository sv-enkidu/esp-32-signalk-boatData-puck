// Integrated version with Compass (0x2C), BME680, and BNO085 IMU on shared I2C
// v2.2 - Fixed redundant BME init error, added robust sensor tracking flags for modular assembly
// V2 code lacked robust initialization and included redundant bme.begin() call, throwing errors and a bit of garbled output
// v2.2 includes: boolean tracking flag flipping to true initialization succeeds, safe conditional acceptance to run if one or more
// sensors are not present without lagging or freezing, plus minor pin changes to accomodate new PCB layout
// See readme for data on dynamic, algorythmic on board IMU adjsutments on the BNO08x.  The board oututs the processed data which, 
// in the case of tilt-adjusted compass heading, is exceptionally hard to beat off board and on an ongoing basis.  Its a 
// remarkable benefit in a boating environment.

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_BME680.h> 
#include <Adafruit_BNO08x.h> 

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

// Helper function to convert BNO Quaternions to Euler Angles
void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { 
    Serial.println("$WARNING,Failed to enable BNO085 Rotation Vector report*00");
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200); // Allow physical bus lines to settle pulling high
  
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
    Serial.println("$NOTIFICATION,BNO085 IMU initialized successfully*00");
  } else {
    Serial.println("$WARNING,BNO085 IMU not found at 0x4A or 0x4B*00");
  }

  // 5. Final Bus Re-assertion verification
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
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float i = sensorValue.un.rotationVector.real;
        float j = sensorValue.un.rotationVector.i;
        float k = sensorValue.un.rotationVector.j;
        float l = sensorValue.un.rotationVector.k;
        
        float r = atan2(2.0 * (i * j + k * l), 1.0 - 2.0 * (j * j + k * k));
        float p = asin(2.0 * (i * k - l * j));
        float y = atan2(2.0 * (i * l + j * k), 1.0 - 2.0 * (k * k + l * l));

        currentIMU.roll = r * 180.0 / M_PI;
        currentIMU.pitch = p * 180.0 / M_PI;
        currentIMU.yaw = y * 180.0 / M_PI;
        if (currentIMU.yaw < 0) currentIMU.yaw += 360.0;
      }
    }
  }

  // 3. Transmit Compass ($HCHDG)
  if (hasCompass && (millis() - lastCompassTime >= compassInterval)) {
    lastCompassTime = millis();
    sendSafeCompassData();
  }

  // 4. Transmit IMU attitude ($IIXDR)
  if (hasBNO && (millis() - lastIMUTime >= imuInterval)) {
    lastIMUTime = millis();
    sendIMUAttitudeData();
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

void sendSafeCompassData() {
  float x = 0.0, y = 0.0, z = 0.0; 

  if (!compass.getGaussField(&x, &y, &z)) return; 
  if (x == 0.0 && y == 0.0) return; 

  float heading = atan2(y, x) * (180.0 / PI);
  if (heading < 0) heading += 360.0;

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
