// Integrated version with Compass (0x2C), BME680, and BNO085 IMU on shared I2C
// v2.1 fixes bug that was sending garbled GPS messages, nests Yaw cleanly, and eliminates duplicates
// Be sure to release port from IDE when connecting to signalK, or data will not be received 

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_BME680.h> 
#include <Adafruit_BNO08x.h> // Include BNO08x Library

// Define GPS Pins
#define RXD2 17
#define TXD2 16

// Define Custom I2C Pins for Lonely Binary ESP32-S3
#define I2C_SDA 8
#define I2C_SCL 9

// Define BNO085 Control Pins
#define BNO_INT 10
#define BNO_RST 11

HardwareSerial gpsSerial(2);
Adafruit_QMC5883P compass;
Adafruit_BME680 bme; 
Adafruit_BNO08x bno08x(BNO_RST); // Pass reset pin to constructor

// Struct to store parsed IMU values securely between reports
struct IMUData {
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;
} currentIMU;

// Timing variables
unsigned long lastCompassTime = 0;
const unsigned long compassInterval = 100; // 10 Hz transmission (100ms)

unsigned long lastIMUTime = 0;
const unsigned long imuInterval = 100; // 10 Hz transmission (100ms)

unsigned long lastWeatherTime = 0;
const unsigned long weatherInterval = 2000; // 0.5 Hz transmission (Every 2 seconds)

// Helper function to convert BNO Quaternions to Euler Angles
void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { // 10,000 microseconds = 100Hz max internal sampling
    Serial.println("$WARNING,Failed to enable BNO085 Rotation Vector report*00");
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100); // Give the physical bus a moment to stabilize
  
  // 2. Initialize the Compass at verified 0x2C architecture address
  if (!compass.begin(0x2C, &Wire)) {
    Serial.println("$WARNING,Adafruit QMC5883P initialization failed at 0x2C*00");
  }

  // 3. Initialize the BME680 
  if (!bme.begin(0x77, &Wire)) {
    if (!bme.begin(0x76, &Wire)) {
      Serial.println("$WARNING,BME680 initialization failed at 0x77 and 0x76*00");
    }
  }

  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
  }

  // 4. Initialize the BNO085 IMU (Default I2C address is usually 0x4A or 0x4B)
  if (!bno08x.begin_I2C(0x4A, &Wire, BNO_INT)) {
    if (!bno08x.begin_I2C(0x4B, &Wire, BNO_INT)) {
      Serial.println("$WARNING,BNO085 IMU initialization failed at 0x4A and 0x4B*00");
    }
  }
  
  if (bno08x.wasReset()) {
    setReports();
  }

  // 5. EXPLICIT COMPASS OVERRIDES: Force configuration parameters directly
  compass.setRange(QMC5883P_RANGE_30G); 
  compass.setODR(QMC5883P_ODR_10HZ);   
  compass.setMode(QMC5883P_MODE_CONTINUOUS); 

  // 6. RE-ASSERT CUSTOM ESP32 PINS: Protect against I2C bus resets
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  // 1. FIXED: Buffer GPS characters into clean, complete sentences before printing
  static String gpsBuffer = "";
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsBuffer += c;
    if (c == '\n') { // Only print when a full sentence is complete
      Serial.print(gpsBuffer); 
      gpsBuffer = ""; // Clear buffer for next sentence
    }
  }

  // 2. Continually poll the BNO085 buffer to keep memory clean
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      // Extract Quaternion components
      float i = sensorValue.un.rotationVector.real;
      float j = sensorValue.un.rotationVector.i;
      float k = sensorValue.un.rotationVector.j;
      float l = sensorValue.un.rotationVector.k;
      
      // Convert Quaternion directly to Euler Angles (Roll, Pitch, Yaw) in Radians
      float r = atan2(2.0 * (i * j + k * l), 1.0 - 2.0 * (j * j + k * k));
      float p = asin(2.0 * (i * k - l * j));
      float y = atan2(2.0 * (i * l + j * k), 1.0 - 2.0 * (k * k + l * l));

      // Remap Radian metrics into standard degrees for NMEA execution
      currentIMU.roll = r * 180.0 / M_PI;
      currentIMU.pitch = p * 180.0 / M_PI;
      currentIMU.yaw = y * 180.0 / M_PI;
      if (currentIMU.yaw < 0) currentIMU.yaw += 360.0;
    }
  }

  // 3. Transmit standalone QMC5883P compass clock ($HCHDG)
  if (millis() - lastCompassTime >= compassInterval) {
    lastCompassTime = millis();
    sendSafeCompassData();
  }

  // 4. Transmit IMU attitude clock ($IIXDR bundled with Pitch, Roll, AND Yaw)
  if (millis() - lastIMUTime >= imuInterval) {
    lastIMUTime = millis();
    sendIMUAttitudeData();
  }

  // 5. Transmit weather data clock ($WIXDR)
  if (millis() - lastWeatherTime >= weatherInterval) {
    lastWeatherTime = millis();
    sendWeatherData();
  }
} // FIXED: Closed the main loop bracket

void sendIMUAttitudeData() {
  // FIXED & ADDED YAW: Bundled cleanly into a single XDR sentence to protect primary heading
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
