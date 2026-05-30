// Integrated version with Compass (0x2C), BME680, and BNO085 IMU on shared I2C
// Converted to native Signal K JSON Delta Format

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_BME680.h> 
#include <Adafruit_BNO08x.h> 

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
Adafruit_BNO08x bno08x(BNO_RST); 

// Struct to store parsed IMU values securely in RADIANS for Signal K
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

void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) { 
    // Kept as an NMEA-like warning or simple string, Signal K will ignore it safely
    Serial.println("{\"updates\":[{\"source\":{\"label\":\"esp32-bridge\"},\"values\":[{\"path\":\"notifications.warning\",\"value\":\"BNO085 Report Failed\"}]}]}");
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(100); 
  
  // 2. Initialize the Compass
  if (!compass.begin(0x2C, &Wire)) {
    Serial.println("{\"updates\":[{\"source\":{\"label\":\"esp32-bridge\"},\"values\":[{\"path\":\"notifications.warning\",\"value\":\"QMC5883P Failed\"}]}]}");
  }

  // 3. Initialize the BME680 
  if (!bme.begin(0x77, &Wire)) {
    bme.begin(0x76, &Wire);
  }

  if (bme.begin(0x77, &Wire) || bme.begin(0x76, &Wire)) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); 
  }

  // 4. Initialize the BNO085 IMU
  if (!bno08x.begin_I2C(0x4A, &Wire, BNO_INT)) {
    bno08x.begin_I2C(0x4B, &Wire, BNO_INT);
  }
  
  if (bno08x.wasReset()) {
    setReports();
  }

  // 5. Compass overrides
  compass.setRange(QMC5883P_RANGE_30G); 
  compass.setODR(QMC5883P_ODR_10HZ);   
  compass.setMode(QMC5883P_MODE_CONTINUOUS); 

  // 6. Re-assert I2C pins
  Wire.begin(I2C_SDA, I2C_SCL);
}

void loop() {
  // 1. GPS Pass-Through (NMEA0183 strings passed cleanly, Signal K handles mixed streams)
  static String gpsBuffer = "";
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gpsBuffer += c;
    if (c == '\n') { 
      Serial.print(gpsBuffer); 
      gpsBuffer = ""; 
    }
  }

  // 2. Poll BNO085 buffer
  sh2_SensorValue_t sensorValue;
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float i = sensorValue.un.rotationVector.real;
      float j = sensorValue.un.rotationVector.i;
      float k = sensorValue.un.rotationVector.j;
      float l = sensorValue.un.rotationVector.k;
      
      // Kept in RADIANS for Signal K native compatibility
      currentIMU.roll = atan2(2.0 * (i * j + k * l), 1.0 - 2.0 * (j * j + k * k));
      currentIMU.pitch = asin(2.0 * (i * k - l * j));
      currentIMU.yaw = atan2(2.0 * (i * l + j * k), 1.0 - 2.0 * (k * k + l * l));
      if (currentIMU.yaw < 0) currentIMU.yaw += (2.0 * M_PI);
    }
  }

  // 3. Transmit standalone QMC5883P compass clock
  if (millis() - lastCompassTime >= compassInterval) {
    lastCompassTime = millis();
    sendSafeCompassData();
  }

  // 4. Transmit IMU attitude clock (Pitch, Roll, and IMU Yaw)
  if (millis() - lastIMUTime >= imuInterval) {
    lastIMUTime = millis();
    sendIMUAttitudeData();
  }

  // 5. Transmit weather data clock
  if (millis() - lastWeatherTime >= weatherInterval) {
    lastWeatherTime = millis();
    sendWeatherData();
  }
}

void sendIMUAttitudeData() {
  // Bundled attitude data using the "bno085-imu" source tag
  String json = "{\"updates\":[{\"source\":{\"label\":\"bno085-imu\"},\"values\":[";
  json += "{\"path\":\"navigation.attitude.roll\",\"value\":" + String(currentIMU.roll, 4) + "},";
  json += "{\"path\":\"navigation.attitude.pitch\",\"value\":" + String(currentIMU.pitch, 4) + "},";
  json += "{\"path\":\"navigation.headingMagnetic\",\"value\":" + String(currentIMU.yaw, 4) + "}";
  json += "]}]}";
  Serial.println(json);
}

void sendSafeCompassData() {
  float x = 0.0, y = 0.0, z = 0.0; 

  if (!compass.getGaussField(&x, &y, &z)) return; 
  if (x == 0.0 && y == 0.0) return; 

  // Heading calculated directly into RADIANS
  float headingRad = atan2(y, x);
  if (headingRad < 0) headingRad += (2.0 * M_PI);

  // Transmitted using the distinct "qmc5883-compass" source tag
  String json = "{\"updates\":[{\"source\":{\"label\":\"qmc5883-compass\"},\"values\":[";
  json += "{\"path\":\"navigation.headingMagnetic\",\"value\":" + String(headingRad, 4) + "}";
  json += "]}]}";
  Serial.println(json);
}

void sendWeatherData() {
  if (!bme.performReading()) return;

  // Conversion to SI units: Celsius to Kelvin, and Pascals to Pascals (Signal K standard)
  float tempK = bme.temperature + 273.15;
  float pressurePa = bme.pressure; 

  String json = "{\"updates\":[{\"source\":{\"label\":\"bme680-weather\"},\"values\":[";
  json += "{\"path\":\"environment.outside.temperature\",\"value\":" + String(tempK, 2) + "},";
  json += "{\"path\":\"environment.outside.pressure\",\"value\":" + String(pressurePa, 1) + "}";
  json += "]}]}";
  Serial.println(json);
}
