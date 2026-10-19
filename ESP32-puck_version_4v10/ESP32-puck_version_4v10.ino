/*
 * VERSION 4.10 (QMC compass diagnostic build)
 * - Adds QMC5883P hard-iron / soft-iron calibration (qmcHardX/Y/Z and
 *   qmcSoftX/Y/Z). Defaults are neutral (0 offset, 1.0 scale) so output is
 *   identical to 4.9 until values are entered.
 * - QMC_TILT_COMP: set to 0 to run the QMC as a plain 2-axis compass
 *   (no roll/pitch from the BNO) for testing.
 * - QMC_RAW_DEBUG: set to 1 to also print $PQMCR raw field values plus the
 *   running hard-iron estimate, for calibration. Turn off for normal use.
 * - QMC range changed from 30G to 8G (better resolution; Earth's field is
 *   only ~0.25-0.65 G).
 *
 * VERSION 4.9
 * - YAW now reports DEVIATION (degrees) from a held heading instead of
 *   duplicating the BNO heading. $IIHDG, roll, pitch, the QMC compass
 *   ($HCHDG), GPS passthrough and weather output are UNCHANGED.
 *     * Positive yaw = bow swung to starboard, negative = to port.
 *     * A heading is "held" (locked) once the BNO heading stays within
 *       +/- yawRange/2 of its own average for yawLockSec seconds. The lock
 *       reference is that average.
 *     * While locked, YAW = deviation of the live heading from the reference
 *       (no deadband - small swings are reported as-is).
 *     * The lock is released when the average heading moves more than
 *       yawRange/2 away from the reference for yawReleaseSec seconds
 *       (deliberate course change or wind shift), or immediately if any
 *       single swing exceeds YAW_HARD_LIMIT_DEG (tack, gybe, sharp turn).
 *     * While unlocked, the YAW field is sent EMPTY (no value), so "no
 *       reference" is never mistaken for "perfectly on course".
 *   New user settings: yawRange, yawLockSec, yawReleaseSec (stored with
 *   the other offsets).
 *   
 *   Simply put, leave the puck still and YAW will set itself to zero, and will
 *   remain at zero within either 10 degree default range, or when an offset is
 *   added that extends this range.  The YAW value reported will be any heading
 *   movement one way or the other outside of that range, in plus or minus degrees.
 *   
 *   Example of YAW in action
 *   - consider a course held by a windvane.  The boat bow will naturally point
 *   one way or the other within some range as the windvane kicks in to counteract
 *   natural yaw, until the heading reverts to that set on the vane. In this case, 
 *   YAW can be used to measure deviation FROM THE COURSE HELD BY THE VANE, which can 
 *   be useful data for voyage planning, alerting (sound alert when windvane struggles
 *   to hold course), boat trimming, autopilot enhancement, etc.
 *   
 *   TO DO: Dynamic IMU Heading Adjustment to real-world conditions:
 *   - Enhance yaw algo to include factors like forward motion,and letting 
 *   the puck size yawRange itself from the swing it measures, so it adapts as 
 *   conditions build or ease.
 *   - Create MQTT mechanism for user to read/send offsets from Node Red or signalk
 *   dashboard (versus hardcoding and flashing offsets at setup)
 *
 * VERSION 4.8 (last working pre-YAW changed version)
 * - Includes both QMC and GPS compasses
 * - UART for both GPS and BNO
 * - Pins change log between PCB's
 * - Sensors: BME280, BNO08x, QMC5883, M10 GPS
 * - Values sent to signalk: All default ublox GPS valies, temp, humidity, IMU heel/roll,
 * pitch, yaw, tilt-adjusted compass using both IMU and QMC compass (two compass outputs)
 * 
 * Note: GPS seems to want to connect at 38400 (although specs state 115200 default)
 * - If GPS does not connect, try using various baud rates (38400, 9600, 115200, etc)
 *  as OEM M10 GPS's can get quirky with baud rates.
 * 
 */

#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_QMC5883P.h>
#include <Adafruit_BME280.h> 
#include <Adafruit_BNO08x.h> 
#include <Preferences.h> // ESP32 NVS Storage Library

// Define GPS Pins (UART 2)
#define RXD2 13 // ON PCB3-18, PCB2-13
#define TXD2 12 // ON PCB3-17, PCB2-12 

// Define BNO085 UART Pins - may need to reverse
#define BNO_RX 2 // ON PCB3-2, PCB2-2            
#define BNO_TX 1 // ON PCB3-1, PCB2-1

// Define BNO085 Control Pins
#define BNO_INT 11 // ON PCB3-13, PCB2-11
#define BNO_RST 10 // ON PCB3-14, PCB2-10

// Define Custom I2C Pins for Lonely Binary ESP32-S3 (BME280 & QMC5883P only)
#define I2C_SDA 9 // ON PCB3-8, PCB2-9
#define I2C_SCL 8 // ON PCB3-9, PCB2-8

HardwareSerial gpsSerial(2);
HardwareSerial bnoSerial(1); // Dedicated UART for BNO085

Adafruit_QMC5883P compass;
Adafruit_BME280 bme; 
Adafruit_BNO08x bno08x(BNO_RST); // Pass reset pin to constructor
Preferences preferences;

// HARDCODED DEFAULT OFFSETS (in degrees - code will convert to radians and apply as such)
float offsetPitch  = 0.0; // BNO085 Pitch Offset
float offsetRoll   = 0.0; // BNO085 Roll Offset
float offsetBnoYaw = 0.0; // BNO085 Compass/Heading Offset (applies to $IIHDG)
float offsetQmcYaw = 0.0; // QMC5883P Compass Heading Offset (applies to $HCHDG)

// HARDCODED YAW DEVIATION SETTINGS (NOT converted to radians - used as-is)
// yawRange:      total width in degrees of the band the heading must stay
//                within to count as "held". 10 = within +/-5 deg of its
//                average. Also the drift limit used to release the lock.
//                Set it a little wider than the boat's normal swing in the
//                current conditions (e.g. swinging ~5 deg each side -> 10-12).
// yawLockSec:    seconds the heading must stay within the band to lock.
// yawReleaseSec: seconds the average heading must stay outside the band
//                before the lock is released (course change / wind shift).
float yawRange      = 10.0; // degrees (total band width) - default 10.0
float yawLockSec    = 5.0;  // seconds - default 5.0
float yawReleaseSec = 5.0;  // seconds - default 5.0

// Set to 1 to also send the locked reference heading as a second XDR value
// named YAWREF (empty while unlocked). Useful for logging; off by default so
// the attitude sentence keeps its existing three fields.
#define YAW_SEND_REF 1

// ===== QMC5883P DIAGNOSTIC / CALIBRATION SETTINGS =====
// 1 = tilt-compensate the QMC heading using BNO roll/pitch (normal)
// 0 = plain 2-axis heading from mx/my only (test: is tilt comp the cause?)
#define QMC_TILT_COMP 0

// 1 = also print "$PQMCR,mx,my,mz,hardX,hardY,hardZ" every compass cycle.
//     mx/my/mz = RAW field in Gauss (before calibration).
//     hardX/Y/Z = running hard-iron estimate from min/max seen since boot.
//     Rotate the puck slowly through a few full turns (and tilt it in all
//     directions if you can) and the hardX/Y/Z values will settle - those
//     are the numbers to enter below. Set back to 0 for normal use.
#define QMC_RAW_DEBUG 1

// Hard-iron offsets (Gauss) - subtracted from the raw field.
// Fixed magnetism on the PCB/enclosure (BNO board, ESP32, regulator, screws,
// battery, USB cable) shifts the field circle off-centre, which is what
// makes the heading speed up through one part of the rotation.
float qmcHardX = 0.0;
float qmcHardY = 0.0;
float qmcHardZ = 0.0;

// Soft-iron scale factors - multiplied after the hard-iron subtraction.
// Corrects a squashed (elliptical) circle. Leave at 1.0 unless the
// calibration shows the X and Y spans differ noticeably.  Note: the two sets   
// of values (hard vs. soft offsets) are independent. The code subtracts qmcHard from each axis,
// then multiplies by qmcSoft. Hard = 0 means nothing is subtracted, 
// and soft = 1.0 means multiplying by one. So the defaults change nothing,
// which is why your output matched v4.9. A soft value only does something when
// it isn’t 1.0, whatever the hard values are.
float qmcSoftX = 1.0;
float qmcSoftY = 1.0;
float qmcSoftZ = 1.0;

// Hardware Presence Tracking Flags
bool hasCompass = false;
bool hasBME     = false;
bool hasBNO     = false;

// Struct to store parsed IMU values securely between reports
struct IMUData {
  float roll = 0.0;
  float pitch = 0.0;
  float yaw = 0.0;   // BNO heading (0-360) - feeds $IIHDG and the yaw deviation logic
} currentIMU;

// Timing variables
unsigned long lastCompassTime = 0;
const unsigned long compassInterval = 100; // 10 Hz

unsigned long lastIMUTime = 0;
const unsigned long imuInterval = 100; // 10 Hz

unsigned long lastWeatherTime = 0;
const unsigned long weatherInterval = 2000; // 0.5 Hz

// ===== YAW DEVIATION LOGIC BEGIN =====
// Self-contained: uses only floats and math functions, called once per
// 10 Hz IMU send cycle with the current BNO heading.

#define YAW_SAMPLE_HZ       10      // yawUpdate() runs at the 10 Hz IMU send rate
#define YAW_MAX_WINDOW_SEC  60      // longest yawLockSec supported
#define YAW_BUF_SIZE        (YAW_SAMPLE_HZ * YAW_MAX_WINDOW_SEC)
#define YAW_HARD_LIMIT_DEG  45.0f   // a single swing beyond this ends the lock at once

// The BNO heading ($IIHDG) currently INCREASES when the puck turns
// counter-clockwise (BNO Z-axis convention), so a swing to starboard makes
// the heading value go DOWN. YAW_SIGN flips the result so that starboard is
// positive. If $IIHDG is ever corrected to compass convention (clockwise =
// increasing), change this to (+1.0f).
#define YAW_SIGN            (-1.0f)

float yawBuf[YAW_BUF_SIZE];   // ring buffer of recent headings (degrees)
int   yawBufHead      = 0;    // next write position
int   yawBufCount     = 0;    // samples currently held
bool  yawLocked       = false;
float yawHeading      = 0.0f; // locked reference heading (same units as $IIHDG)
float yawDev          = 0.0f; // deviation from reference (valid while locked)
int   yawReleaseTicks = 0;    // consecutive samples with average off-course

static float yawWrap180(float a) {
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

static float yawWrap360(float a) {
  while (a < 0.0f)     a += 360.0f;
  while (a >= 360.0f)  a -= 360.0f;
  return a;
}

// Number of samples in the averaging window (yawLockSec at 10 Hz)
static int yawWindowSamples() {
  int n = (int)(yawLockSec * YAW_SAMPLE_HZ + 0.5f);
  if (n < 2) n = 2;
  if (n > YAW_BUF_SIZE) n = YAW_BUF_SIZE;
  return n;
}

// Sample n-steps back from the newest (0 = newest)
static float yawSampleAgo(int k) {
  int idx = yawBufHead - 1 - k;
  while (idx < 0) idx += YAW_BUF_SIZE;
  return yawBuf[idx];
}

// Circular mean (0-360) of the newest n samples, so 359 and 1 average to 0
static float yawWindowMean(int n) {
  const float d2r = (float)M_PI / 180.0f;
  float s = 0.0f, c = 0.0f;
  for (int k = 0; k < n; k++) {
    float r = yawSampleAgo(k) * d2r;
    s += sinf(r);
    c += cosf(r);
  }
  return yawWrap360(atan2f(s, c) / d2r);
}

static void yawReset() {
  yawBufHead      = 0;
  yawBufCount     = 0;
  yawLocked       = false;
  yawReleaseTicks = 0;
}

// Call once per 10 Hz cycle with the current BNO heading (0-360)
void yawUpdate(float heading) {
  yawBuf[yawBufHead] = heading;
  yawBufHead = (yawBufHead + 1) % YAW_BUF_SIZE;
  if (yawBufCount < YAW_BUF_SIZE) yawBufCount++;

  float range = (yawRange > 0.0f) ? yawRange : 10.0f;
  float half  = range * 0.5f;
  int   n     = yawWindowSamples();

  // --- Acquire: lock once a full window sits within +/- half of its average
  if (!yawLocked) {
    if (yawBufCount < n) return;
    float mean = yawWindowMean(n);
    for (int k = 0; k < n; k++) {
      if (fabsf(yawWrap180(yawSampleAgo(k) - mean)) > half) return;
    }
    yawHeading      = mean;
    yawLocked       = true;
    yawReleaseTicks = 0;
  }

  // --- Report: deviation of live heading from the locked reference
  yawDev = YAW_SIGN * yawWrap180(heading - yawHeading);

  // --- Release (immediate): one very large swing = tack, gybe or sharp turn
  if (fabsf(yawDev) > YAW_HARD_LIMIT_DEG) {
    yawReset();
    return;
  }

  // --- Release (sustained): the AVERAGE heading has left the band. Normal
  // swings average out and keep the lock; course changes and wind shifts don't.
  float mean = yawWindowMean(n);
  if (fabsf(yawWrap180(mean - yawHeading)) > half) {
    int releaseTicks = (int)(yawReleaseSec * YAW_SAMPLE_HZ + 0.5f);
    if (releaseTicks < 1) releaseTicks = 1;
    if (++yawReleaseTicks >= releaseTicks) {
      yawReset();
      return;
    }
  } else {
    yawReleaseTicks = 0;
  }
}
// ===== YAW DEVIATION LOGIC END =====

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
//   delay(100); // Pause for reboot
//
//   // 2. Re-initialize UART serial port for BNO085
//   bnoSerial.begin(3000000, SERIAL_8N1, BNO_RX, BNO_TX); // Standard BNO085 UART baud rate is 
//   often high (3M) or configurable; check your sensor configuration. Default Adafruit UART uses 3000000 or 115200 depending on PS0/PS1 jumpers.
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

  preferences.putFloat("pitch",  offsetPitch);
  preferences.putFloat("roll",   offsetRoll);
  preferences.putFloat("bnoyaw", offsetBnoYaw);
  preferences.putFloat("qmcyaw", offsetQmcYaw);
  preferences.putFloat("yawrange", yawRange);
  preferences.putFloat("yawlock",  yawLockSec);
  preferences.putFloat("yawrel",   yawReleaseSec);

  offsetPitch  = preferences.getFloat("pitch",  0.0);
  offsetRoll   = preferences.getFloat("roll",   0.0);
  offsetBnoYaw = preferences.getFloat("bnoyaw", 0.0);
  offsetQmcYaw = preferences.getFloat("qmcyaw", 0.0);
  yawRange      = preferences.getFloat("yawrange", 10.0);
  yawLockSec    = preferences.getFloat("yawlock",  5.0);
  yawReleaseSec = preferences.getFloat("yawrel",   5.0);

  offsetPitch  = offsetPitch  * (M_PI / 180.0);
  offsetRoll   = offsetRoll   * (M_PI / 180.0);
  offsetBnoYaw = offsetBnoYaw * (M_PI / 180.0);
  offsetQmcYaw = offsetQmcYaw * (M_PI / 180.0);
  // yawRange / yawLockSec / yawReleaseSec are degrees and seconds - no conversion

  preferences.end();
  Serial.println("$NOTIFICATION,NVS Sensor offsets linked and synchronized*00");
}

void setup() {
  Serial.begin(115200);

  // Wait for USB host (native-USB ESP32-S3), max 3 s. On boards with a
  // USB-UART bridge chip (allow time for diagnostic messages on startup
  // to apear on the serial monitor. Serial is always "true" so this costs ~0 ms.
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000)) {
    delay(10);
  }
  delay(250); // short settle so the monitor is ready to display text
  
  gpsSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);

  // 1. Initialize custom ESP32-S3 I2C hardware matrix pins (For BME280 and QMC5883P)
  Wire.begin(I2C_SDA, I2C_SCL);
  delay(200);
  
  // 2. Initialize Compass (0x2C)
  if (compass.begin(0x2C, &Wire)) {
    hasCompass = true;
    compass.setRange(QMC5883P_RANGE_8G);  // was 30G - 8G gives ~4x finer resolution
    compass.setODR(QMC5883P_ODR_10HZ);    
    compass.setMode(QMC5883P_MODE_CONTINUOUS); 
    Serial.println("$NOTIFICATION,QMC5883P Compass initialized successfully at 0x2C*00");
  } else {
    Serial.println("$WARNING,QMC5883P Compass not found at 0x2C*00");
  }

  // 3. Initialize BME280 (0x77 or 0x76)
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

  // 4. Initialize BNO085 IMU over UART
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

  // 5. Initialize NVS offsets
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

  // 4. Transmit QMC5883P stand-alone compass data ($HCHDG)
  if (hasCompass && (millis() - lastCompassTime >= compassInterval)) {
    lastCompassTime = millis();
    sendSafeCompassData();
  }

  // 5. Transmit IMU attitude ($IIXDR) and BNO085 heading ($IIHDG)
  // WATCHGUARD (DISABLED): original condition also required "!isResettingBNO" here.
  // If the watchguard is re-enabled, restore it: hasBNO && !isResettingBNO && (...)
  if (hasBNO && (millis() - lastIMUTime >= imuInterval)) {
    lastIMUTime = millis();
    yawUpdate(currentIMU.yaw);   // update yaw deviation from the current BNO heading
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
  // YAW = deviation from the held heading (+ starboard / - port).
  // Sent as an empty field while no heading is held.
  String yawField = yawLocked ? String(yawDev, 1) : String("");

  String attitudeBody = "IIXDR,"
                        "A," + String(currentIMU.pitch, 1) + ",D,PITCH,"
                        "A," + String(currentIMU.roll, 1) + ",D,ROLL,"
                        "A," + yawField + ",D,YAW";
#if YAW_SEND_REF
  String refField = yawLocked ? String(yawHeading, 1) : String("");
  attitudeBody += ",A," + refField + ",D,YAWREF";
#endif
  outputNmeasentence(attitudeBody);
}

void sendBNOHeadingData() {
  String nmeaBody = "IIHDG," + String(currentIMU.yaw, 1) + ",,,,";
  outputNmeasentence(nmeaBody);
}

void sendSafeCompassData() {
  float mx = 0.0, my = 0.0, mz = 0.0;

  if (!compass.getGaussField(&mx, &my, &mz)) return;
  if (mx == 0.0 && my == 0.0 && mz == 0.0) return;

#if QMC_RAW_DEBUG
  // Track min/max of the RAW field since boot -> hard-iron estimate
  static float minX =  1e9, maxX = -1e9;
  static float minY =  1e9, maxY = -1e9;
  static float minZ =  1e9, maxZ = -1e9;
  if (mx < minX) minX = mx;  if (mx > maxX) maxX = mx;
  if (my < minY) minY = my;  if (my > maxY) maxY = my;
  if (mz < minZ) minZ = mz;  if (mz > maxZ) maxZ = mz;

  String dbg = "PQMCR,"
               + String(mx, 4) + "," + String(my, 4) + "," + String(mz, 4) + ","
               + String((maxX + minX) * 0.5f, 4) + ","
               + String((maxY + minY) * 0.5f, 4) + ","
               + String((maxZ + minZ) * 0.5f, 4);
  outputNmeasentence(dbg);
#endif

  // Apply hard-iron (offset) then soft-iron (scale) calibration
  mx = (mx - qmcHardX) * qmcSoftX;
  my = (my - qmcHardY) * qmcSoftY;
  mz = (mz - qmcHardZ) * qmcSoftZ;

#if QMC_TILT_COMP
  float rollRad  = currentIMU.roll  * (M_PI / 180.0);
  float pitchRad = currentIMU.pitch * (M_PI / 180.0);

  float cosRoll  = cos(rollRad);
  float sinRoll  = sin(rollRad);
  float cosPitch = cos(pitchRad);
  float sinPitch = sin(pitchRad);

  float mxH = mx * cosPitch
             + my * sinRoll * sinPitch
             - mz * cosRoll * sinPitch;

  float myH = my * cosRoll
             + mz * sinRoll;
#else
  // Plain 2-axis compass: ignore roll/pitch and Z entirely
  float mxH = mx;
  float myH = my;
#endif

  float heading_rad = atan2(myH, mxH);

  heading_rad += offsetQmcYaw;

  float heading = heading_rad * (180.0 / M_PI);
  while (heading < 0.0)    heading += 360.0;
  while (heading >= 360.0) heading -= 360.0;

  String nmeaBody = "HCHDG," + String(heading, 1) + ",,,,";
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
