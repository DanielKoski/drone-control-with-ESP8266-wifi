#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>               // accel + gyro
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS3MDL.h>    // magnetometer




// ─── altIMU10v5 CONFIG ──────────────────────────────────────────────────────────────

Adafruit_LIS3MDL lis3mdl;

// Magnetometer noise RMS (in µT), e.g. you measured 0.8 µT
const float magRms      = 0.602;
// LIS3MDL default sensitivity is 1 µT/LSB for 4 gauss range
const float magScale    = 1.0;




//–––––– CONFIGURABLE ––––––//
// how long (ms) to spin the drone once you twist the magnetometer
const unsigned long ROTATE_TIME_MS       = 300;  

// how big a heading change (°) counts as a “twist” event
const float           HEADING_THRESHOLD  = 5.0;   

//–––––– STATE ––––––//
bool           airborne = false;
float          lastHeading = 0;
bool           rotating    = false;
unsigned long  rotateStart = 0;
int            rotateDir   = 0;  // +1 = right, -1 = left

const int STEP = 78

// yaw register values
const int YAW_NEUTRAL = 128;
const int YAW_LEFT    = YAW_NEUTRAL - STEP;
const int YAW_RIGHT   = YAW_NEUTRAL + STEP;

// LEFT (-) / RIGHT (+)
const int ROLL_NEUTRAL = 137;
const int ROLL_LEFT = ROLL_NEUTRAL - STEP;
const int ROLL_RIGHT = ROLL_NEUTRAL + STEP;

// BACKWARDS (-) / FORWARDS (+)
const int PITCH_NEUTRAL = 112;
const int PITCH_BACK = PITCH_NEUTRAL - STEP;
const int PITCH_FORWARD = PITCH_NEUTRAL + STEP;

// LOWER (-) / HIGHER (+)
const int THROTTLE_NEUTRAL = 128;
const int THROTTLE_DOWN = THROTTLE_NEUTRAL - STEP;
const int THROTTLE_UP = THROTTLE_NEUTRAL + STEP;

const unsigned long HEADING_CHECK_INTERVAL = 50;  // ms
static unsigned long lastCheck = 0;





// ─── CONFIG ──────────────────────────────────────────────────────────────────────
// Replace with your drone’s SSID (open network, no password)
const char* ssid         = "WIFI_UFO_3c2e90";
const IPAddress DRONE_IP(192, 168, 1, 1);
const uint16_t  DRONE_PORT = 7099;

// “Base” packet: [ 0x03, 0x66, roll, pitch, throttle, yaw, mode, checksum, 0x99 ]
uint8_t baseCmd[] = {
  0x03,   // start byte
  0x66,   // sequence/magic byte
  0x80,   // roll  = 128 (neutral)
  0x80,   // pitch = 128 (neutral)
  0x00,   // throttle (we will override)
  0x80,   // yaw   = 128 (neutral)
  0x00,   // mode  (override to 1,0,2)
  0x66,   // checksum placeholder
  0x99    // end byte
};

WiFiUDP udp;




// ─── Compute XOR checksum over bytes [2..6] ────────────────────────────────────────
uint8_t computeChecksum(const uint8_t *data) {
  uint8_t chk = 0;
  for (uint8_t i = 2; i <= 6; i++) {
    chk ^= data[i];
  }
  return chk;
}

// ─── Send one raw packet: [r, p, t, y, m] → FLY_DRONE_DATA ───────────────────────
void sendDroneCmd(uint8_t r, uint8_t p, uint8_t t, uint8_t y, uint8_t m) {
  uint8_t cmd[sizeof(baseCmd)];
  memcpy(cmd, baseCmd, sizeof(baseCmd));
  cmd[2] = r;   // roll
  cmd[3] = p;   // pitch
  cmd[4] = t;   // throttle
  cmd[5] = y;   // yaw
  cmd[6] = m;   // mode
  cmd[7] = computeChecksum(cmd);
  
  udp.beginPacket(DRONE_IP, DRONE_PORT);
  udp.write(cmd, sizeof(cmd));
  udp.endPacket();
}

// ─── 1) TAKEOFF: send m=1 at ~20 Hz for 1 s ───────────────────────────────────────
void droneTakeOff() {
  Serial.println("⚡️ TAKEOFF (m=1) → 1 second");
  unsigned long start = millis();
  while (millis() - start < 1000) {
    // neutral roll/pitch/yaw, throttle ~128, mode = 1
    sendDroneCmd(128, 128, 128, 128, 1);
    delay(50);  // ~20 Hz
  }
  Serial.println("✅  Takeoff burst complete.");
}

// ─── 2) HOVER: send m=0 (neutral-sticks, maintain last throttle) at ~20 Hz ────────
void droneHover(unsigned long hoverMillis) {
  Serial.printf("🕐 HOVER (m=0) → %lu ms\n", hoverMillis);
  unsigned long start = millis();
  while (millis() - start < hoverMillis) {
    // neutral roll/pitch/yaw, throttle = ~128, mode = 0 (“do nothing”)
    sendDroneCmd(128, 128, 128, 128, 0);
    delay(50);
  }
  Serial.println("✅  Hover loop complete.");
}

// ─── 3) LAND: send m=2 at ~20 Hz for 1 s ──────────────────────────────────────────
void droneLand() {
  Serial.println("🛬 LAND (m=2) → 1 second");
  unsigned long start = millis();
  while (millis() - start < 1000) {
    // neutral roll/pitch/yaw, throttle = ~128, mode = 2
    sendDroneCmd(128, 128, 128, 128, 2);
    delay(50);
  }
  Serial.println("✅  Land burst complete.");
}

float readMagHeading() {
  // e.g. vector from magX/magY:

  // 1) read raw sensors
  sensors_event_t magEvt;
  lis3mdl.getEvent(&magEvt);

  // 2) convert mag to µT and apply dead-band
  float mx = magEvt.magnetic.z * magScale;
  float my = magEvt.magnetic.y * magScale;
  if (fabs(mx) < magRms) mx = 0;
  if (fabs(my) < magRms) my = 0;


  float bearing = atan2(my, mx) * 180.0 / PI;
  if (bearing < 0) bearing += 360;
  return bearing;
}

void writeMotors(unsigned long now) {
  
  // 1) If we’re in a rotation window, keep spinning:
  if (rotating) {
    if (now - rotateStart < ROTATE_TIME_MS) {
      // continue rotating
      int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW, 0);
      return;  
    } else {
      // done rotating
      rotating = false;
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);      // reset lastHeading baseline
      lastHeading = readMagHeading();
      return;
    }
  }

  // 2) Otherwise, read new heading and check for a twist:
  float heading = readMagHeading();
  float delta   = heading - lastHeading;
  // wrap into [-180,180]
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  if (fabs(delta) >= HEADING_THRESHOLD) {
    // start a rotation
    rotating    = true;
    rotateStart = now;
    rotateDir   = (delta > 0 ? +1 : -1);
    // send first command immediately
    int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
    sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW, 0);
    return;
  }

  // 3) else do nothing (hover neutral)

  sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);
}

void turn() {
  unsigned long now = millis();
  rotateStart = now;
  
  // 1) If we’re in a rotation window, keep spinning:
  if (rotating) {
    if (now - rotateStart < ROTATE_TIME_MS) {
      // continue rotating
      int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, 38, 0);
      Serial.println("Damn");
      return;  
    } else {
      // done rotating
      rotating = false;
      Serial.println("done");
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);      // reset lastHeading baseline
      lastHeading = readMagHeading();
      return;
    }
  }
  Serial.println("not good");

  // 2) Otherwise, read new heading and check for a twist:
  float heading = readMagHeading();
  float delta   = heading - lastHeading;
  // wrap into [-180,180]
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  if (fabs(delta) >= HEADING_THRESHOLD) {
    // start a rotation
    rotating    = true;
    rotateStart = now;
    rotateDir   = (delta > 0 ? +1 : -1);
    // send first command immediately
    int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
    sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, 38, 0);
    return;
  }

  // 3) else do nothing (hover neutral)

  sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);
}

void setup() {
  Serial.begin(115200);

  // altIMU10v5 setup
  Wire.begin();
  delay(100);

  // — magnetometer init (default 0x1E address) —
  constexpr uint8_t MAG_ADDR = 0x1E;
  while (!lis3mdl.begin_I2C(MAG_ADDR, &Wire)) {
    Serial.println("Failed to detect LIS3MDL!");
    
  }


  lastHeading = readMagHeading();


  // estabilish connection to drone
  delay(10);
  Serial.printf("🔌 Connecting to SSID: %s\n", ssid);


  

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ Could not join drone hotspot. Halting.");
    while (true) delay(1000);
  }
  Serial.println("✅ Connected to drone hotspot!");
  Serial.print("   Local IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(44444);

  // Give the drone ~5 seconds to finish booting its Wi-Fi & flight controller
  Serial.println("⏳ Waiting 5 s before TAKEOFF...");
  delay(5000);

  // ─── Sequence: Take off → Hover → Land ─────────────────────────────────────────
  // droneTakeOff();           // (m=1) for 1 s → climb to hover altitude
  // droneHover(5000);         // (m=0) for 5 s → maintain hovering
  // droneLand();              // (m=2) for 1 s → descent
  
}

void loop() {
  unsigned long now = millis();

  /*if (now - lastCheck >= HEADING_CHECK_INTERVAL) {
    lastCheck = now;
    writeMotors(now);
  } */

  
  /*
  // 1) If we’re in a rotation window, keep spinning:
  if (rotating) {
    if (now - rotateStart < ROTATE_TIME_MS) {
      // continue rotating
      int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW, 0);
      return;  
    } else {
      // done rotating
      rotating = false;
      sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);      // reset lastHeading baseline
      lastHeading = readMagHeading();
      return;
    }
  }

  // 2) Otherwise, read new heading and check for a twist:
  float heading = readMagHeading();
  float delta   = heading - lastHeading;
  // wrap into [-180,180]
  if (delta > 180)  delta -= 360;
  if (delta < -180) delta += 360;

  if (fabs(delta) >= HEADING_THRESHOLD) {
    // start a rotation
    rotating    = true;
    rotateStart = now;
    rotateDir   = (delta > 0 ? +1 : -1);
    // send first command immediately
    int YAW = (rotateDir > 0 ? YAW_RIGHT : YAW_LEFT);
    sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW, 0);
    return;
  }
  */
  // 3) else do nothing (hover neutral)

  // sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, 128, YAW_NEUTRAL, 0);



  // land drone if spacebar is pressed
  if (Serial.available()) {
    char c = Serial.read();

    switch (c) {
      case 'W':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_FORWARD, THROTTLE_NEUTRAL, YAW_NEUTRAL, 0);
        break;
      case 'S':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_BACK, THROTTLE_NEUTRAL, YAW_NEUTRAL, 0);
        break;
      case 'A':
        sendDroneCmd(ROLL_LEFT, PITCH_NEUTRAL, THROTTLE_NEUTRAL, YAW_NEUTRAL, 0);
        break;
      case 'D':
        sendDroneCmd(ROLL_RIGHT, PITCH_NEUTRAL, THROTTLE_NEUTRAL, YAW_NEUTRAL, 0);
        break;
      case 'L':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, THROTTLE_NEUTRAL, YAW_LEFT, 0);
        break;
      case 'R':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, THROTTLE_NEUTRAL, YAW_RIGHT, 0);
        break;
      case 'U':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, THROTTLE_UP, YAW_NEUTRAL, 0);
        break;
      case 'DN':
        sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, THROTTLE_DOWN, YAW_NEUTRAL, 0);
        break;
      case ' ':
        if (airborne) {
          droneLand();
          airborne = false;
        }
        else {
          droneTakeOff();
          airborne = true;
        }
        break;
    }
    
  }
  else {
    sendDroneCmd(ROLL_NEUTRAL, PITCH_NEUTRAL, THROTTLE_NEUTRAL, YAW_NEUTRAL, 0);
  }
  
  delay(50);
}
