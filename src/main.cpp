#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <HardwareSerial.h>

//=== Konfig WiFi & Firebase ===
#define WIFI_SSID     "Farel"
#define WIFI_PASSWORD "123456789"
#define API_KEY       "AIzaSyCvyT7WSOgRdLJ4ZzUtsTt3OEygEMVPJlE"
#define DATABASE_URL  "test-ta-f2816-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL    "alfarezi31as@gmail.com"
#define USER_PASSWORD "123456789"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//=== Pins motor & PWM ===
const int pin1    = 27;
const int pin2    = 26;
const int pinEn   = 14;
const int pwmCh   = 0;
const int pwmFreq = 5000;
const int pwmRes  = 8;

//=== Ultrasonic UART (serial) ===
HardwareSerial ultra(2);
const int RX_PIN = 16, TX_PIN = 17;

//=== Scheduler ===
const unsigned long INTERVAL = 2000; //ms
unsigned long lastMillis = 0;

//=== Motor state ===
enum MotorState { STOPPED, OPENING, CLOSING };
MotorState motorState = STOPPED;
unsigned long motorStart=0, motorDur=0;

//— Prototypes —
void initHardware();
bool readDistance(float &dist);
void sendToFirebase(float dist);
void checkCommands();
void updateMotor();
void startMotor(MotorState dir, int pct);

void setup() {
  Serial.begin(115200);
  Serial.println("=== BOOT ===");

  // Wi-Fi
  Serial.print("WiFi: connect…");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200);
  }
  Serial.println(" OK");

  // Firebase
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email    = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase: ready");

  // Hardware
  initHardware();
  Serial.println("Hardware: initialized");
}

void loop() {
  unsigned long now = millis();
  if (now - lastMillis < INTERVAL) {
    updateMotor();
    return;
  }
  lastMillis = now;
  Serial.println("\n--- Cycle start ---");

  // 1) Baca ultrasonic
  float dist;
  if (readDistance(dist)) {
    Serial.printf("Ultrasonic: %.1f cm\n", dist);
    // 2) Kirim ke Firebase
    sendToFirebase(dist);
  } else {
    Serial.println("Ultrasonic: read FAILED");
  }

  // 3) Cek perintah motor
  checkCommands();

  // 4) Update motor (stop jika perlu)
  updateMotor();
}

// Inisialisasi motor + serial ultrasonic
void initHardware() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pinEn, OUTPUT);
  ledcSetup(pwmCh, pwmFreq, pwmRes);
  ledcAttachPin(pinEn, pwmCh);
  ultra.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
}

// Baca 4-byte frame sensor, timeout per byte 50ms
bool readDistance(float &dist) {
  // buang sisa buffer
  while (ultra.available()) ultra.read();
  uint8_t buf[4];
  unsigned long t0 = millis();

  // cari header 0xFF
  while (millis() - t0 < 50) {
    if (ultra.available() && ultra.read() == 0xFF) {
      buf[0] = 0xFF;
      break;
    }
  }
  if (buf[0] != 0xFF) return false;

  // baca byte 1–3
  for (int i = 1; i < 4; i++) {
    unsigned long t1 = millis();
    while (!ultra.available()) {
      if (millis() - t1 > 50) return false;
    }
    buf[i] = ultra.read();
  }

  // cek checksum
  if ((((buf[0] + buf[1] + buf[2]) & 0xFF) != buf[3])) return false;

  // hitung jarak
  dist = ((buf[1] << 8) | buf[2]) / 10.0;
  return true;
}

// Kirim float ke RTDB
void sendToFirebase(float dist) {
  Serial.printf("Firebase: sending %.1f…", dist);
  if (Firebase.RTDB.setFloat(&fbdo, "/TEST/s1", dist)) {
    Serial.println(" OK");
  } else {
    Serial.printf(" ERROR: %s\n", fbdo.errorReason().c_str());
  }
}

// Cek PB & PT, eksekusi lalu reset di DB
void checkCommands() {
  int pb = 0, pt = 0;
  Serial.print("Cek PB…");
  if (Firebase.RTDB.getInt(&fbdo, "/TEST/PB")) pb = fbdo.intData();
  Serial.printf(" %d\n", pb);
  Serial.print("Cek PT…");
  if (Firebase.RTDB.getInt(&fbdo, "/TEST/PT")) pt = fbdo.intData();
  Serial.printf(" %d\n", pt);

  if (pb > 0) {
    Serial.printf("Action: OPEN %d%%\n", pb);
    startMotor(OPENING, pb);
    Firebase.RTDB.setInt(&fbdo, "/TEST/PB", 0);
  } else if (pt > 0) {
    Serial.printf("Action: CLOSE %d%%\n", pt);
    startMotor(CLOSING, pt);
    Firebase.RTDB.setInt(&fbdo, "/TEST/PT", 0);
  } else {
    Serial.println("Action: none");
  }
}

// Stop motor jika durasi habis
void updateMotor() {
  if (motorState == STOPPED) return;
  if (millis() - motorStart >= motorDur) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    ledcWrite(pwmCh, 0);
    motorState = STOPPED;
    Serial.println("Motor: STOPPED");
  }
}

// Nyalakan motor non-blocking
void startMotor(MotorState dir, int pct) {
  pct = constrain(pct, 1, 100);
  motorDur   = (unsigned long)pct * 5000UL / 100UL;
  motorStart = millis();
  motorState = dir;
  if (dir == OPENING) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  ledcWrite(pwmCh, 200);
  Serial.printf("Motor: %s for %lums\n",
                dir==OPENING?"OPENING":"CLOSING", motorDur);
}
