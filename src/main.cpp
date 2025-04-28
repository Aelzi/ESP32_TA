#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>  // Pastikan install library EspSoftwareSerial melalui PlatformIO

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

//=== Pins Motor & PWM ===
const int pin1    = 27;
const int pin2    = 26;
const int pinEn   = 14;
const int pwmCh   = 0;
const int pwmFreq = 5000;
const int pwmRes  = 8;

//=== Pins Water Pump ===
const int pump1Pin = 32;  // Pompa 1
const int pump2Pin = 33;  // Pompa 2
const int pump3Pin = 25;  // Pompa 3

//=== Sensor Ultrasonik ===
// Sensor 1 (HardwareSerial2)
HardwareSerial sensor1(2);
const int s1_RX = 16, s1_TX = 17;

// Sensor 2 (HardwareSerial1)
HardwareSerial sensor2(1);
const int s2_RX = 4, s2_TX = 15;

// Sensor 3 (EspSoftwareSerial)
SoftwareSerial sensor3(18, 19); // pin RX, TX

//=== Scheduler ===
const unsigned long INTERVAL = 2000; //ms
unsigned long lastMillis = 0;

//=== Motor State ===
enum MotorState { STOPPED, OPENING, CLOSING };
MotorState motorState = STOPPED;
unsigned long motorStart = 0;
unsigned long motorDur   = 0;

// Prototipe fungsi
void initHardware();
bool readUltrasonic(Stream &stream, float &dist);
void sendToFirebase(float dist, const char* path);
void checkCommands();
void updateMotor();
void startMotor(MotorState dir, int pct);
void waterPump1On();
void waterPump1Off();
void waterPump2On();
void waterPump2Off();
void waterPump3On();
void waterPump3Off();
void checkPumps();

void setup() {
  Serial.begin(115200);
  Serial.println("=== BOOT ===");

  // WiFi Connection
  Serial.print("WiFi: connect…");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200);
  }
  Serial.println(" OK");

  // Firebase Initialization
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email    = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase: ready");

  // Initialize Hardware
  initHardware();
  Serial.println("Hardware: initialized");
}

void loop() {
    unsigned long now = millis();
    // Update motor secara real time
    updateMotor();
    
    if (now - lastMillis < INTERVAL) {
        return;
    }
    lastMillis = now;
    Serial.println("\n--- Cycle start ---");
    
    // 1) Baca sensor ultrasonik 1
    float dist;
    if (readUltrasonic(sensor1, dist)) {
        Serial.printf("Sensor 1: %.1f cm\n", dist);
        sendToFirebase(dist, "/TEST/s1");
    } else {
        Serial.println("Sensor 1: read FAILED");
    }
    
    // 2) Baca sensor ultrasonik 2
    if (readUltrasonic(sensor2, dist)) {
        Serial.printf("Sensor 2: %.1f cm\n", dist);
        sendToFirebase(dist, "/TEST/s2");
    } else {
        Serial.println("Sensor 2: read FAILED");
    }
    
    // 3) Baca sensor ultrasonik 3
    if (readUltrasonic(sensor3, dist)) {
        Serial.printf("Sensor 3: %.1f cm\n", dist);
        sendToFirebase(dist, "/TEST/s3");
    } else {
        Serial.println("Sensor 3: read FAILED");
    }
    
    // 4) Baca perintah motor dari Firebase
    checkCommands();
    
    // 5) Cek status water pump
    checkPumps();
}

void initHardware() {
  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);
  pinMode(pinEn, OUTPUT);

  // Inisiasi pin water pump (active HIGH)
  pinMode(pump1Pin, OUTPUT);
  digitalWrite(pump1Pin, LOW);
  pinMode(pump2Pin, OUTPUT);
  digitalWrite(pump2Pin, LOW);
  pinMode(pump3Pin, OUTPUT);
  digitalWrite(pump3Pin, LOW);

  // Setup PWM motor
  ledcSetup(pwmCh, pwmFreq, pwmRes);
  ledcAttachPin(pinEn, pwmCh);

  // Inisiasi sensor ultrasonik
  sensor1.begin(9600, SERIAL_8N1, s1_RX, s1_TX);
  sensor2.begin(9600, SERIAL_8N1, s2_RX, s2_TX);
  sensor3.begin(9600); // EspSoftwareSerial hanya butuh kecepatan baud
}

// Fungsi generik membaca sensor ultrasonik dengan timeout yang diperpanjang
bool readUltrasonic(Stream &stream, float &dist) {
    // Bersihkan buffer terlebih dahulu
    while (stream.available()) stream.read();
    
    uint8_t buf[4] = {0};
    unsigned long t0 = millis();
    
    // Tunggu header 0xFF, timeout diperpanjang menjadi 100ms
    while (millis() - t0 < 100) {
        if (stream.available()) {
            uint8_t byteRead = stream.read();
            if (byteRead == 0xFF) {
                buf[0] = 0xFF;
                break;
            }
        }
    }
    if (buf[0] != 0xFF) return false;
    
    // Baca byte 1 sampai 3 dengan timeout masing-masing 100ms
    for (int i = 1; i < 4; i++) {
        unsigned long t1 = millis();
        while (!stream.available()) {
            if (millis() - t1 > 100) return false;
        }
        buf[i] = stream.read();
    }
    
    // Cek checksum
    if ((((buf[0] + buf[1] + buf[2]) & 0xFF) != buf[3])) return false;
    
    // Hitung jarak (dalam cm)
    dist = ((buf[1] << 8) | buf[2]) / 10.0;
    return true;
}

// Fungsi untuk mengirim data ke Firebase pada path yang diberikan
void sendToFirebase(float dist, const char* path) {
  Serial.printf("Firebase: sending %.1f to %s…", dist, path);
  if (Firebase.RTDB.setFloat(&fbdo, path, dist)) {
    Serial.println(" OK");
  } else {
    Serial.printf(" ERROR: %s\n", fbdo.errorReason().c_str());
  }
}

void checkCommands() {
  int pb = 0, pt = 0;
  Serial.print("Cek PB…");
  if (Firebase.RTDB.getInt(&fbdo, "/TEST/PB"))
    pb = fbdo.intData();
  Serial.printf(" %d\n", pb);

  Serial.print("Cek PT…");
  if (Firebase.RTDB.getInt(&fbdo, "/TEST/PT"))
    pt = fbdo.intData();
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

void startMotor(MotorState dir, int pct) {
  pct = constrain(pct, 1, 100);
  unsigned long calculatedDur = (unsigned long)pct * 5000UL / 100UL;
  motorDur = max(calculatedDur, 2500UL);
  motorStart = millis();
  motorState = dir;
  
  if (dir == OPENING) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    Serial.println("Motor: mulai OPENING");
  } else {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    Serial.println("Motor: mulai CLOSING");
  }
  ledcWrite(pwmCh, 255);
  Serial.printf("Motor: %s for %lums\n", (dir == OPENING ? "OPENING" : "CLOSING"), motorDur);
}

void updateMotor() {
  if (motorState == STOPPED) return;
  if (millis() - motorStart >= motorDur) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    ledcWrite(pwmCh, 0);
    motorState = STOPPED;
    Serial.println("Motor: STOPPED");
  } else {
    Serial.printf("Motor running: sisa waktu %lums\n", motorDur - (millis() - motorStart));
  }
}

void waterPump1On() {
  digitalWrite(pump1Pin, HIGH);
  Serial.println("Water Pump 1: ON");
}

void waterPump1Off() {
  digitalWrite(pump1Pin, LOW);
  Serial.println("Water Pump 1: OFF");
}

void waterPump2On() {
  digitalWrite(pump2Pin, HIGH);
  Serial.println("Water Pump 2: ON");
}

void waterPump2Off() {
  digitalWrite(pump2Pin, LOW);
  Serial.println("Water Pump 2: OFF");
}

void waterPump3On() {
  digitalWrite(pump3Pin, HIGH);
  Serial.println("Water Pump 3: ON");
}

void waterPump3Off() {
  digitalWrite(pump3Pin, LOW);
  Serial.println("Water Pump 3: OFF");
}

void checkPumps() {
  // Pump 1
  bool pump1Status = false;
  Serial.print("Cek pump1… ");
  if (Firebase.RTDB.getBool(&fbdo, "/TEST/pump1"))
    pump1Status = fbdo.boolData();
  else
    Serial.printf("Error: %s\n", fbdo.errorReason().c_str());
  Serial.printf("%s\n", pump1Status ? "ON" : "OFF");
  pump1Status ? waterPump1On() : waterPump1Off();

  // Pump 2
  bool pump2Status = false;
  Serial.print("Cek pump2… ");
  if (Firebase.RTDB.getBool(&fbdo, "/TEST/pump2"))
    pump2Status = fbdo.boolData();
  else
    Serial.printf("Error: %s\n", fbdo.errorReason().c_str());
  Serial.printf("%s\n", pump2Status ? "ON" : "OFF");
  pump2Status ? waterPump2On() : waterPump2Off();

  // Pump 3
  bool pump3Status = false;
  Serial.print("Cek pump3… ");
  if (Firebase.RTDB.getBool(&fbdo, "/TEST/pump3"))
    pump3Status = fbdo.boolData();
  else
    Serial.printf("Error: %s\n", fbdo.errorReason().c_str());
  Serial.printf("%s\n", pump3Status ? "ON" : "OFF");
  pump3Status ? waterPump3On() : waterPump3Off();
}