#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>  // Pastikan install library EspSoftwareSerial melalui PlatformIO
#include <LiquidCrystal_I2C.h>  // Tambahan untuk LCD

//=== Konfig WiFi & Firebase Production ===
#define WIFI_SSID     "Farel"
#define WIFI_PASSWORD "x"
#define API_KEY       "x"
#define DATABASE_URL  "x-x-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL    "alfarezi31as@gmail.com"
#define USER_PASSWORD "x"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//=== Pins Motor & PWM ====
const int pin1    = 27;
const int pin2    = 26;
const int pinEn   = 14;
const int pwmCh   = 0;
const int pwmFreq = 5000;
const int pwmRes  = 8;

//=== Pins Water Pump ====
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

// --  Global Variables untuk Water Flow Sensor --
const int flowPin1 = 34;
const int flowPin2 = 35;
const int flowPin3 = 39;

volatile unsigned long flowCount1 = 0;
volatile unsigned long flowCount2 = 0;
volatile unsigned long flowCount3 = 0;

// Faktor konversi: misal, untuk YF-S201 biasanya Flow (L/min) = (PulseCount per 2 sec) / 15
// (Karena 7.5 pps per L/min => pada 2 detik, 15 pulsa setara 1 L/min)

// ISR untuk masing-masing sensor flow
void IRAM_ATTR flowISR1() { flowCount1++; }
void IRAM_ATTR flowISR2() { flowCount2++; }
void IRAM_ATTR flowISR3() { flowCount3++; }

//=== LCD Initialization ===
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Sesuaikan alamat I2C jika perlu

// Variabel status untuk LCD
bool sensor1Ok = false;
bool sensor2Ok = false;
bool sensor3Ok = false;
bool motorRunning = false;
unsigned long lastTxTime = 0;  // Waktu terakhir data terkirim berhasil ke Firebase

// Variabel untuk batch update data sensor (misal: 30 detik)
const unsigned long sensorWriteInterval = 1000; // 30 detik
unsigned long lastSensorWriteTime = 0;

// Tambahan global variable untuk posisi pintu saat ini (0 = tertutup, 100 = terbuka maksimal)
int currentDoorPos = 0;

// Tambahkan deklarasi buzzerPin dan variabel sensor terbaru
const int buzzerPin = 12;  // Pilih pin yang tidak digunakan (misal: GPIO 12)
float latestUltrasonic1 = 0, latestUltrasonic2 = 0, latestUltrasonic3 = 0;
float latestFlow1 = 0, latestFlow2 = 0, latestFlow3 = 0;

// Prototipe fungsi
void initHardware();
bool readUltrasonic(Stream &stream, float &dist);
void sendToFirebase(float data, const char* path);
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
void updateLCD();  // Fungsi LCD baru
String getTimestamp();  // Fungsi helper baru
void checkPintuAir();  // Fungsi baru
void checkBuzzer();  // Fungsi baru
void debugBuzzerCause();  // Fungsi baru
void updateBuzzerSensors();  // Fungsi baru

void setup() {
  Serial.begin(115200);
  Serial.println("=== BOOT ===");

  lcd.init();
  lcd.backlight();

  // Inisialisasi buzzer
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  // WiFi Connection
  Serial.print("WiFi: connect…");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(200);
  }
  Serial.println(" OK");

  // Konfigurasi NTP (WIB: UTC+7)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
  } else {
    Serial.println("Time synchronized");
  }

  // Firebase Initialization
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email    = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  Serial.println("Firebase: ready");

  initHardware();
  Serial.println("Hardware: initialized");
}

void loop() {
    static unsigned long lastBuzzerSensorUpdate = 0;
    unsigned long now = millis();
    
    // Perbarui sensor untuk buzzer setiap 500 ms agar pengecekan bersifat real time
    if(now - lastBuzzerSensorUpdate >= 500) {
        updateBuzzerSensors();
        lastBuzzerSensorUpdate = now;
    }
    
    // Lanjutkan fungsi-fungsi lain: batch update ke Firebase, kontrol pintu/pompa, update LCD, dll.
    updateMotor();
    
    if (now - lastSensorWriteTime >= sensorWriteInterval) {
        // ... (kode batch update Firebase seperti biasa)
    }
    
    checkPintuAir();
    checkPumps();
    motorRunning = (motorState != STOPPED);
    updateLCD();
    checkBuzzer();  // Pengecekan buzzer menggunakan nilai sensor yang sudah update secara real time

    // Tambahkan di loop() session debug, misalnya setiap 3 detik:
    static unsigned long lastFlowDebug = 0;
    if (millis() - lastFlowDebug >= 3000) {
      Serial.printf("Flow counts: %lu, %lu, %lu\n", flowCount1, flowCount2, flowCount3);
      lastFlowDebug = millis();
    }
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
  sensor3.begin(9600); // SoftwareSerial: hanya baud rate
  
  // -- Inisiasi sensor Water Flow --
  pinMode(flowPin1, INPUT);
  pinMode(flowPin2, INPUT);
  pinMode(flowPin3, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin1), flowISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin2), flowISR2, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin3), flowISR3, RISING);
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

// Fungsi untuk mengirim data ke Firebase (write new child untuk sensor, overwrite untuk kontrol)
void sendToFirebase(float data, const char* path) {
  // Jika path mengandung "/Polder/", lakukan push data dengan key timestamp
  if (strstr(path, "/Polder/") != nullptr) {
    String timestamp = getTimestamp();
    String fullPath = String(path) + "/" + timestamp;
    if (Firebase.RTDB.setFloat(&fbdo, fullPath.c_str(), data)) {
      Serial.printf("Firebase: pushed %.1f to %s\n", data, fullPath.c_str());
      lastTxTime = millis();
    } else {
      Serial.printf("Firebase: ERROR pushing to %s: %s\n", fullPath.c_str(), fbdo.errorReason().c_str());
    }
  }
  else {
    // Untuk kontrol (misal pb, pt)
    if (Firebase.RTDB.setFloat(&fbdo, path, data)) {
      Serial.printf("Firebase: sent %.1f to %s OK\n", data, path);
      lastTxTime = millis();
    } else {
      Serial.printf("Firebase: ERROR setting %s: %s\n", path, fbdo.errorReason().c_str());
    }
  }
}

void checkCommands() {
  int pb = 0, pt = 0;
  Serial.print("Cek PB…");
  if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PB"))
    pb = fbdo.intData();
  Serial.printf(" %d\n", pb);

  Serial.print("Cek PT…");
  if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PT"))
    pt = fbdo.intData();
  Serial.printf(" %d\n", pt);

  if (pb > 0) {
    Serial.printf("Action: OPEN %d%%\n", pb);
    startMotor(OPENING, pb);
    Firebase.RTDB.setInt(&fbdo, "/Kontrol/PB", 0);
  } else if (pt > 0) {
    Serial.printf("Action: CLOSE %d%%\n", pt);
    startMotor(CLOSING, pt);
    Firebase.RTDB.setInt(&fbdo, "/Kontrol/PT", 0);
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
  static bool lastPump1 = false, lastPump2 = false, lastPump3 = false;
  bool pump1Status = false;
  if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump1"))
    pump1Status = fbdo.boolData();
  bool pump2Status = false;
  if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump2"))
    pump2Status = fbdo.boolData();
  bool pump3Status = false;
  if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump3"))
    pump3Status = fbdo.boolData();
  
  if (pump1Status != lastPump1) {
    Serial.printf("Pump1 changed: %s\n", pump1Status ? "ON" : "OFF");
    pump1Status ? waterPump1On() : waterPump1Off();
    lastPump1 = pump1Status;
  }
  if (pump2Status != lastPump2) {
    Serial.printf("Pump2 changed: %s\n", pump2Status ? "ON" : "OFF");
    pump2Status ? waterPump2On() : waterPump2Off();
    lastPump2 = pump2Status;
  }
  if (pump3Status != lastPump3) {
    Serial.printf("Pump3 changed: %s\n", pump3Status ? "ON" : "OFF");
    pump3Status ? waterPump3On() : waterPump3Off();
    lastPump3 = pump3Status;
  }
}

// Fungsi LCD: Menampilkan status sistem di LCD 4x20
void updateLCD() {
    lcd.clear();
    // Baris 1: Status Sistem
    lcd.setCursor(0, 0);
    lcd.print("SYS: RUN");
    
    // Baris 2: Status Jaringan/Cloud
    lcd.setCursor(0, 1);
    lcd.print("NET: ");
    lcd.print(WiFi.status()==WL_CONNECTED ? "OK" : "ERR");
    
    // Baris 3: Waktu data terakhir terkirim (dalam detik)
    lcd.setCursor(0, 2);
    lcd.print("Tx: ");
    unsigned long secs = (millis() - lastTxTime) / 1000;
    lcd.print(secs);
    lcd.print("s");
    
    // Baris 4: Status Sensor dan Motor
    lcd.setCursor(0, 3);
    // Format: "1:O 2:O 3:F M:S" => O=OK, F=Fail; M: R=Running, S=Stopped.
    lcd.print("1:");
    lcd.print(sensor1Ok ? "O" : "F");
    lcd.print(" 2:");
    lcd.print(sensor2Ok ? "O" : "F");
    lcd.print(" 3:");
    lcd.print(sensor3Ok ? "O" : "F");
    lcd.print(" M:");
    lcd.print(motorRunning ? "R" : "S");
}

// Fungsi helper: Mendapatkan timestamp dalam format "YYYY-MM-DD-HH_MM_SS"
String getTimestamp() {
  struct tm timeinfo;
  unsigned long start = millis();
  // Tunggu hingga waktu tersinkron maksimal 5 detik
  while(!getLocalTime(&timeinfo) && (millis() - start < 5000)) {
    delay(100);
  }
  if (!getLocalTime(&timeinfo)) {
    // Jika gagal sinkron waktu setelah 5 detik
    return "0000-00-00-00_00_00";
  }
  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H_%M_%S", &timeinfo);
  return String(buffer);
}

// Fungsi cek status pintu (motor) dengan hanya mencetak saat terjadi perubahan
void checkPintuAir() {
  static int lastTargetPos = -1;  // Nilai awal berbeda sehingga pasti tercetak pada awalnya
  int targetPos = 0;
  // Ambil nilai dari Firebase
  if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PintuAir"))
    targetPos = fbdo.intData();
  
  // Hanya cetak jika target berubah
  if (targetPos != lastTargetPos) {
    Serial.printf("Cek PintuAir... %d\n", targetPos);
    lastTargetPos = targetPos;
  }
  
  // Jika motor sedang tidak berjalan dan target berbeda dengan posisi saat ini, jalankan perintah
  if (motorState == STOPPED && targetPos != currentDoorPos) {
    int diff = targetPos - currentDoorPos;
    if (diff > 0) {
      Serial.printf("Action: OPEN dari %d%% ke %d%% (selisih %d%%)\n", currentDoorPos, targetPos, diff);
      startMotor(OPENING, diff);
    } else {
      Serial.printf("Action: CLOSE dari %d%% ke %d%% (selisih %d%%)\n", currentDoorPos, targetPos, -diff);
      startMotor(CLOSING, -diff);
    }
    currentDoorPos = targetPos;
    // Opsional: reset perintah di Firebase
    Firebase.RTDB.setInt(&fbdo, "/Kontrol/PintuAir", currentDoorPos);
  } else {
    // Hanya cetak jika terjadi perubahan target (agar tidak membanjiri serial)
    static bool printedNoAction = false;
    if (!printedNoAction) {
      Serial.println("PintuAir: Tidak ada aksi, posisi sudah sesuai");
      printedNoAction = true;
    }
  }
}

void checkBuzzer() {
  bool buzzerCondition = false;
  
  // Sebelum cek, tampilkan debug detail penyebab buzzer menyala
  debugBuzzerCause();
  
  // Cek kondisi dari motor
  if (motorRunning) {
    buzzerCondition = true;
  }
  
  // Cek kondisi sensor ultrasonik
  if (sensor1Ok && latestUltrasonic1 < 5.0) {
    buzzerCondition = true;
  }
  if (sensor2Ok && latestUltrasonic2 < 5.0) {
    buzzerCondition = true;
  }
  if (sensor3Ok && latestUltrasonic3 < 5.0) {
    buzzerCondition = true;
  }
  
  // Cek kondisi sensor water flow
  if (latestFlow1 > 5.0) {
    buzzerCondition = true;
  }
  if (latestFlow2 > 5.0) {
    buzzerCondition = true;
  }
  if (latestFlow3 > 5.0) {
    buzzerCondition = true;
  }
  
  if (buzzerCondition) {
    digitalWrite(buzzerPin, HIGH);
    Serial.println("DEBUG: Buzzer ON");
    delay(200);  // Durasi buzzer
    digitalWrite(buzzerPin, LOW);
    Serial.println("DEBUG: Buzzer OFF");
  }
}

// Fungsi untuk menampilkan debug detail penyebab buzzer menyala
void debugBuzzerCause() {
  Serial.println("=== Debug Buzzer Cause ===");
  
  // Cek kondisi motor
  if (motorRunning) {
    Serial.println(" - Motor running: TRUE");
  } else {
    Serial.println(" - Motor running: FALSE");
  }
  
  // Cek sensor ultrasonik
  if (sensor1Ok) {
    Serial.printf(" - Sensor1: %.1f cm", latestUltrasonic1);
    if(latestUltrasonic1 < 5.0) Serial.println(" -> BELOW threshold (5.0 cm)");
    else Serial.println(" -> OK");
  } else {
    Serial.println(" - Sensor1: read FAILED");
  }
  
  if (sensor2Ok) {
    Serial.printf(" - Sensor2: %.1f cm", latestUltrasonic2);
    if(latestUltrasonic2 < 5.0) Serial.println(" -> BELOW threshold (5.0 cm)");
    else Serial.println(" -> OK");
  } else {
    Serial.println(" - Sensor2: read FAILED");
  }
  
  if (sensor3Ok) {
    Serial.printf(" - Sensor3: %.1f cm", latestUltrasonic3);
    if(latestUltrasonic3 < 5.0) Serial.println(" -> BELOW threshold (5.0 cm)");
    else Serial.println(" -> OK");
  } else {
    Serial.println(" - Sensor3: read FAILED");
  }
  
  // Cek sensor water flow
  Serial.printf(" - Flow1: %.2f L/min", latestFlow1);
  if(latestFlow1 > 5.0) Serial.println(" -> ABOVE threshold (5.0 L/min)");
  else Serial.println(" -> OK");
  
  Serial.printf(" - Flow2: %.2f L/min", latestFlow2);
  if(latestFlow2 > 5.0) Serial.println(" -> ABOVE threshold (5.0 L/min)");
  else Serial.println(" -> OK");
  
  Serial.printf(" - Flow3: %.2f L/min", latestFlow3);
  if(latestFlow3 > 5.0) Serial.println(" -> ABOVE threshold (5.0 L/min)");
  else Serial.println(" -> OK");

  Serial.println("==========================");
}

// Fungsi updateBuzzerSensors() dipanggil setiap 500 ms untuk memperbarui nilai sensor secara real time untuk pengecekan buzzer
void updateBuzzerSensors() {
  float d;
  // Baca sensor Ultrasonik 1 (jika tersedia; jika gagal, biarkan nilai tak berubah)
  if (readUltrasonic(sensor1, d)) {
    latestUltrasonic1 = d;
    sensor1Ok = true;
  } else {
    sensor1Ok = false;
  }
  
  // Baca sensor Ultrasonik 2
  if (readUltrasonic(sensor2, d)) {
    latestUltrasonic2 = d;
    sensor2Ok = true;
  } else {
    sensor2Ok = false;
  }
  
  // Baca sensor Ultrasonik 3
  if (readUltrasonic(sensor3, d)) {
    latestUltrasonic3 = d;
    sensor3Ok = true;
  } else {
    sensor3Ok = false;
  }
}