#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// ================= Global Declarations =================

// Konfigurasi WiFi & Firebase
#define WIFI_SSID     "Floody-IoT"          
#define WIFI_PASSWORD "apalahapalah"  
// #define WIFI_SSID     "EJI-LPY"          
// #define WIFI_PASSWORD "123456789"     
// #define Web_API_KEY   "XXXXXXXXXX"
// #define DATABASE_URL  "test-ta-f2816-default-rtdb.asia-southeast1.firebasedatabase.app"
#define Web_API_KEY   "XXXXXXXXXX"
#define DATABASE_URL  "floody-252ef-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL    "alfarezi31as@gmail.com"
#define USER_PASS     "XXXXXXXXXX"

// Mapping Pin
#define pump1Pin 32
#define pump2Pin 33
#define pump3Pin 25

#define pump1BtnPin 4
#define pump2BtnPin 17
#define pump3BtnPin 23

#define STEPPER_ENABLE_PIN 26
#define STEPPER_DIR_PIN    27
#define STEPPER_STEP_PIN   14
#define BTN_OPEN    18
#define BTN_CLOSE   5

#define LAMP_GREEN_PIN 15
#define LAMP_RED_PIN   13

#define BUZZER_PIN 12



int currentPattern = 0; // 0 = off, 1 = double beep, 2 = error beep, 3 = rapid blink
unsigned long buzzerLastTime = 0;
int buzzerStep = 0;     // Tahap dalam pola buzzer


// Variabel Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
unsigned long lastTxTime = 0;

// Variabel tracking pompa
bool pump1ButtonPrev = HIGH;
bool pump2ButtonPrev = HIGH;
bool pump3ButtonPrev = HIGH;
bool pump1ButtonControl = false;
bool pump2ButtonControl = false;
bool pump3ButtonControl = false;
bool pump1State = false;
bool pump2State = false;
bool pump3State = false;


// Flag untuk inisialisasi dependency internet
bool internetDependenciesReady = false;
unsigned long doorMoveStartTime = 0;


// Variabel debounce & lainnya
unsigned long lastFirebasePoll = 0;
const unsigned long firebasePollInterval = 500;
const unsigned long debounceDelay = 0;
unsigned long lastPump1DebounceTime = 0;
unsigned long lastPump2DebounceTime = 0;
unsigned long lastPump3DebounceTime = 0;
unsigned long lastWifiCheckTime = 0;
const unsigned long wifiReconnectInterval = 10000;
const unsigned long TotalTimeOn = 300000; // 5 menit



// Deklarasi instance LCD (alamat 0x27, 20 kolom, 4 baris)
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Variabel untuk menyimpan waktu upload & baca terakhir
String lastUploadTime = "----";
String lastReadTime   = "----";

// Tambahkan variabel global untuk update LCD secara berkala
unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval =  1000; // update tiap 1 detik

// Tambahkan variabel global ini di bagian Global Declarations
String bootTime = "----";

// Tambahkan variabel global untuk melacak error token
unsigned long lastTokenErrorTime = 0;
bool tokenErrorActive = false;

// Tambahkan di bagian Global Declarations
unsigned long lastActivityTime = 0;


 
// Stepper config (0–100 % = 0…totalSteps)
const long   stepperTotalSteps      = 4650;                 // full travel
const float  stepperStepsPerPercent = stepperTotalSteps/100.0; 
int   doorPosition    = 0;   // [0..100]% current door position
long  stepPosition    = 0;   // current step index

// stepper sederhana tanpa library    // full travel = 6000 pulse (0→100%)
const unsigned long stepIntervalUs = 1000; // µs per step
long   doorTargetSteps = 0;     // target step for auto
bool   doorMoving      = false; // auto‐move flag
bool   manualStepping  = false; // manual‐move flag
int    stepDirection   = 0;     // +1 / –1
unsigned long lastStepTimeUs = 0;
const unsigned int pulseWidthUs = 10;  // HIGH pulse duration

// variabel global untuk menandai waktu manual selesai
unsigned long lastManualMoveTime = 0;
const unsigned long manualSyncDelay = 2000; // 2 detik
bool pendingFirebaseUpdate = false;

// ================= End Global Declarations =================

// Fungsi prototipe
void initHardware();
void initInternetDependencies();
void checkInternetReconnect();
void sendToFirebase(float data, const char* path);
void checkButtons();
void checkPumps();
void waterPump1On();
void waterPump1Off();
void waterPump2On();
void waterPump2Off();
void waterPump3On();
void waterPump3Off();
String getTimestamp();
void checkDoorControl();
void checkManualDoorControl();
void triggerBuzzerPattern(int pattern);
void updateBuzzer();
void checkBuzzerTrigger();
void startupBeep();
void checkLampIndicators();
void updateLCD();
void i2cScanner();
String repeatChar(char c, int count);
void serviceStepper();
void debugStepper(int steps, int direction, unsigned long intervalUs);

// ----- Buzzer Functions -----
void triggerBuzzerPattern(int pattern) {
    currentPattern = pattern;
    buzzerStep = 0;
    buzzerLastTime = millis();
    digitalWrite(BUZZER_PIN, LOW);
}

void updateBuzzer() {
    unsigned long now = millis();
    if (currentPattern == 1) {  
        if (buzzerStep == 0) {
            digitalWrite(BUZZER_PIN, HIGH);
            buzzerLastTime = now;
            buzzerStep = 1;
        } else if (buzzerStep == 1) {
            if(now - buzzerLastTime >= 300) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerLastTime = now;
                buzzerStep = 2;
            }
        } else if (buzzerStep == 2) {
            if(now - buzzerLastTime >= 300) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerLastTime = now;
                buzzerStep = 3;
            }
        } else if (buzzerStep == 3) {
            if(now - buzzerLastTime >= 300) {
                digitalWrite(BUZZER_PIN, LOW);
                currentPattern = 0;
            }
        }
    }
    else if (currentPattern == 2) { 
        if(buzzerStep == 0) {
            if(now - buzzerLastTime >= 10000) {
                digitalWrite(BUZZER_PIN, HIGH);
                buzzerLastTime = now;
                buzzerStep = 1;
            }
        } else if (buzzerStep == 1) {
            if(now - buzzerLastTime >= 300) {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerLastTime = now;
                buzzerStep = 0;
            }
        }
    }
    else if (currentPattern == 3) { 
        if(now - buzzerLastTime >= 100) {
            if (digitalRead(BUZZER_PIN) == LOW) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                digitalWrite(BUZZER_PIN, LOW);
            }
            buzzerLastTime = now;
        }
    }
    else {
        digitalWrite(BUZZER_PIN, LOW);
    }
}

void checkBuzzerTrigger() {
    // Kondisi 2: error, WiFi/timing masalah
    if(WiFi.status() != WL_CONNECTED || !internetDependenciesReady) {
        if(currentPattern != 2) {
            triggerBuzzerPattern(2);
        }
    }
    // Kondisi “dekat batas” selama gerak manual/otomatis
    else if ((manualStepping || doorMoving) &&
             ((stepDirection ==  1 && doorPosition >= 90) ||
              (stepDirection == -1 && doorPosition <= 10))) {
        if (currentPattern != 3) triggerBuzzerPattern(3);
    }
    // Kondisi 1: pintu di batas (0 atau 100) dan tombol ditekan
    else if (!doorMoving && !manualStepping && 
             ( (doorPosition == 0 && digitalRead(BTN_CLOSE)==LOW) ||
               (doorPosition == 100 && digitalRead(BTN_OPEN)==LOW) ) ) {
        if(currentPattern != 1) {
            triggerBuzzerPattern(1);
        }
    }
    else {
        if(currentPattern != 0) {
            currentPattern = 0;
            digitalWrite(BUZZER_PIN, LOW);
        }
    }
}

void startupBeep() {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
}

// // Callback token status
// void tokenStatusCallback(TokenInfo info) {
//     // Cetak status token untuk debug
//     Serial.printf("Token Info: type=%s, status=%s\n", info.type.c_str(), info.status.c_str());
// }

// ================= Fungsi Setup & Loop (tidak diubah) =================

void setup() {
    Serial.begin(115200);
    // Inisialisasi WiFi (untuk mendapatkan MAC Address)
    WiFi.mode(WIFI_STA);
    Serial.println("MAC Address: " + WiFi.macAddress());
    
    Wire.begin(21, 22);
    delay(1000);

    // Inisialisasi LCD
    lcd.init();
    lcd.backlight();

    // Inisialisasi pin dan hardware lainnya
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);
    digitalWrite(STEPPER_ENABLE_PIN, LOW);   // ENABLE active low
    
    // Inisialisasi pin tombol untuk pompa
    pinMode(pump1BtnPin, INPUT_PULLUP);
    pinMode(pump2BtnPin, INPUT_PULLUP);
    pinMode(pump3BtnPin, INPUT_PULLUP);
    
    // Inisialisasi pin untuk tombol pintu
    pinMode(BTN_OPEN, INPUT_PULLUP);
    pinMode(BTN_CLOSE, INPUT_PULLUP);

    // Inisialisasi pin buzzer
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    
    // Inisialisasi lampu indikator
    pinMode(LAMP_GREEN_PIN, OUTPUT);
    pinMode(LAMP_RED_PIN, OUTPUT);
    // Lampu hijau nyala (sistem normal), merah mati
    digitalWrite(LAMP_GREEN_PIN, HIGH);
    digitalWrite(LAMP_RED_PIN, LOW);
    
    // Inisialisasi hardware pompa dan stepper driver
    initHardware();
    
    // Coba koneksi WiFi dengan timeout 10 detik
    Serial.print("WiFi: connecting");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long wifiTimeout = 10000; // 10 detik
    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < wifiTimeout) {
        Serial.print('.');
        delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println(" OK");
        // Inisialisasi dependency internet (NTP, Firebase, dsb)
        initInternetDependencies();
    }
    else {
        Serial.println(" WiFi not connected, proceeding offline.");
    }
    
    Serial.println("Hardware: initialized");
    
    // Update LCD terakhir (agar tampilan sudah up-to-date)
    lcd.begin(20, 4); // Jika library mendukung
    lcd.backlight();
    updateLCD();
    
    // Semua komponen sudah di-setup; kini panggil startup beep.
    // Buzzer akan berbunyi hanya setelah semua inisialisasi selesai.
    startupBeep();

}

void loop() {
  checkManualDoorControl();

  // auto
  if (doorMoving) {
    serviceStepper();
    // auto-move baru saja selesai? paksa refresh LCD
    if (!doorMoving) {
      updateLCD(); lastLCDUpdate = millis();
    }
    return;
  }

  // manual
  if (manualStepping) {
    serviceStepper();
    // manual baru saja selesai? paksa refresh LCD
    if (!manualStepping) {
      updateLCD(); lastLCDUpdate = millis();
    }
    return;
  }

  // idle
  checkButtons();
  checkPumps();
  checkDoorControl();
  checkBuzzerTrigger();
  updateBuzzer();
  checkLampIndicators();

  if (millis() - lastLCDUpdate >= lcdUpdateInterval) {
    updateLCD();
    lastLCDUpdate = millis();
  }
}

// Fungsi inisialisasi hardware pompa
void initHardware() {
    // Pompa
    pinMode(pump1Pin, OUTPUT);
    digitalWrite(pump1Pin, LOW);
    pinMode(pump2Pin, OUTPUT);
    digitalWrite(pump2Pin, LOW);
    pinMode(pump3Pin, OUTPUT);
    digitalWrite(pump3Pin, LOW);


    // Inisialisasi stepper driver (NEMA27)
    pinMode(STEPPER_ENABLE_PIN, OUTPUT);  // ENA+  
    pinMode(STEPPER_DIR_PIN,    OUTPUT);  // DIR+  
    pinMode(STEPPER_STEP_PIN,   OUTPUT);  // PUL+  
    digitalWrite(STEPPER_ENABLE_PIN, LOW); // aktifkan driver (ENA low)
    digitalWrite(STEPPER_DIR_PIN, LOW);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    // Tombol pintu
    pinMode(BTN_OPEN,  INPUT_PULLUP);
    pinMode(BTN_CLOSE, INPUT_PULLUP);
}

// Fungsi untuk inisialisasi dependency internet (NTP, Firebase, dsb)
void initInternetDependencies() {
    // Konfigurasi NTP (WIB: UTC+7)
    configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
    } else {
        Serial.println("Time synchronized");
        char buffer[16];
        // Format: nama hari singkat (3 huruf), jam:menit 
        strftime(buffer, sizeof(buffer), "%a %H:%M", &timeinfo);
        // Set bootTime hanya satu kali (saat ESP32 mulai)
        if (bootTime == "----") {
            bootTime = String(buffer);
        }
    }
    
    // Konfigurasi Firebase
    config.api_key = Web_API_KEY;
    config.database_url = DATABASE_URL;
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASS;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
    internetDependenciesReady = true;
    Serial.println("Firebase: ready");

    if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PintuAir")) {
        doorPosition = fbdo.intData();
        stepPosition  = long(doorPosition * stepperStepsPerPercent);
        Serial.printf("Initial position: %d%% (%ld step)\n",
                      doorPosition, stepPosition);
    } else {
        Serial.printf("Firebase read error during init: %s\n", fbdo.errorReason().c_str());
    }
    lastActivityTime = millis();
}

// Fungsi untuk mengecek and mencoba koneksi ulang WiFi setiap 10 detik
void checkInternetReconnect() {
    unsigned long currentMillis = millis();
    if(currentMillis - lastWifiCheckTime >= wifiReconnectInterval) {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        WiFi.reconnect();
        lastWifiCheckTime = currentMillis;

        // Jika koneksi berhasil, inisialisasi dependency internet kembali
        if(WiFi.status() == WL_CONNECTED) {
            Serial.println("WiFi reconnected.");
            internetDependenciesReady = false;
            initInternetDependencies();
        }
    }
}

// Fungsi untuk mengirim data ke Firebase
void sendToFirebase(float data, const char* path) {
    if (Firebase.RTDB.setFloat(&fbdo, path, data)) {
        Serial.printf("Firebase: sent %.1f to %s OK\n", data, path);
        lastUploadTime = getTimestamp();  // update waktu upload
        lastTxTime = millis();
    } else {
        Serial.printf("Firebase: ERROR setting %s: %s\n", path, fbdo.errorReason().c_str());
    }
}

void checkButtons() {
    unsigned long now = millis();
    // --- Tombol Pompa 1 ---
    bool currentPump1 = digitalRead(pump1BtnPin);
    if (currentPump1 != pump1ButtonPrev) {
        unsigned long now = millis();
        if(now - lastPump1DebounceTime >= debounceDelay) {
            lastPump1DebounceTime = now;
            // Tombol ditekan (LOW) untuk mengaktifkan pompa
            if (currentPump1 == LOW) { 
                Serial.println("Debug: Pump1 button toggled to LOW");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump1")) {
                        bool pumpStatus = fbdo.boolData();
                        if (!pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump1", true)) {
                                pump1ButtonControl = true;
                                pump1State = true;
                                Serial.println("Pump1 turned ON by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump1 already ON (remote control), ignoring LOW toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump1: %s\n", fbdo.errorReason().c_str());
                    }
                } else { // Offline mode
                    if (!pump1State) {
                        pump1State = true;
                        pump1ButtonControl = true;
                        Serial.println("Local: Pump1 turned ON by button");
                    }
                }
            } 
            // Tombol dilepas (HIGH) untuk mematikan pompa
            else { 
                Serial.println("Debug: Pump1 button toggled to HIGH");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump1")) {
                        bool pumpStatus = fbdo.boolData();
                        // Ubah: cukup periksa jika aktif, tanpa syarat pump1ButtonControl
                        if (pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump1", false)) {
                                pump1ButtonControl = false;
                                pump1State = false;
                                Serial.println("Pump1 turned OFF by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump1 already OFF, ignoring HIGH toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump1: %s\n", fbdo.errorReason().c_str());
                    }
                } else { // Offline mode
                    if (pump1State) {
                        pump1State = false;
                        pump1ButtonControl = false;
                        Serial.println("Local: Pump1 turned OFF by button");
                    }
                }
            }
        }
        pump1ButtonPrev = currentPump1;
    }
    

    // --- Tombol Pompa 2 ---
    bool currentPump2 = digitalRead(pump2BtnPin);
    if (currentPump2 != pump2ButtonPrev) {
        unsigned long now = millis();
        if(now - lastPump2DebounceTime >= debounceDelay) {
            lastPump2DebounceTime = now;
            if (currentPump2 == LOW) {
                Serial.println("Debug: Pump2 button toggled to LOW");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump2")) {
                        bool pumpStatus = fbdo.boolData();
                        if (!pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump2", true)) {
                                pump2ButtonControl = true;
                                pump2State = true;
                                Serial.println("Pump2 turned ON by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump2 already ON (remote control), ignoring LOW toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump2: %s\n", fbdo.errorReason().c_str());
                    }
                } else {
                    if (!pump2State) {
                        pump2State = true;
                        pump2ButtonControl = true;
                        Serial.println("Local: Pump2 turned ON by button");
                    }
                }
            }
            else {
                Serial.println("Debug: Pump2 button toggled to HIGH");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump2")) {
                        bool pumpStatus = fbdo.boolData();
                        // Sederhanakan: periksa jika aktif saja
                        if (pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump2", false)) {
                                pump2ButtonControl = false;
                                pump2State = false;
                                Serial.println("Pump2 turned OFF by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump2 already OFF, ignoring HIGH toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump2: %s\n", fbdo.errorReason().c_str());
                    }
                }
                else {
                    if (pump2State) {
                        pump2State = false;
                        pump2ButtonControl = false;
                        Serial.println("Local: Pump2 turned OFF by button");
                    }
                }
            }
        }
        pump2ButtonPrev = currentPump2;
    }
    

    // --- Tombol Pompa 3 ---
    bool currentPump3 = digitalRead(pump3BtnPin);
    if (currentPump3 != pump3ButtonPrev) {
        unsigned long now = millis();
        if(now - lastPump3DebounceTime >= debounceDelay) {
            lastPump3DebounceTime = now;
            if (currentPump3 == LOW) {
                Serial.println("Debug: Pump3 button toggled to LOW");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump3")) {
                        bool pumpStatus = fbdo.boolData();
                        if (!pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump3", true)) {
                                pump3ButtonControl = true;
                                pump3State = true;
                                Serial.println("Pump3 turned ON by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump3 already ON (remote control), ignoring LOW toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump3: %s\n", fbdo.errorReason().c_str());
                    }
                } else {
                    if (!pump3State) {
                        pump3State = true;
                        pump3ButtonControl = true;
                        Serial.println("Local: Pump3 turned ON by button");
                    }
                }
            }
            else {
                Serial.println("Debug: Pump3 button toggled to HIGH");
                if(WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
                    if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump3")) {
                        bool pumpStatus = fbdo.boolData();
                        // Sederhanakan: periksa jika aktif saja
                        if (pumpStatus) {
                            if (Firebase.RTDB.setBool(&fbdo, "/Kontrol/pump3", false)) {
                                pump3ButtonControl = false;
                                pump3State = false;
                                Serial.println("Pump3 turned OFF by button (Firebase)");
                            }
                        } else {
                            Serial.println("Debug: Pump3 already OFF, ignoring HIGH toggle");
                        }
                    } else {
                        Serial.printf("Error reading /Kontrol/pump3: %s\n", fbdo.errorReason().c_str());
                    }
                }
                else {
                    if (pump3State) {
                        pump3State = false;
                        pump3ButtonControl = false;
                        Serial.println("Local: Pump3 turned OFF by button");
                    }
                }
            }
        }
        pump3ButtonPrev = currentPump3;
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
    bool pump1Status = pump1State;
    bool pump2Status = pump2State;
    bool pump3Status = pump3State;
    
    // Hanya update dari Firebase jika koneksi benar-benar stabil
    if (WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
        if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump1")) {
            pump1Status = fbdo.boolData();
            pump1State = pump1Status;
        }
        if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump2")) {
            pump2Status = fbdo.boolData();
            pump2State = pump2Status;
        }
        if (Firebase.RTDB.getBool(&fbdo, "/Kontrol/pump3")) {
            pump3Status = fbdo.boolData();
            pump3State = pump3Status;
        }
    }
    
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

String getTimestamp() {
    struct tm timeinfo;
    unsigned long start = millis();
    while(!getLocalTime(&timeinfo) && (millis() - start < 5000)){
        delay(100);
    }
    if (!getLocalTime(&timeinfo)) {
        return "0000-00-00-00_00_00";
    }
    char buffer[32];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H_%M_%S", &timeinfo);
    return String(buffer);
}


// Fungsi untuk melakukan kontrol pintu berdasarkan nilai Firebase
void checkDoorControl() {
  // Jika update manual masih pending, cek apakah Firebase sudah mencerminkan nilai baru
  if (pendingFirebaseUpdate) {
    if (WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
      if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PintuAir")) {
         int fbDoor = fbdo.intData();
         if (fbDoor == doorPosition) {
            pendingFirebaseUpdate = false;
            Serial.println("Pending update cleared; Firebase doorPosition confirmed.");
         }
      }
    }
    return; // jangan lakukan sinkronisasi jika update masih pending
  }

  // Abaikan sinkronisasi jika baru selesai manual
  if (millis() - lastManualMoveTime < manualSyncDelay)
      return;

  if (!doorMoving && !manualStepping &&
      WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
    if (Firebase.RTDB.getInt(&fbdo, "/Kontrol/PintuAir")) {
      int tgtPct = fbdo.intData();
      if (tgtPct != doorPosition) {
        digitalWrite(STEPPER_ENABLE_PIN, LOW);
        doorMoving      = true;
        doorTargetSteps = long(tgtPct * stepperTotalSteps / 100.0);
        stepDirection   = (doorTargetSteps > stepPosition) ? +1 : -1;
        lastStepTimeUs  = micros();
      }
    }
  }
}


// --- Fungsi checkManualDoorControl yang telah diupdate ---
void checkManualDoorControl() {
  bool btnO = digitalRead(BTN_OPEN) == LOW;
  bool btnC = digitalRead(BTN_CLOSE) == LOW;

  // Jika tombol OPEN ditekan dan pintu belum mencapai 100%
  if (btnO && !manualStepping && doorPosition < 100) {
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    manualStepping = true;
    stepDirection  = +1;
    lastStepTimeUs = micros();
  }
  // Jika tombol CLOSE ditekan dan pintu belum mencapai 0%
  else if (btnC && !manualStepping && doorPosition > 0) {
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    manualStepping = true;
    stepDirection  = -1;
    lastStepTimeUs = micros();
  }
  // Saat tombol dilepas (kedua tombol HIGH) dan manualStepping aktif, hentikan manual
  else if (!btnO && !btnC && manualStepping) {
    manualStepping = false;
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    lastManualMoveTime = millis(); // Catat waktu selesai manual
    // Update Firebase dengan nilai pintu baru
    if (WiFi.status() == WL_CONNECTED && internetDependenciesReady) {
      if (Firebase.RTDB.setInt(&fbdo, "/Kontrol/PintuAir", doorPosition)) {
         lastUploadTime = getTimestamp();
         pendingFirebaseUpdate = true;  // Set flag update pending
         Serial.println("Firebase updated with manual doorPosition");
      } else {
         Serial.printf("Firebase update failed: %s\n", fbdo.errorReason().c_str());
      }
    }
  }
}

// Fungsi untuk perbaharui status lampu indikator
void checkLampIndicators() {
    bool errorFlag = false;

    // Kondisi error: WiFi atau dependency internet tidak siap
    if (WiFi.status() != WL_CONNECTED || !internetDependenciesReady) {
        errorFlag = true;
    }
    // Kondisi error: pintu sudah mencapai batas dan tombol ditekan
    if ((doorPosition == 0 && digitalRead(BTN_CLOSE) == LOW) ||
        (doorPosition == 100 && digitalRead(BTN_OPEN) == LOW)) {
        errorFlag = true;
    }
    // Tambahkan kondisi error lain jika diperlukan, misal pembacaan komponen gagal atau error internal

    // Jika terjadi error, nyalakan lampu merah, matikan lampu hijau
    if (errorFlag) {
        digitalWrite(LAMP_GREEN_PIN, LOW);
        digitalWrite(LAMP_RED_PIN, HIGH);
    }
    // Jika sistem normal, nyalakan lampu hijau, matikan lampu merah
    else {
        digitalWrite(LAMP_GREEN_PIN, HIGH);
        digitalWrite(LAMP_RED_PIN, LOW);
    }
}

// Fungsi bantu untuk menampilkan waktu boot (fix sejak ESP32 menyala)
String getBootDisplay() {
    return bootTime;
}

void updateLCD() {
    // Ambil timestamp saat ini untuk baris 1 (jam terkini)
    String timeStr = getTimestamp();
    String currentTime = "";
    if(timeStr.length() >= 19) {
        currentTime = timeStr.substring(11, 19);
    }

    // Baris 0: Status pintu
    String line0 = "Pintu: " + String(doorPosition) + "%";
    if(line0.length() < 20) {
        line0 += repeatChar(' ', 20 - line0.length());
    }
    
    // Baris 1: Jam terkini
    String line1 = "Jam:" + currentTime;
    if(line1.length() < 20) {
        line1 += repeatChar(' ', 20 - line1.length());
    }
    
    // Baris 2: Tampilan waktu boot fix (hari, jam:menit)
    String line2 = "Boot:" + getBootDisplay();
    if(line2.length() < 20) {
        line2 += repeatChar(' ', 20 - line2.length());
    } else if(line2.length() > 20) {
        line2 = line2.substring(0,20);
    }
    
    // Baris 3: Tampilan kondisi pompa
    int pumpCount = 0;
    if(pump1State) pumpCount++;
    if(pump2State) pumpCount++;
    if(pump3State) pumpCount++;
    
    String line3;
    if(pumpCount == 0) {
        line3 = "Kondisi: Aman";
    } else if(pumpCount == 1) {
        line3 = "Kondisi: Siaga 1";
    } else if(pumpCount == 2) {
        line3 = "Kondisi: Siaga 2";
    } else if(pumpCount == 3) {
        line3 = "Kondisi: Siaga 3";
    }
    if(line3.length() < 20) {
        line3 += repeatChar(' ', 20 - line3.length());
    } else if(line3.length() > 20) {
        line3 = line3.substring(0,20);
    }
    
    lcd.setCursor(0, 0);
    lcd.print(line0);
    
    lcd.setCursor(0, 1);
    lcd.print(line1);
    
    lcd.setCursor(0, 2);
    lcd.print(line2);
    
    lcd.setCursor(0, 3);
    lcd.print(line3);
}

void i2cScanner() {
    byte error, address;
    int nDevices = 0;
    
    Serial.println("Scanning I2C bus...");
    
    for(address = 1; address < 127; address++ ) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
  
        if (error == 0) {
            Serial.print("I2C device found at address 0x");
            if (address < 16) Serial.print("0");
            Serial.print(address, HEX);
            Serial.println(" !");
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("I2C scan complete\n");
}

String repeatChar(char c, int count) {
    String result = "";
    for (int i = 0; i < count; i++) {
        result += c;
    }
    return result;
}

// … ganti serviceStepper() lengkap dengan debug per-step:
void serviceStepper() {
  if (!doorMoving && !manualStepping) return;

  // STOP jika manual mencapai limit di arah yang salah
  if (manualStepping &&
      ((stepDirection > 0 && doorPosition >= 100) ||
       (stepDirection < 0 && doorPosition <=   0))) {
    manualStepping = false;
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    return;
  }

  unsigned long now = micros();
  if (now - lastStepTimeUs < stepIntervalUs) return;
  lastStepTimeUs = now;

  // arah & pulse
  digitalWrite(STEPPER_DIR_PIN, stepDirection > 0 ? HIGH : LOW);
  digitalWrite(STEPPER_STEP_PIN, HIGH);
  delayMicroseconds(pulseWidthUs);
  digitalWrite(STEPPER_STEP_PIN, LOW);

  // update counter langkah
  stepPosition += stepDirection;
  // clamp agar tidak overshoot
  if (stepPosition < 0) stepPosition = 0;
  if (stepPosition > stepperTotalSteps) stepPosition = stepperTotalSteps;
  // hitung persen pintu
  doorPosition = constrain(int(stepPosition * 100 / stepperTotalSteps), 0, 100);

  // --- AUTO-STOP LOGIC ---
  if (doorMoving &&
      ((stepDirection > 0 && stepPosition >= doorTargetSteps) ||
       (stepDirection < 0 && stepPosition <= doorTargetSteps))) {
    doorMoving = false;
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
    // tulis kembali posisi akhir ke Firebase
    if (WiFi.status()==WL_CONNECTED && internetDependenciesReady) {
      Firebase.RTDB.setInt(&fbdo, "/Kontrol/PintuAir", doorPosition);
      lastUploadTime = getTimestamp();
    }
  }
}

// Fungsi debug: gerakkan stepper sejumlah langkah ke depan & belakang
void debugStepper(int steps, int direction, unsigned long intervalUs) {
    Serial.printf(">> Debug Stepper: %d steps, dir=%s, interval=%lums\n",
                  steps, direction>0?"CW":"CCW", intervalUs/1000);
    // Aktifkan driver
    digitalWrite(STEPPER_ENABLE_PIN, LOW);
    // Set arah
    digitalWrite(STEPPER_DIR_PIN, direction>0 ? HIGH : LOW);
    // Loop pulse
    for (int i = 0; i < steps; i++) {
        digitalWrite(STEPPER_STEP_PIN, HIGH);
        delayMicroseconds(pulseWidthUs);
        digitalWrite(STEPPER_STEP_PIN, LOW);
        delayMicroseconds(intervalUs - pulseWidthUs);
        if (i % 20 == 0) Serial.print('.');  // tanda progress
    }
    Serial.println(" DONE");
    // Non-aktifkan driver
    digitalWrite(STEPPER_ENABLE_PIN, HIGH);
}

// Global: waktu terakhir ESP direset (dalam millis)
unsigned long lastResetTime = 0;
// Interval auto reset (misal 24 jam = 86,400,000 ms)
const unsigned long autoResetInterval = 86400000;

// Fungsi untuk cek jika sudah waktunya auto reset
void checkAutoReset() {
    if(millis() - lastResetTime > autoResetInterval) {
        Serial.println("Auto reset triggered.");
        ESP.restart();
    }
}
