#include <Arduino.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <time.h>

//--------------------
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

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig configFirebase;



//--------------------
// WiFi & NTP setup
void initWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  
  // Waktu disesuaikan ke WIB (UTC+7)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
}

//--------------------
//=== Sensor Ultrasonik ===
// Sensor 1 (HardwareSerial2)
HardwareSerial sensor1(2);
const int s1_RX = 16; // ESP32 menerima data dari sensor 1
const int s1_TX = 17; // ESP32 mengirim data ke sensor 1

// Sensor 2 (HardwareSerial1)
HardwareSerial sensor2(1);
const int s2_RX = 4;  // ESP32 menerima data dari sensor 2
const int s2_TX = 15; // ESP32 mengirim data ke sensor 2

// Sensor 3 (SoftwareSerial)
SoftwareSerial sensor3(18, 19); // Format: (RX, TX)
// ESP32: TX sensor3 → pin 19, RX sensor3 → pin 18

// Variabel global hasil ultrasonik
float avgUS1 = -1, avgUS2 = -1, avgUS3 = -1;

// Fungsi generik membaca sensor ultrasonik dengan timeout 100ms
bool readUltrasonic(Stream &stream, float &dist) {
  while (stream.available()) stream.read();
  
  uint8_t buf[4] = {0};
  unsigned long t0 = millis();
  
  // Tunggu header 0xFF dengan timeout 100ms
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
  
  // Baca byte 1-3 dengan timeout masing-masing 100ms
  for (int i = 1; i < 4; i++) {
    unsigned long t1 = millis();
    while (!stream.available()) {
      if (millis() - t1 > 100) return false;
    }
    buf[i] = stream.read();
  }
  
  // Validasi checksum
  if ((((buf[0] + buf[1] + buf[2]) & 0xFF) != buf[3])) return false;
  
  // Hitung jarak (cm)
  // dist = ((buf[1] << 8) | buf[2]) / 10.0;
  // Hitung jarak (mm)
  dist = ((buf[1] << 8) | buf[2]) ;
  return true;
}

// Inisialisasi sensor ultrasonik
void initUltrasonicSensors() {
  sensor1.begin(9600, SERIAL_8N1, s1_RX, s1_TX);
  sensor2.begin(9600, SERIAL_8N1, s2_RX, s2_TX);
  sensor3.begin(9600);
  Serial.println("Ultrasonic sensors initialized.");
}

//--------------------
//=== Flow Water Sensor Section ===
// Definisi pin untuk masing-masing flow sensor
#define FLOW_SENSOR1_PIN 27  // Flow Sensor 1
#define FLOW_SENSOR2_PIN 26  // Flow Sensor 2
#define FLOW_SENSOR3_PIN 25  // Flow Sensor 3

// Calibration factor (sesuai sensor)
float calibrationFactor = 4.5;

// ----- FLOW SENSOR CALIBRATION -----
// Catatan kalibrasi (data 5 detik):
// Debit Cipalasari: 208202 pulsa -> 700 ml air
//   => calibrationFactor1 = (700 / 208202) * (60/1000)
//                      ≈ 0.00020184
//
// Debit Hulu: 148 pulsa -> 680 ml air
//   => calibrationFactor2 = (680 / 148) * (60/1000)
//                      ≈ 0.27568
//
// Debit Hilir: 357 pulsa -> 800 ml air
//   => calibrationFactor3 = (800 / 357) * (60/1000)
//                      ≈ 0.1344
float calibrationFactor1 = 0.020184;  // untuk Debit Cipalasari
float calibrationFactor2 = 0.27568;     // untuk Debit Hulu
float calibrationFactor3 = 0.1344;      // untuk Debit Hilir

// Variabel global flow sensor
volatile unsigned long pulseCount1 = 0;
volatile unsigned long pulseCount2 = 0;
volatile unsigned long pulseCount3 = 0;

float flowRate1 = 0, flowRate2 = 0, flowRate3 = 0;   // L/min
unsigned int flowMilliLitres1 = 0, flowMilliLitres2 = 0, flowMilliLitres3 = 0;
unsigned long totalMilliLitres1 = 0, totalMilliLitres2 = 0, totalMilliLitres3 = 0;

unsigned long previousFlowMillis = 0;
const int flowInterval = 1000;  // 1 detik

// ISR untuk flow sensor
void IRAM_ATTR pulseCounter1() { pulseCount1++; }
void IRAM_ATTR pulseCounter2() { pulseCount2++; }
void IRAM_ATTR pulseCounter3() { pulseCount3++; }

// Fungsi inisialisasi flow sensor
void initFlowSensors() {
  pinMode(FLOW_SENSOR1_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR2_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR3_PIN, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR1_PIN), pulseCounter1, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR2_PIN), pulseCounter2, FALLING);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR3_PIN), pulseCounter3, FALLING);
  previousFlowMillis = millis();
  Serial.println("Flow sensors initialized.");
}

// Fungsi membaca flow sensor selama 1 detik
void readFlowSensors() {
  unsigned long currentMillisFlow = millis();
  if (currentMillisFlow - previousFlowMillis >= flowInterval) {
    // Salin nilai pulse secara atomik dan reset pulseCount tiap sensor
    noInterrupts();
    unsigned long pulses1 = pulseCount1; pulseCount1 = 0;
    unsigned long pulses2 = pulseCount2; pulseCount2 = 0;
    unsigned long pulses3 = pulseCount3; pulseCount3 = 0;
    interrupts();
    
    // Perhitungan flowRate L/min untuk tiap sensor:
// Rumus: flowRate = (pulses * calibrationFactor_i)
// karena deltaT = 1000 ms (1 detik)
// Selanjutnya, volume (mL per detik) = (flowRate / 60) * 1000 = pulses * calibrationFactor_i * (1000/60)
    flowRate1 = pulses1 * calibrationFactor1;
    flowRate2 = pulses2 * calibrationFactor2;
    flowRate3 = pulses3 * calibrationFactor3;
    
    flowMilliLitres1 = (flowRate1 / 60.0) * 1000;
    flowMilliLitres2 = (flowRate2 / 60.0) * 1000;
    flowMilliLitres3 = (flowRate3 / 60.0) * 1000;
    
    totalMilliLitres1 += flowMilliLitres1;
    totalMilliLitres2 += flowMilliLitres2;
    totalMilliLitres3 += flowMilliLitres3;
    
    previousFlowMillis = currentMillisFlow;
  }
}

//--------------------
// Deklarasi prototype fungsi median sebelum digunakan
float getMedianUltrasonicReading(Stream &stream, int numSamples = 5);


float getMedianUltrasonicReading(Stream &stream, int numSamples) {
    float samples[numSamples];
    int count = 0;
    // Tunggu maksimal 200 ms untuk mendapatkan numSamples valid
    unsigned long start = millis();
    while (count < numSamples && (millis() - start < 200)) {
        if (stream.available()) {
            float d;
            if (readUltrasonic(stream, d)) {
                samples[count++] = d;
            }
        }
        yield(); // non-blocking, memberi kesempatan task background
    }
    if (count == 0) return -1;
    
    // Sorting (bubble sort sederhana)
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (samples[i] > samples[j]) {
                float temp = samples[i];
                samples[i] = samples[j];
                samples[j] = temp;
            }
        }
    }
    
    return samples[count / 2];
}

// Misalnya, definisikan baseline sensor (dalam mm)
const float baseTMA_Citarum = 325;      // untuk ultrasonik sensor 1
const float baseTMA_Kolam = 328;      // untuk ultrasonik sensor 2
const float baseTMA_Cipalasari = 180;   // untuk ultrasonik sensor 3

// --------------------
// Fungsi membaca semua sensor selama 1 detik tanpa delay blocking
void readAllSensorsOneSecond() {
    unsigned long startTime = millis();
    
    int sampleCount1 = 0, sampleCount2 = 0, sampleCount3 = 0;
    float sum1 = 0, sum2 = 0, sum3 = 0;
    
    // Kumpulkan pembacaan selama 1 detik
    while (millis() - startTime < 1000) {
        float reading1 = getMedianUltrasonicReading(sensor1, 3);
        float reading2 = getMedianUltrasonicReading(sensor2, 3);
        float reading3 = getMedianUltrasonicReading(sensor3, 3);
        
        if (reading1 != -1) { sum1 += reading1; sampleCount1++; }
        if (reading2 != -1) { sum2 += reading2; sampleCount2++; }
        if (reading3 != -1) { sum3 += reading3; sampleCount3++; }
        
        yield(); // hindari blocking
    }
    
    // Hitung rata-rata pembacaan (hasil dalam mm)
    float newUS1 = (sampleCount1 > 0) ? (sum1 / sampleCount1) : -1;
    float newUS2 = (sampleCount2 > 0) ? (sum2 / sampleCount2) : -1;
    float newUS3 = (sampleCount3 > 0) ? (sum3 / sampleCount3) : -1;
    
    // Konversikan pembacaan (jarak sensor ke air) menjadi water level (ketinggian air)
    // Water level = baseline - pembacaan, dimana baseline sudah dalam mm
    float waterLevel1 = (newUS1 != -1) ? (baseTMA_Citarum - newUS1) : -1;
    float waterLevel2 = (newUS2 != -1) ? (baseTMA_Kolam - newUS2) : -1;
    float waterLevel3 = (newUS3 != -1) ? (baseTMA_Cipalasari - newUS3) : -1;
    
    // Terapkan exponential smoothing
    const float alpha = 0.1;
    avgUS1 = (avgUS1 < 0) ? waterLevel1 : (alpha * waterLevel1 + (1 - alpha) * avgUS1);
    avgUS2 = (avgUS2 < 0) ? waterLevel2 : (alpha * waterLevel2 + (1 - alpha) * avgUS2);
    avgUS3 = (avgUS3 < 0) ? waterLevel3 : (alpha * waterLevel3 + (1 - alpha) * avgUS3);
    
    // Baca flow sensor tidak menggunakan delay (menggunakan ISR)
    readFlowSensors();
}

// --------------------
// Fungsi mengirim data ke Firebase secara terpisah
void sendFirebaseData() {
  // Dapatkan waktu NTP
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char timeString[25];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d-%H_%M_%S", timeinfo);
  String timestampKey = String(timeString);  // Key unik untuk pengiriman
  
  String basePath = "/Polder/";
  String currentPath;
  bool success = true;
  
  // Sebelum dikirim, konversikan water level dari mm ke cm,
  // dan bulatkan ke dua angka di belakang koma menggunakan roundf()
  if (avgUS1 != -1) {
    float level_cm = roundf((avgUS1 / 10.0) * 100) / 100.0;
    currentPath = basePath + "TMA_Citarum/" + timestampKey;
    if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), level_cm)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
    }
  }
  
  if (avgUS2 != -1) {
    float level_cm = roundf((avgUS2 / 10.0) * 100) / 100.0;
    currentPath = basePath + "TMA_Kolam/" + timestampKey;
    if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), level_cm)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
    }
  }
  
  if (avgUS3 != -1) {
    float level_cm = roundf((avgUS3 / 10.0) * 100) / 100.0;
    currentPath = basePath + "TMA_Cipalasari/" + timestampKey;
    if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), level_cm)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
    }
  }
  
  // Kirim data flow sensor secara individual (tidak diubah)
  currentPath = basePath + "Debit_Cipalasari/" + timestampKey;
  if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), flowRate1)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
  }
  
  currentPath = basePath + "Debit_Hulu/" + timestampKey;
  if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), flowRate2)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
  }
  
  currentPath = basePath + "Debit_Hilir/" + timestampKey;
  if (!Firebase.RTDB.setFloat(&fbdo, currentPath.c_str(), flowRate3)) {
      Serial.println("Firebase setFloat failed for " + currentPath + ": " + fbdo.errorReason());
      success = false;
  }
  
  if (success)
    Serial.println("Firebase: All sensor data sent successfully. Timestamp: " + timestampKey);
  else
    Serial.println("Firebase: One or more sensor data failed to send. Timestamp: " + timestampKey);
}

// Interval Firebase (2 detik)
unsigned long previousFirebaseMillis = 0;
const int firebaseInterval = 3000;

// Tambahkan variabel global untuk interval tampilan pulsa (5 detik)
unsigned long previousPulseDisplayMillis = 0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address: " + WiFi.macAddress());

  initWiFi();
  
  // Konfigurasi Firebase
  configFirebase.api_key = Web_API_KEY;
  configFirebase.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASS;
  Firebase.begin(&configFirebase, &auth);
  Firebase.reconnectWiFi(true);
  
  // Sinkronisasi waktu ke WIB (UTC+7)
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  
  initUltrasonicSensors();
  initFlowSensors();
  
  Serial.println("System initialized.");
  previousFirebaseMillis = millis();
}

void loop() {
  // Baca semua sensor selama 1 detik
  readAllSensorsOneSecond();
  
  // Setiap firebaseInterval (3 detik), kirim data ke Firebase
  if (millis() - previousFirebaseMillis >= firebaseInterval) {
    sendFirebaseData();
    previousFirebaseMillis = millis();
  }
  
  // Tampilkan data secara lokal via Serial
  Serial.println("==== Combined Sensor Readings ====");
  Serial.print("TMA_Citarum: "); Serial.print(avgUS1); Serial.println(" cm");
  Serial.print("TMA_Kolam: "); Serial.print(avgUS2); Serial.println(" cm");
  Serial.print("TMA_Cipalasari: "); Serial.print(avgUS3); Serial.println(" cm");

  Serial.print("Flow Sensor 1 (Debit_Cipalasari): Rate = "); Serial.print(flowRate1); Serial.println(" L/min");
  Serial.print("Flow Sensor 2 (Debit_Hulu): Rate = "); Serial.print(flowRate2); Serial.println(" L/min");
  Serial.print("Flow Sensor 3 (Debit_Hilir): Rate = "); Serial.print(flowRate3); Serial.println(" L/min");
  Serial.println("==================================\n");
  
}




