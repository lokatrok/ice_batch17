// ======== LIBRARY & PIN SETUP ========
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>

#define TEMP_SENSOR_PIN     32
#define VALVE_DRAIN_PIN     33
#define VALVE_INLET_PIN     26
#define FLOAT_SENSOR_PIN    5
#define FLOW_SENSOR_PIN     14
#define TDS_SENSOR_PIN      34
#define COMPRESSOR_PIN      25
#define PUMP_UV_PIN         27
#define RTC_SDA_PIN         21
#define RTC_SCL_PIN         22

// ======== GLOBAL VARIABLES ========
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
RTC_DS3231 rtc;

// Variabel sensor
float currentTemp = 0.0;
float tdsValue = 0.0;
volatile unsigned long pulseCount = 0;

// Variabel kontrol
bool fillingActive = false;
bool drainingActive = false;
bool sistemAktif = false;
bool statusKompresor = false;
int suhuTarget = 29;
int activeProcess = 0;
bool debug = true;

// Variabel mode auto
bool autoMode = false;
bool circulationActive = false;
bool manualCirculationOn = false;  // Flag untuk menandai sirkulasi diaktifkan manual
bool manualCirculationOff = false; // Tambahkan ini untuk menandai sirkulasi dimatikan manual
int autoHour1 = 0;
int autoHour2 = 0;
int autoMin1 = 0;
int autoMin2 = 0;
int autoTempTarget = 3;
int autoChangeDay = 7;
unsigned long lastAutoCheck = 0;
unsigned long autoCheckInterval = 60000; // Cek setiap menit
DateTime lastChangeDate; // Tanggal pergantian air terakhir
bool waterChangeScheduled = false; // Flag untuk pergantian air terjadwal
bool preFillScheduled = false; // Flag untuk pengisian sebelum operasi
bool preFillExecuted = false;
DateTime lastPreFillDate; // Tanggal terakhir pre-fill dieksekusi

// Variabel untuk menerima parameter dari Nextion
bool receivingAutoParams = false;
int paramIndex = 0;
String autoParams[6]; // Untuk menyimpan jam, menit, suhu, hari

// Variabel filling
bool fillingStarted = false;
bool fillingStoppedBySensor = false;
unsigned long drainStartTime = 0;
unsigned long lastFillingDebug = 0;
bool lastFloatState = HIGH;
unsigned long lastFloatTriggerTime = 0; // Tambah ini
bool floatSensorStable = false; // Tambah ini
int fillingStage = 0; // 0=idle, 1=draining, 2=filling
const unsigned long FILLING_TIMEOUT = 600000; // 10 menit timeout

// Variabel kontrol suhu
const int histeresis = 2;
bool initialCoolingMode = false;
bool targetReached = false;
unsigned long waktuTerakhirCek = 0;
unsigned long intervalCek = 3000;

// Variabel draining
bool graceActive = false;
unsigned long graceStart = 0;
bool flowOK = false;
unsigned long lastFlowCheckTime = 0;
const unsigned long GRACE_PERIOD = 5000; // 5 detik grace period
int lowFlowCount = 0; // Counter untuk mendeteksi flow rate rendah secara konsisten

// Variabel timing
unsigned long lastSensorUpdate = 0;
unsigned long intervalSensorUpdate = 2000;

// Variabel komunikasi Nextion
String cmd = "";
int ffCount = 0;
unsigned long lastNextionRead = 0;
unsigned long intervalNextionRead = 1000;
int cachedActiveProcess = -1;
int cachedSuhuTarget = -1;
int lastValidTarget = 29;
unsigned long lastTargetChange = 0;
const unsigned long TARGET_CHANGE_DEBOUNCE = 3000;

// Di ice_bath9.ino - setelah include
// Deklarasi fungsi
void setupSensors();
void readSensors();
void processManualSystem();
void handleNextionCommunication();
void processAutoSystem();
void setupRTC();
void readAutoSettings();
void checkAutoSchedule();
void startWaterChange();
void startPreFill();
void startCirculation();
void stopCirculation();
void kontrolSuhuAuto();
void kontrolSuhuManual();
void processFilling();
void processDraining();
void IRAM_ATTR flowISR();
void sendFF();
void updateNextionDisplay();
void stopFillingBySensor();
void stopFillingManual();
void stopDraining();
int bacaKomponenWaktu(String namaKomponen, int minVal, int maxVal);
String bacaTeksDariNextion(String namaKomponen);
void bacaParameterDariNextion();
void processAutoParameters();
void readNextion();
void processNextionCommand(String command);

// ======== SETUP ========
// Di ice_bath7.ino - setup()
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  // Reset semua state filling
  fillingActive = false;
  fillingStarted = false;
  fillingStage = 0;
  lastFloatTriggerTime = 0;
  floatSensorStable = false;
  lastFloatState = HIGH;
  
  setupSensors();
  setupRTC();


  // Test hardware
  Serial.println("=== TESTING HARDWARE ===");
  
  // Test valve drain
  Serial.println("Testing DRAIN valve...");
  digitalWrite(VALVE_DRAIN_PIN, HIGH);
  delay(1000);
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  Serial.println("DRAIN valve test complete");
  
  // Test valve inlet
  Serial.println("Testing INLET valve...");
  digitalWrite(VALVE_INLET_PIN, HIGH);
  delay(1000);
  digitalWrite(VALVE_INLET_PIN, LOW);
  Serial.println("INLET valve test complete");
  
  // Test float sensor
  Serial.println("Testing FLOAT sensor...");
  bool floatState = digitalRead(FLOAT_SENSOR_PIN);
  Serial.print("Float sensor state: ");
  Serial.println(floatState == LOW ? "TRIGGERED" : "NORMAL");
  
  Serial.println("=== TESTING COMPLETE ===");
  
  if (debug) {
    Serial.println("=== SISTEM KONTROL TANK TERINTEGRASI ===");
    Serial.println("Sistem siap...");
    Serial.println("Send 'f' to start filling, 's' to stop");
  }
}

// Di ice_bath5.ino - loop()
void loop() {
  handleNextionCommunication();
  
  // Update sensor dengan interval
  if (millis() - lastSensorUpdate >= intervalSensorUpdate) {
    readSensors();
    lastSensorUpdate = millis();
  }
  
  // Jalankan sistem manual atau auto
  if (autoMode) {
    processAutoSystem();
  } else {
    processManualSystem();
  }

  // PASTIKAN INI ADA: Panggil processFilling() jika fillingActive
  if (fillingActive) {
    Serial.println(">> DEBUG: Calling processFilling()");
    processFilling();
  }
  
  // Cek timeout untuk penerimaan parameter auto
  if (receivingAutoParams && (millis() - lastAutoCheck > 5000)) {
    if (debug) Serial.println(">> Auto parameter timeout - using defaults");
    
    // Set parameter default
    autoHour1 = 1;
    autoHour2 = 1;
    autoMin1 = 0;
    autoMin2 = 0;
    autoTempTarget = 3;
    autoChangeDay = 7;
    
    // Reset flag penerimaan parameter
    receivingAutoParams = false;
    paramIndex = 0;
    
    if (debug) {
      Serial.print(">> Default Auto Settings - Time: ");
      Serial.print(autoHour1);
      Serial.print(autoHour2);
      Serial.print(":");
      Serial.print(autoMin1);
      Serial.print(autoMin2);
      Serial.print(" | Temp: ");
      Serial.print(autoTempTarget);
      Serial.print("°C | Change every: ");
      Serial.print(autoChangeDay);
      Serial.println(" days");
    }
  }
}