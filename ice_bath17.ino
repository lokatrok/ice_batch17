// ======== ICE BATH SYSTEM V17 - INTEGRATED & OPTIMIZED ========
// Updated to work seamlessly with optimized auto.ino, manual.ino, nextion.ino, and sensors.ino
// This version integrates all system components with proper state management

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <RTClib.h>

// ======== PIN DEFINITIONS ========
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

// ======== HARDWARE INSTANCES ========
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature sensors(&oneWire);
RTC_DS3231 rtc;

// ======== CONSTANTS AND DEFINITIONS ========
// Temperature sensor error constant (compatibility with DallasTemperature)
#define DEVICE_DISCONNECTED_C -127

// Timing constants
const unsigned long autoCheckInterval = 60000;     // 1 minute for auto system checks
const unsigned long intervalSensorUpdate = 2000;  // 2 seconds for sensor updates
const unsigned long intervalNextionRead = 1000;   // 1 second for Nextion communication

// ======== STRUCT DEFINITIONS (Compatible with other files) ========
struct TimeInfo {
  int hour;
  int minute;
  int totalMinutes;
};

struct ScheduleInfo {
  TimeInfo current;
  TimeInfo target;
  TimeInfo preFill;
  bool isPreFillToday;
  bool isOperationToday;
  int daysSinceLastChange;
};

// ======== ENUM DEFINITIONS (Add after struct definitions) ========
// Auto System State enum
enum AutoSystemState {
  AUTO_IDLE = 0,
  AUTO_PRE_FILL = 1,
  AUTO_CIRCULATION = 2,
  AUTO_COOLING = 3,
  AUTO_COMPLETED = 4
};

// Schedule Status enum
enum ScheduleStatus {
  SCHEDULE_WAITING = 0,
  SCHEDULE_PRE_FILL_READY = 1,
  SCHEDULE_OPERATION_READY = 2,
  SCHEDULE_COMPLETED = 3
};

// Filling State enum
enum FillingState {
  FILLING_IDLE = 0,
  FILLING_DRAINING = 1,
  FILLING_INLET = 2,
  FILLING_COMPLETED = 3
};

// Temperature Control Mode enum
enum TempControlMode {
  TEMP_OFF = 0,
  TEMP_INITIAL_COOLING = 1,
  TEMP_HYSTERESIS = 2
};

// Nextion Communication State enum
enum NextionCommState {
  NEXTION_IDLE = 0,
  NEXTION_RECEIVING = 1,
  NEXTION_PROCESSING = 2,
  NEXTION_ERROR = 3
};

// Sensor State enum
enum SensorState {
  SENSOR_INITIALIZING = 0,
  SENSOR_NORMAL = 1,
  SENSOR_WARNING = 2,
  SENSOR_ERROR = 3
};
  
// ======== GLOBAL SENSOR VARIABLES ========
float currentTemp = 0.0;
float tdsValue = 0.0;
volatile unsigned long pulseCount = 0;

// ======== SYSTEM STATE VARIABLES ========
// Manual system states
bool fillingActive = false;
bool drainingActive = false;
bool sistemAktif = false;
bool statusKompresor = false;
int suhuTarget = 29;
int activeProcess = 0;
bool debug = true;

// Auto system states (synchronized with auto.ino)
bool autoMode = false;
bool circulationActive = false;
bool manualCirculationOn = false;
bool manualCirculationOff = false;
bool initialCoolingMode = false;
bool targetReached = false;

// Auto system parameters
int autoHour1 = 0;
int autoHour2 = 1;
int autoMin1 = 0;
int autoMin2 = 0;
int autoTempTarget = 3;
int autoChangeDay = 7;

// Auto system schedule tracking
DateTime lastChangeDate;
DateTime lastPreFillDate;
bool waterChangeScheduled = false;
bool preFillScheduled = false;
bool preFillExecuted = false;

// Timing and communication variables
unsigned long lastAutoCheck = 0;
unsigned long lastSensorUpdate = 0;
unsigned long lastNextionRead = 0;

// ======== FILLING SYSTEM VARIABLES (Compatible with manual.ino) ========
bool fillingStarted = false;
bool fillingStoppedBySensor = false;
unsigned long drainStartTime = 0;
unsigned long lastFillingDebug = 0;
bool lastFloatState = HIGH;
unsigned long lastFloatTriggerTime = 0;
bool floatSensorStable = false;
int fillingStage = 0; // 0=idle, 1=draining, 2=filling
const unsigned long FILLING_TIMEOUT = 600000; // 10 minutes timeout

// ======== DRAINING SYSTEM VARIABLES (Compatible with manual.ino) ========
bool graceActive = false;
unsigned long graceStart = 0;
bool flowOK = false;
unsigned long lastFlowCheckTime = 0;
const unsigned long GRACE_PERIOD = 5000;
int lowFlowCount = 0;

// ======== NEXTION COMMUNICATION VARIABLES (Compatible with nextion.ino) ========
bool receivingAutoParams = false;
int paramIndex = 0;
String autoParams[6];
String cmd = "";
int ffCount = 0;
int cachedActiveProcess = -1;
int cachedSuhuTarget = -1;
int lastValidTarget = 29;
unsigned long lastTargetChange = 0;
const unsigned long TARGET_CHANGE_DEBOUNCE = 3000;

// ======== TEMPERATURE CONTROL VARIABLES (Compatible with auto.ino & manual.ino) ========
const int histeresis = 2;
unsigned long waktuTerakhirCek = 0;
unsigned long intervalCek = 3000;

// ======== FORWARD DECLARATIONS ========
// Core system functions
void setupSensors();
void setupRTC();
void readSensors();
void handleNextionCommunication();

// Manual system functions (implemented in this file, compatible with manual.ino)
void processManualSystem();
void processFillingStateMachine();
void processDrainingWithFlowControl();
void controlTemperatureManual();
void stopFillingBySensor();
void stopFillingManual();
void stopDraining();

// Auto system functions (implemented in this file, compatible with auto.ino)
void processAutoSystem();
void readAutoSettings();
void processAutoSchedule();
void startManualCirculation();
void stopCirculation();
void controlTemperatureAuto();

// Nextion functions (implemented in this file, compatible with nextion.ino)
void processNextionInputStream();
bool processNextionCommand(String command);
void readParametersFromNextion();
void updateNextionDisplayOptimized();
int bacaKomponenWaktu(String namaKomponen, int minVal, int maxVal);
void processAutoParameters();

// Sensor functions (implemented in this file, compatible with sensors.ino)
void readTemperatureSensor(unsigned long currentTime);
void readTDSSensor(unsigned long currentTime);
void readFlowSensor(unsigned long currentTime);
void updateClockDisplay(unsigned long currentTime);
void handleSerialCommands();

// Utility functions
void IRAM_ATTR flowISR();
void sendFF();

// ======== SETUP FUNCTION ========
void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  
  if (debug) {
    Serial.println("=== ICE BATH SYSTEM V17 - STARTING ===");
    Serial.println(">> Initializing integrated control system...");
  }
  
  // Initialize all system states to safe defaults
  initializeSystemStates();
  
  // Setup hardware components
  setupSensors();
  setupRTC();
  
  // Perform hardware validation tests
  performHardwareTests();
  
  if (debug) {
    Serial.println("=== SYSTEM READY ===");
    Serial.println(">> Manual Mode: Send 'f' for filling, 'd' for draining, 'c' for cooling");
    Serial.println(">> Auto Mode: Configured via Nextion display");
    Serial.println(">> Time Setting: Send 'SETTIME' to configure RTC");
  }
}

// ======== MAIN LOOP ========
void loop() {
  // Handle Nextion communication with high priority
  handleNextionCommunication();
  
  // Update sensors with controlled intervals
  if (millis() - lastSensorUpdate >= intervalSensorUpdate) {
    readSensors();
    lastSensorUpdate = millis();
  }
  
  // Execute system logic based on current mode
  if (autoMode) {
    processAutoSystem();
  } else {
    processManualSystem();
  }
  
  // Handle filling process if active (critical for both modes)
  if (fillingActive) {
    processFillingStateMachine();
  }
  
  // Handle draining process if active (critical for both modes)
  if (drainingActive) {
    processDrainingWithFlowControl();
  }
  
  // Handle timeout for auto parameter reception
  handleAutoParameterTimeout();
  
  // Handle serial commands for debugging and time setting
  handleSerialCommands();
}

// ======== SYSTEM INITIALIZATION ========
void initializeSystemStates() {
  // Reset all filling states
  fillingActive = false;
  fillingStarted = false;
  fillingStage = 0;
  lastFloatTriggerTime = 0;
  floatSensorStable = false;
  lastFloatState = HIGH;
  
  // Reset all draining states
  drainingActive = false;
  graceActive = false;
  flowOK = false;
  lowFlowCount = 0;
  
  // Reset all auto system states
  autoMode = false;
  circulationActive = false;
  manualCirculationOn = false;
  manualCirculationOff = false;
  preFillScheduled = false;
  preFillExecuted = false;
  waterChangeScheduled = false;
  
  // Reset temperature control
  sistemAktif = false;
  statusKompresor = false;
  initialCoolingMode = false;
  targetReached = false;
  
  // Reset communication states
  receivingAutoParams = false;
  paramIndex = 0;
  cmd = "";
  ffCount = 0;
  
  if (debug) Serial.println(">> System states initialized to safe defaults");
}

void performHardwareTests() {
  if (debug) Serial.println(">> Performing hardware validation tests...");
  
  // Test drain valve
  Serial.println(">> Testing DRAIN valve...");
  digitalWrite(VALVE_DRAIN_PIN, HIGH);
  delay(500);
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  
  // Test inlet valve
  Serial.println(">> Testing INLET valve...");
  digitalWrite(VALVE_INLET_PIN, HIGH);
  delay(500);
  digitalWrite(VALVE_INLET_PIN, LOW);
  
  // Test float sensor
  bool floatState = digitalRead(FLOAT_SENSOR_PIN);
  Serial.print(">> Float sensor state: ");
  Serial.println(floatState == LOW ? "TRIGGERED (Water Present)" : "NORMAL (No Water)");
  
  // Test temperature sensor
  sensors.requestTemperatures();
  delay(750);
  float testTemp = sensors.getTempCByIndex(0);
  if (testTemp != DEVICE_DISCONNECTED_C) {
    Serial.print(">> Temperature sensor: OK (");
    Serial.print(testTemp);
    Serial.println("°C)");
  } else {
    Serial.println(">> Temperature sensor: ERROR - Disconnected!");
  }
  
  Serial.println(">> Hardware tests complete");
}

// ======== AUTO PARAMETER TIMEOUT HANDLER ========
void handleAutoParameterTimeout() {
  if (receivingAutoParams && (millis() - lastAutoCheck > 10000)) {
    if (debug) Serial.println(">> AUTO: Parameter timeout - Using safe defaults");
    
    // Set safe default parameters
    autoHour1 = 0;
    autoHour2 = 1;
    autoMin1 = 0;
    autoMin2 = 0;
    autoTempTarget = 3;
    autoChangeDay = 7;
    
    // Clear parameter reception state
    receivingAutoParams = false;
    paramIndex = 0;
    for (int i = 0; i < 6; i++) {
      autoParams[i] = "";
    }
    
    if (debug) {
      Serial.println(">> AUTO: Default settings applied");
      Serial.print(">>   Schedule: ");
      if ((autoHour1 * 10 + autoHour2) < 10) Serial.print("0");
      Serial.print(autoHour1 * 10 + autoHour2);
      Serial.print(":");
      if ((autoMin1 * 10 + autoMin2) < 10) Serial.print("0");
      Serial.print(autoMin1 * 10 + autoMin2);
      Serial.print(" | Target: ");
      Serial.print(autoTempTarget);
      Serial.print("°C | Interval: ");
      Serial.print(autoChangeDay);
      Serial.println(" days");
    }
  }
}

// ======== INTEGRATED SETUP FUNCTIONS ========
// These functions integrate functionality from sensors.ino and are optimized for this system

void setupSensors() {
  if (debug) Serial.println(">> SENSORS: Initializing integrated sensor system");
  
  // Configure all pins with proper modes
  pinMode(FLOAT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(TDS_SENSOR_PIN, INPUT);
  
  // Configure actuator output pins
  pinMode(VALVE_DRAIN_PIN, OUTPUT);
  pinMode(VALVE_INLET_PIN, OUTPUT);
  pinMode(COMPRESSOR_PIN, OUTPUT);
  pinMode(PUMP_UV_PIN, OUTPUT);
  
  // Initialize all outputs to safe state
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(VALVE_INLET_PIN, LOW);
  digitalWrite(COMPRESSOR_PIN, LOW);
  digitalWrite(PUMP_UV_PIN, LOW);
  
  // Initialize temperature sensor
  sensors.begin();
  if (sensors.getDeviceCount() > 0) {
    sensors.setResolution(12);
    if (debug) Serial.println(">> SENSORS: Temperature sensor initialized (12-bit resolution)");
  } else {
    if (debug) Serial.println(">> SENSORS: WARNING - No temperature sensors detected");
  }
  
  // Setup flow sensor interrupt
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowISR, RISING);
  if (debug) Serial.println(">> SENSORS: Flow sensor interrupt configured");
  
  if (debug) Serial.println(">> SENSORS: All sensors initialized successfully");
}

void setupRTC() {
  if (debug) Serial.println(">> RTC: Initializing Real Time Clock");
  
  // Initialize I2C for RTC
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  
  if (!rtc.begin()) {
    Serial.println(">> RTC: ERROR - Could not find RTC module!");
    while (1) {
      delay(1000); // Halt with watchdog-friendly delay
    }
  }
  
  // Check if RTC lost power and needs time reset
  if (rtc.lostPower()) {
    Serial.println(">> RTC: Power lost - Setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Initialize schedule dates to safe past values
  DateTime safePastDate = DateTime(2020, 1, 1, 0, 0, 0);
  lastChangeDate = safePastDate;
  lastPreFillDate = safePastDate;
  
  // Validate RTC time
  DateTime now = rtc.now();
  if (now.year() >= 2020 && now.year() <= 2030) {
    if (debug) {
      Serial.print(">> RTC: Initialized successfully - Current time: ");
      Serial.print(now.day());
      Serial.print("/");
      Serial.print(now.month());
      Serial.print("/");
      Serial.print(now.year());
      Serial.print(" ");
      Serial.print(now.hour());
      Serial.print(":");
      if (now.minute() < 10) Serial.print("0");
      Serial.println(now.minute());
    }
  } else {
    Serial.println(">> RTC: WARNING - Time appears invalid, please set manually");
  }
}

// ======== INTEGRATED SENSOR READING ========
void readSensors() {
  unsigned long currentTime = millis();
  
  // Read temperature sensor (non-blocking approach)
  static unsigned long lastTempRequest = 0;
  static bool tempRequested = false;
  
  if (!tempRequested) {
    sensors.requestTemperatures();
    tempRequested = true;
    lastTempRequest = currentTime;
  } else if (currentTime - lastTempRequest >= 750) {
    float newTemp = sensors.getTempCByIndex(0);
    if (newTemp != DEVICE_DISCONNECTED_C && newTemp >= -10 && newTemp <= 80) {
      currentTemp = newTemp;
    }
    tempRequested = false;
  }
  
  // Read TDS sensor with temperature compensation
  int rawTDS = analogRead(TDS_SENSOR_PIN);
  float voltage = rawTDS * (3.3 / 4095.0);
  if (voltage >= 0 && voltage <= 3.3) {
    // Apply temperature compensation and convert to TDS
    float compensatedVoltage = voltage * (1.0 + (currentTemp - 25.0) * 0.02);
    float ec = (133.42 * compensatedVoltage * compensatedVoltage * compensatedVoltage 
                - 255.86 * compensatedVoltage * compensatedVoltage 
                + 857.39 * compensatedVoltage);
    tdsValue = ec * 0.5;
    
    // Validate TDS range
    if (tdsValue < 0) tdsValue = 0;
    if (tdsValue > 2000) tdsValue = 2000;
  }
  
  // Update clock display directly here
  static unsigned long lastClockUpdate = 0;
  if (currentTime - lastClockUpdate >= 1000) {
    DateTime now = rtc.now();
    
    // Format and send time to Nextion
    Serial2.print("tClock.txt=\"");
    if (now.hour() < 10) Serial2.print("0");
    Serial2.print(now.hour());
    Serial2.print(":");
    if (now.minute() < 10) Serial2.print("0");
    Serial2.print(now.minute());
    Serial2.print("\"");
    sendFF();
    
    lastClockUpdate = currentTime;
  }
  
  // Debug output (throttled)
  static unsigned long lastSensorDebug = 0;
  if (debug && (currentTime - lastSensorDebug >= 5000)) {
    Serial.print(">> SENSORS: Temp=");
    Serial.print(currentTemp, 1);
    Serial.print("°C | TDS=");
    Serial.print(tdsValue, 1);
    Serial.print(" ppm | Float=");
    Serial.print(digitalRead(FLOAT_SENSOR_PIN) == LOW ? "TRIGGERED" : "NORMAL");
    Serial.print(" | Flow Pulses=");
    Serial.println(pulseCount);
    lastSensorDebug = currentTime;
  }
}

void updateClockDisplay(unsigned long currentTime) {
  // This function is now integrated into readSensors() to avoid duplication
  // Left here for compatibility but not used
}

// ======== FLOW SENSOR INTERRUPT ========
void IRAM_ATTR flowISR() {
  pulseCount++;
}

// ======== NEXTION COMMUNICATION HELPERS ========
void sendFF() {
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// ======== SERIAL COMMAND HANDLER ========
void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readString();
    command.trim();
    command.toLowerCase();
    
    if (command == "f" && !autoMode) {
      // Manual filling command
      if (!fillingActive) {
        Serial.println(">> CMD: Starting manual filling");
        fillingActive = true;
        fillingStarted = false;
        fillingStage = 0;
      } else {
        Serial.println(">> CMD: Filling already active");
      }
    }
    else if (command == "s" && !autoMode) {
      // Stop filling command
      if (fillingActive) {
        Serial.println(">> CMD: Stopping filling");
        stopFillingManual();
      } else {
        Serial.println(">> CMD: Filling not active");
      }
    }
    else if (command == "d" && !autoMode) {
      // Manual draining command
      if (!drainingActive) {
        Serial.println(">> CMD: Starting manual draining");
        drainingActive = true;
        graceActive = false;
        flowOK = false;
        pulseCount = 0;
      } else {
        Serial.println(">> CMD: Draining already active");
      }
    }
    else if (command == "c" && !autoMode) {
      // Toggle cooling command
      if (activeProcess == 1) {
        Serial.println(">> CMD: Stopping cooling");
        activeProcess = 0;
      } else {
        Serial.println(">> CMD: Starting cooling");
        activeProcess = 1;
      }
    }
    else if (command == "settime") {
      Serial.println(">> TIME: Entering time setting mode");
      Serial.println(">> TIME: Send new time in format HH:MM (24-hour format)");
    }
    else if (command.indexOf(":") > 0 && command.length() == 5) {
      // Time setting in HH:MM format
      int colonPos = command.indexOf(":");
      int newHour = command.substring(0, colonPos).toInt();
      int newMinute = command.substring(colonPos + 1).toInt();
      
      if (newHour >= 0 && newHour <= 23 && newMinute >= 0 && newMinute <= 59) {
        DateTime now = rtc.now();
        rtc.adjust(DateTime(now.year(), now.month(), now.day(), newHour, newMinute, 0));
        Serial.print(">> TIME: Time set to ");
        if (newHour < 10) Serial.print("0");
        Serial.print(newHour);
        Serial.print(":");
        if (newMinute < 10) Serial.print("0");
        Serial.println(newMinute);
      } else {
        Serial.println(">> TIME: Invalid time format! Use HH:MM (00-23:00-59)");
      }
    }
    else if (command == "status") {
      // System status command
      printSystemStatus();
    }
    else if (command == "help") {
      // Help command
      printCommandHelp();
    }
  }
}

void printSystemStatus() {
  Serial.println(">> === SYSTEM STATUS ===");
  Serial.print(">> Mode: ");
  Serial.println(autoMode ? "AUTO" : "MANUAL");
  Serial.print(">> Temperature: ");
  Serial.print(currentTemp, 1);
  Serial.println("°C");
  Serial.print(">> TDS: ");
  Serial.print(tdsValue, 1);
  Serial.println(" ppm");
  Serial.print(">> Active Process: ");
  Serial.println(activeProcess);
  Serial.print(">> Filling: ");
  Serial.println(fillingActive ? "ACTIVE" : "INACTIVE");
  Serial.print(">> Draining: ");
  Serial.println(drainingActive ? "ACTIVE" : "INACTIVE");
  Serial.print(">> Circulation: ");
  Serial.println(circulationActive ? "ACTIVE" : "INACTIVE");
  Serial.print(">> Compressor: ");
  Serial.println(statusKompresor ? "ON" : "OFF");
  
  DateTime now = rtc.now();
  Serial.print(">> Current Time: ");
  if (now.hour() < 10) Serial.print("0");
  Serial.print(now.hour());
  Serial.print(":");
  if (now.minute() < 10) Serial.print("0");
  Serial.print(now.minute());
  Serial.print(" ");
  Serial.print(now.day());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.println(now.year());
  
  if (autoMode) {
    Serial.print(">> Auto Schedule: ");
    if ((autoHour1 * 10 + autoHour2) < 10) Serial.print("0");
    Serial.print(autoHour1 * 10 + autoHour2);
    Serial.print(":");
    if ((autoMin1 * 10 + autoMin2) < 10) Serial.print("0");
    Serial.print(autoMin1 * 10 + autoMin2);
    Serial.print(" | Target: ");
    Serial.print(autoTempTarget);
    Serial.print("°C | Days: ");
    Serial.println(autoChangeDay);
  }
  Serial.println(">> === END STATUS ===");
}

void printCommandHelp() {
  Serial.println(">> === AVAILABLE COMMANDS ===");
  Serial.println(">> f - Start filling (manual mode only)");
  Serial.println(">> s - Stop filling (manual mode only)");
  Serial.println(">> d - Start draining (manual mode only)");
  Serial.println(">> c - Toggle cooling (manual mode only)");
  Serial.println(">> settime - Enter time setting mode");
  Serial.println(">> HH:MM - Set time (24-hour format, e.g., 14:30)");
  Serial.println(">> status - Show system status");
  Serial.println(">> help - Show this help");
  Serial.println(">> === Auto mode controlled via Nextion display ===");
}

// ======== MINIMAL FUNCTION IMPLEMENTATIONS ========
// These provide basic functionality compatible with the other optimized files
// Full implementations are in the respective specialized files

void processManualSystem() {
  // Basic manual system processing
  if (autoMode) return; // Only process in manual mode
  
  // Basic temperature control
  controlTemperatureManual();
}

void processAutoSystem() {
  // Basic auto system processing
  if (!autoMode) return; // Only process in auto mode
  
  static unsigned long lastAutoCheck = 0;
  if (millis() - lastAutoCheck >= autoCheckInterval) {
    readAutoSettings();
    processAutoSchedule();
    lastAutoCheck = millis();
  }
  
  // Auto temperature control if circulation active
  if (circulationActive) {
    controlTemperatureAuto();
  }
}

void processFillingStateMachine() {
  // Basic filling state machine
  static unsigned long stateStartTime = 0;
  
  if (!fillingStarted) {
    // Start filling process
    digitalWrite(VALVE_DRAIN_PIN, HIGH);
    digitalWrite(VALVE_INLET_PIN, LOW);
    fillingStarted = true;
    fillingStage = 1;
    stateStartTime = millis();
    if (debug) Serial.println(">> FILLING: Started - Draining phase");
    return;
  }
  
  if (fillingStage == 1 && (millis() - stateStartTime >= 5000)) {
    // Switch to inlet after 5 seconds of draining
    digitalWrite(VALVE_DRAIN_PIN, LOW);
    delay(500);
    digitalWrite(VALVE_INLET_PIN, HIGH);
    fillingStage = 2;
    stateStartTime = millis();
    if (debug) Serial.println(">> FILLING: Switched to inlet mode");
  }
  
  if (fillingStage == 2) {
    // Check float sensor for completion
    if (digitalRead(FLOAT_SENSOR_PIN) == LOW || 
        (millis() - stateStartTime >= FILLING_TIMEOUT)) {
      stopFillingBySensor();
    }
  }
}

void processDrainingWithFlowControl() {
  // Basic draining with flow monitoring
  static unsigned long lastFlowCheck = 0;
  
  if (!graceActive && !flowOK) {
    // Initialize draining
    graceActive = true;
    graceStart = millis();
    digitalWrite(VALVE_DRAIN_PIN, HIGH);
    digitalWrite(PUMP_UV_PIN, HIGH);
    pulseCount = 0;
    if (debug) Serial.println(">> DRAINING: Started with grace period");
  }
  
  if (millis() - lastFlowCheck >= 1000) {
    float flowRate = pulseCount / 7.5; // Convert to L/min
    pulseCount = 0;
    
    if (graceActive && flowRate >= 0.3) {
      flowOK = true;
      graceActive = false;
      lowFlowCount = 0;
      if (debug) Serial.println(">> DRAINING: Flow detected");
    } else if (!graceActive && flowRate < 0.1) {
      lowFlowCount++;
      if (lowFlowCount >= 3) {
        if (debug) Serial.println(">> DRAINING: Low flow - stopping");
        stopDraining();
      }
    } else {
      lowFlowCount = 0;
    }
    
    lastFlowCheck = millis();
  }
  
  // Grace period timeout
  if (graceActive && (millis() - graceStart >= GRACE_PERIOD)) {
    if (!flowOK) {
      if (debug) Serial.println(">> DRAINING: No flow after grace period");
      stopDraining();
    } else {
      graceActive = false;
    }
  }
}

void controlTemperatureManual() {
  static unsigned long lastTempCheck = 0;
  
  if (millis() - lastTempCheck < intervalCek) return;
  lastTempCheck = millis();
  
  if (activeProcess == 1) {
    if (!sistemAktif) {
      sistemAktif = true;
      digitalWrite(PUMP_UV_PIN, HIGH);
      initialCoolingMode = true;
      if (debug) Serial.println(">> TEMP: Manual control activated");
    }
    
    float tempDiff = currentTemp - suhuTarget;
    
    if (initialCoolingMode) {
      if (tempDiff > 0) {
        if (!statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, HIGH);
          statusKompresor = true;
        }
      } else {
        if (statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, LOW);
          statusKompresor = false;
        }
        initialCoolingMode = false;
        targetReached = true;
      }
    } else {
      if (tempDiff > histeresis) {
        if (!statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, HIGH);
          statusKompresor = true;
        }
      } else if (tempDiff < 0) {
        if (statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, LOW);
          statusKompresor = false;
        }
      }
    }
  } else {
    if (sistemAktif) {
      sistemAktif = false;
      digitalWrite(COMPRESSOR_PIN, LOW);
      digitalWrite(PUMP_UV_PIN, LOW);
      statusKompresor = false;
      if (debug) Serial.println(">> TEMP: Manual control deactivated");
    }
  }
}

void controlTemperatureAuto() {
  static unsigned long lastAutoTempCheck = 0;
  
  if (millis() - lastAutoTempCheck < 5000) return;
  lastAutoTempCheck = millis();
  
  if (circulationActive && currentTemp > -50 && currentTemp < 100) {
    float tempDiff = currentTemp - autoTempTarget;
    
    if (initialCoolingMode) {
      if (tempDiff > 0) {
        if (!statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, HIGH);
          statusKompresor = true;
        }
      } else {
        if (statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, LOW);
          statusKompresor = false;
        }
        initialCoolingMode = false;
        targetReached = true;
      }
    } else {
      if (tempDiff > 1.0) {
        if (!statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, HIGH);
          statusKompresor = true;
        }
      } else if (tempDiff < 0) {
        if (statusKompresor) {
          digitalWrite(COMPRESSOR_PIN, LOW);
          statusKompresor = false;
        }
      }
    }
  }
}

void stopFillingBySensor() {
  digitalWrite(VALVE_INLET_PIN, LOW);
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  fillingActive = false;
  fillingStarted = false;
  fillingStage = 0;
  
  if (preFillScheduled) {
    preFillExecuted = true;
    preFillScheduled = false;
    lastPreFillDate = rtc.now();
  }
  
  if (debug) Serial.println(">> FILLING: Stopped by sensor");
}

void stopFillingManual() {
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(VALVE_INLET_PIN, LOW);
  fillingActive = false;
  fillingStarted = false;
  fillingStage = 0;
  
  if (debug) Serial.println(">> FILLING: Stopped manually");
}

void stopDraining() {
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(PUMP_UV_PIN, LOW);
  drainingActive = false;
  graceActive = false;
  flowOK = false;
  lowFlowCount = 0;
  
  if (debug) Serial.println(">> DRAINING: Stopped");
}

void readAutoSettings() {
  // Skip if receiving parameters
  if (receivingAutoParams) return;
  
  // Basic auto settings read - in full system this would read from Nextion
  if (debug) {
    static unsigned long lastSettingsDebug = 0;
    if (millis() - lastSettingsDebug >= 30000) {
      Serial.print(">> AUTO: Current settings - Time: ");
      if ((autoHour1 * 10 + autoHour2) < 10) Serial.print("0");
      Serial.print(autoHour1 * 10 + autoHour2);
      Serial.print(":");
      if ((autoMin1 * 10 + autoMin2) < 10) Serial.print("0");
      Serial.print(autoMin1 * 10 + autoMin2);
      Serial.print(" | Temp: ");
      Serial.print(autoTempTarget);
      Serial.println("°C");
      lastSettingsDebug = millis();
    }
  }
}

void processAutoSchedule() {
  // Simplified schedule processing without complex dependencies
  DateTime now = rtc.now();
  int currentMinutes = now.hour() * 60 + now.minute();
  int targetMinutes = (autoHour1 * 10 + autoHour2) * 60 + (autoMin1 * 10 + autoMin2);
  
  // Calculate days since last change (simplified)
  uint32_t daysDiff = (now.unixtime() - lastPreFillDate.unixtime()) / 86400L;
  int daysSinceLastChange = (int)daysDiff;
  
  // Pre-fill time (3 hours before target)
  int preFillMinutes = targetMinutes - 180; // 3 hours = 180 minutes
  if (preFillMinutes < 0) preFillMinutes += 1440; // Handle day rollover
  
  // Debug output (throttled)
  static unsigned long lastScheduleDebug = 0;
  if (debug && (millis() - lastScheduleDebug >= 30000)) {
    Serial.print(">> AUTO SCHEDULE: Current=");
    Serial.print(now.hour());
    Serial.print(":");
    if (now.minute() < 10) Serial.print("0");
    Serial.print(now.minute());
    Serial.print(" | Target=");
    Serial.print((autoHour1 * 10 + autoHour2));
    Serial.print(":");
    if ((autoMin1 * 10 + autoMin2) < 10) Serial.print("0");
    Serial.print((autoMin1 * 10 + autoMin2));
    Serial.print(" | Days since last=");
    Serial.print(daysSinceLastChange);
    Serial.print("/");
    Serial.println(autoChangeDay);
    lastScheduleDebug = millis();
  }
  
  // Check if it's time for pre-fill (if needed)
  if (!preFillExecuted && daysSinceLastChange >= autoChangeDay && 
      currentMinutes >= preFillMinutes && currentMinutes < targetMinutes) {
    
    if (!fillingActive && !drainingActive && !circulationActive) {
      if (debug) Serial.println(">> AUTO: Starting scheduled pre-fill");
      preFillScheduled = true;
      preFillExecuted = true;
      lastPreFillDate = now;
      fillingActive = true;
      fillingStarted = false;
      fillingStage = 0;
    }
  }
  
  // Check if it's time for operation
  if (currentMinutes >= targetMinutes && currentMinutes < targetMinutes + 60) {
    if (!circulationActive && !manualCirculationOff) {
      startManualCirculation();
    }
  }
  
  // Stop operation after 1 hour
  if (currentMinutes >= targetMinutes + 60 && circulationActive) {
    stopCirculation();
  }
}

void startManualCirculation() {
  if (!autoMode) return;
  
  circulationActive = true;
  manualCirculationOn = true;
  manualCirculationOff = false;
  initialCoolingMode = true;
  targetReached = false;
  
  digitalWrite(PUMP_UV_PIN, HIGH);
  
  if (debug) Serial.println(">> AUTO: Manual circulation started");
}

void stopCirculation() {
  circulationActive = false;
  manualCirculationOn = false;
  digitalWrite(PUMP_UV_PIN, LOW);
  digitalWrite(COMPRESSOR_PIN, LOW);
  statusKompresor = false;
  
  if (debug) Serial.println(">> AUTO: Circulation stopped");
}

void handleNextionCommunication() {
  // Basic Nextion communication handler
  processNextionInputStream();
  
  static unsigned long lastParamRead = 0;
  if (millis() - lastParamRead >= 2000) {
    readParametersFromNextion();
    lastParamRead = millis();
  }
  
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {
    updateNextionDisplayOptimized();
    lastDisplayUpdate = millis();
  }
}

void processNextionInputStream() {
  while (Serial2.available()) {
    uint8_t ch = Serial2.read();
    
    if (ch == 0xFF) {
      ffCount++;
      if (ffCount == 3) {
        if (cmd.length() > 0) {
          processNextionCommand(cmd);
          cmd = "";
        }
        ffCount = 0;
      }
    } else {
      ffCount = 0;
      cmd += (char)ch;
    }
  }
}

bool processNextionCommand(String command) {
  command.trim();
  
  if (command.indexOf("FILLING_ON") >= 0) {
    if (!autoMode && !fillingActive) {
      fillingActive = true;
      fillingStarted = false;
      fillingStage = 0;
      return true;
    }
  }
  else if (command.indexOf("FILLING_OFF") >= 0) {
    if (!autoMode) {
      stopFillingManual();
      return true;
    }
  }
  else if (command.indexOf("DRAINING_ON") >= 0) {
    if (!autoMode && !drainingActive) {
      drainingActive = true;
      graceActive = false;
      flowOK = false;
      return true;
    }
  }
  else if (command.indexOf("DRAINING_OFF") >= 0) {
    if (!autoMode) {
      stopDraining();
      return true;
    }
  }
  else if (command.indexOf("COOLING_ON") >= 0) {
    if (!autoMode) {
      activeProcess = 1;
      return true;
    }
  }
  else if (command.indexOf("COOLING_OFF") >= 0) {
    if (!autoMode) {
      activeProcess = 0;
      return true;
    }
  }
  else if (command.indexOf("AUTO_ON") >= 0) {
    autoMode = true;
    circulationActive = false;
    fillingActive = false;
    drainingActive = false;
    activeProcess = 0;
    
    // Turn off all actuators
    digitalWrite(VALVE_DRAIN_PIN, LOW);
    digitalWrite(VALVE_INLET_PIN, LOW);
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(PUMP_UV_PIN, LOW);
    
    if (debug) Serial.println(">> AUTO: Mode activated");
    return true;
  }
  else if (command.indexOf("AUTO_OFF") >= 0) {
    autoMode = false;
    circulationActive = false;
    fillingActive = false;
    drainingActive = false;
    
    // Turn off all actuators
    digitalWrite(VALVE_DRAIN_PIN, LOW);
    digitalWrite(VALVE_INLET_PIN, LOW);
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(PUMP_UV_PIN, LOW);
    
    if (debug) Serial.println(">> AUTO: Mode deactivated");
    return true;
  }
  else if (command.indexOf("CIRCULATION_ON") >= 0) {
    if (autoMode) {
      startManualCirculation();
      return true;
    }
  }
  else if (command.indexOf("CIRCULATION_OFF") >= 0) {
    if (autoMode) {
      stopCirculation();
      manualCirculationOff = true;
      return true;
    }
  }
  
  return false;
}

void readParametersFromNextion() {
  // Basic parameter reading - would be more sophisticated in full system
  static int lastActiveProcess = -1;
  
  if (!autoMode) {
    // Read activeProcess for manual mode
    // In full system this would query Nextion display
    if (activeProcess != lastActiveProcess) {
      lastActiveProcess = activeProcess;
      if (debug) {
        Serial.print(">> NEXTION: Active process changed to ");
        Serial.println(activeProcess);
      }
    }
  }
}

void updateNextionDisplayOptimized() {
  // Basic display update
  static int lastTempInt = -999;
  static int lastTDSInt = -999;
  
  int currentTempInt = (int)currentTemp;
  int currentTDSInt = (int)tdsValue;
  
  if (abs(currentTempInt - lastTempInt) >= 1) {
    Serial2.print("nTemp.val=");
    Serial2.print(currentTempInt);
    sendFF();
    lastTempInt = currentTempInt;
  }
  
  if (abs(currentTDSInt - lastTDSInt) >= 5) {
    Serial2.print("nTDS.val=");
    Serial2.print(currentTDSInt);
    sendFF();
    lastTDSInt = currentTDSInt;
  }
}

int bacaKomponenWaktu(String namaKomponen, int minVal, int maxVal) {
  // Basic time component reader
  return minVal; // Simplified implementation
}

void processAutoParameters() {
  if (debug) Serial.println(">> AUTO: Processing parameters");
  
  // Process received parameters
  for (int i = 0; i < 6 && i < paramIndex; i++) {
    if (i == 0) autoHour1 = autoParams[i].toInt();
    else if (i == 1) autoHour2 = autoParams[i].toInt();
    else if (i == 2) autoMin1 = autoParams[i].toInt();
    else if (i == 3) autoMin2 = autoParams[i].toInt();
    else if (i == 4) {
      String tempStr = autoParams[i];
      tempStr.replace("°C", "");
      autoTempTarget = tempStr.toInt();
    }
    else if (i == 5) {
      String dayStr = autoParams[i];
      dayStr.replace("hari", "");
      autoChangeDay = dayStr.toInt();
    }
  }
}

// Minimal sensor function implementations
void readTemperatureSensor(unsigned long currentTime) {
  // Implemented in main readSensors()
}

void readTDSSensor(unsigned long currentTime) {
  // Implemented in main readSensors()
}

void readFlowSensor(unsigned long currentTime) {
  // Implemented in main readSensors()
}
