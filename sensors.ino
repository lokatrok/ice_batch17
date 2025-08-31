// ======== OPTIMIZED SENSORS.INO ========
// Optimized sensor management with better error handling, calibration, and performance

// Sensor state management
enum SensorState {
  SENSOR_INITIALIZING = 0,
  SENSOR_NORMAL = 1,
  SENSOR_WARNING = 2,
  SENSOR_ERROR = 3
};

// Sensor reading structure
struct SensorReading {
  float value;
  unsigned long timestamp;
  SensorState state;
  int errorCount;
  bool isValid;
};

// Time setting mode structure
struct TimeSettingMode {
  bool active;
  int step;                    // 0=waiting, 1=hour, 2=minute, 3=confirm
  int tempHour;
  int tempMinute;
  unsigned long lastPrompt;
};

// Sensor constants
const float TEMP_MIN_VALID = -10.0;                  // Minimum valid temperature
const float TEMP_MAX_VALID = 80.0;                   // Maximum valid temperature
const float TDS_VOLTAGE_MIN = 0.0;                   // Minimum TDS voltage
const float TDS_VOLTAGE_MAX = 3.3;                   // Maximum TDS voltage
const int SENSOR_ERROR_THRESHOLD = 5;               // Max consecutive errors
const int SENSOR_STABLE_READINGS = 3;               // Readings for stability check
const unsigned long SENSOR_TIMEOUT = 10000;         // 10s sensor read timeout
const unsigned long CLOCK_UPDATE_INTERVAL = 1000;   // 1s clock update
const unsigned long TIME_PROMPT_INTERVAL = 10000;   // 10s prompt reminder
const unsigned long SENSOR_STABILIZATION_DELAY = 2000; // 2s stabilization delay

// Flow sensor constants
const float FLOW_CALIBRATION_FACTOR = 7.5;          // Pulses per liter
const unsigned long FLOW_CALCULATION_INTERVAL = 1000; // 1s flow calculation

// TDS sensor calibration
const float TDS_VREF = 3.3;                         // Reference voltage
const float TDS_ADC_RESOLUTION = 4095.0;            // 12-bit ADC
const float TDS_COMPENSATION_COEFF = 0.5;           // Temperature compensation

// Sensor readings storage
static SensorReading temperatureReading = {0.0, 0, SENSOR_INITIALIZING, 0, false};
static SensorReading tdsReading = {0.0, 0, SENSOR_INITIALIZING, 0, false};
static SensorReading flowReading = {0.0, 0, SENSOR_INITIALIZING, 0, false};

// Time setting mode
static TimeSettingMode timeMode = {false, 0, 0, 0, 0};

// Sensor timing
static unsigned long lastSensorStabilization = 0;
static unsigned long lastClockUpdate = 0;
static unsigned long lastFlowCalculation = 0;
static bool sensorsInitialized = false;

// Error tracking
static int totalSensorErrors = 0;
static bool sensorSystemHealthy = true;

// Flow sensor variables
static volatile unsigned long flowPulseCount = 0;
static unsigned long lastFlowPulseCount = 0;

// ======== SENSOR SETUP (OPTIMIZED) ========
void setupSensors() {
  if (debug) Serial.println(">> SENSORS: Initializing sensor system");
  
  // Configure pins dengan proper modes
  configureSensorPins();
  
  // Initialize sensor hardware
  initializeSensorHardware();
  
  // Setup interrupts
  setupInterrupts();
  
  // Initial stabilization period
  lastSensorStabilization = millis();
  
  if (debug) Serial.println(">> SENSORS: Sensor system initialized");
}

void configureSensorPins() {
  // Input pins dengan pull-up where needed
  pinMode(FLOAT_SENSOR_PIN, INPUT_PULLUP);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(TDS_SENSOR_PIN, INPUT);
  
  // Output pins untuk actuators
  pinMode(VALVE_DRAIN_PIN, OUTPUT);
  pinMode(VALVE_INLET_PIN, OUTPUT);
  pinMode(COMPRESSOR_PIN, OUTPUT);
  pinMode(PUMP_UV_PIN, OUTPUT);
  
  // Ensure all outputs start LOW
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(VALVE_INLET_PIN, LOW);
  digitalWrite(COMPRESSOR_PIN, LOW);
  digitalWrite(PUMP_UV_PIN, LOW);
  
  if (debug) Serial.println(">> SENSORS: Pins configured");
}

void initializeSensorHardware() {
  // Initialize temperature sensor dengan error handling
  sensors.begin();
  
  // Validate temperature sensor
  if (sensors.getDeviceCount() == 0) {
    if (debug) Serial.println(">> SENSORS: WARNING - No temperature sensors found");
    temperatureReading.state = SENSOR_ERROR;
  } else {
    sensors.setResolution(12); // 12-bit resolution
    temperatureReading.state = SENSOR_INITIALIZING;
    if (debug) Serial.println(">> SENSORS: Temperature sensor initialized");
  }
  
  // Initialize other sensors
  tdsReading.state = SENSOR_INITIALIZING;
  flowReading.state = SENSOR_INITIALIZING;
}

void setupInterrupts() {
  // Attach flow sensor interrupt dengan error handling
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowISR, RISING);
  if (debug) Serial.println(">> SENSORS: Flow sensor interrupt configured");
}

// ======== MAIN SENSOR READING FUNCTION ========
void readSensors() {
  unsigned long currentTime = millis();
  
  // Handle time setting mode first
  if (timeMode.active) {
    handleTimeSettingMode();
    return; // Skip sensor readings during time setting
  }
  
  // Check stabilization period
  if (!sensorsInitialized) {
    if (currentTime - lastSensorStabilization >= SENSOR_STABILIZATION_DELAY) {
      sensorsInitialized = true;
      if (debug) Serial.println(">> SENSORS: Stabilization period complete");
    } else {
      return; // Still stabilizing
    }
  }
  
  // Read sensors dengan error handling
  readTemperatureSensor(currentTime);
  readTDSSensor(currentTime);
  readFlowSensor(currentTime);
  
  // Update system health status
  updateSensorSystemHealth();
  
  // Update clock display
  updateClockDisplay(currentTime);
  
  // Handle serial commands for time setting
  handleSerialCommands();
  
  // Debug sensor readings (throttled)
  debugSensorReadings(currentTime);
}

// ======== TEMPERATURE SENSOR (OPTIMIZED) ========
void readTemperatureSensor(unsigned long currentTime) {
  static unsigned long lastTempRequest = 0;
  static bool tempRequested = false;
  
  // Non-blocking temperature reading
  if (!tempRequested) {
    sensors.requestTemperatures();
    tempRequested = true;
    lastTempRequest = currentTime;
    return;
  }
  
  // Wait for conversion (750ms for 12-bit)
  if (currentTime - lastTempRequest < 750) {
    return;
  }
  
  // Read temperature
  float newTemp = sensors.getTempCByIndex(0);
  tempRequested = false; // Ready for next reading
  
  // Validate reading
  if (validateTemperature(newTemp)) {
    temperatureReading.value = newTemp;
    temperatureReading.timestamp = currentTime;
    temperatureReading.isValid = true;
    temperatureReading.errorCount = 0;
    
    if (temperatureReading.state == SENSOR_ERROR) {
      temperatureReading.state = SENSOR_NORMAL;
      if (debug) Serial.println(">> SENSORS: Temperature sensor recovered");
    }
    
    // Update global variable
    currentTemp = newTemp;
    
  } else {
    // Handle temperature error
    handleTemperatureError();
  }
}

bool validateTemperature(float temp) {
  if (temp == DEVICE_DISCONNECTED_C) {
    if (debug) Serial.println(">> SENSORS: Temperature sensor disconnected");
    return false;
  }
  
  if (temp < TEMP_MIN_VALID || temp > TEMP_MAX_VALID) {
    if (debug) {
      Serial.print(">> SENSORS: Temperature out of range: ");
      Serial.println(temp);
    }
    return false;
  }
  
  return true;
}

void handleTemperatureError() {
  temperatureReading.errorCount++;
  temperatureReading.isValid = false;
  
  if (temperatureReading.errorCount >= SENSOR_ERROR_THRESHOLD) {
    temperatureReading.state = SENSOR_ERROR;
    
    // Use safe fallback temperature
    currentTemp = 25.0; // Room temperature fallback
    
    if (debug) {
      Serial.print(">> SENSORS: Temperature sensor error (count: ");
      Serial.print(temperatureReading.errorCount);
      Serial.println("), using fallback");
    }
  } else {
    temperatureReading.state = SENSOR_WARNING;
  }
}

// ======== TDS SENSOR (OPTIMIZED) ========
void readTDSSensor(unsigned long currentTime) {
  // Read raw ADC value
  int rawADC = analogRead(TDS_SENSOR_PIN);
  
  // Convert to voltage
  float voltage = rawADC * (TDS_VREF / TDS_ADC_RESOLUTION);
  
  // Validate voltage reading
  if (validateTDSVoltage(voltage)) {
    // Calculate TDS dengan temperature compensation
    float compensatedVoltage = compensateTDSTemperature(voltage);
    float ec = calculateEC(compensatedVoltage);
    float tds = ec * TDS_COMPENSATION_COEFF;
    
    // Validate final TDS value
    if (tds >= 0 && tds <= 2000) { // Reasonable TDS range
      tdsReading.value = tds;
      tdsReading.timestamp = currentTime;
      tdsReading.isValid = true;
      tdsReading.errorCount = 0;
      
      if (tdsReading.state == SENSOR_ERROR) {
        tdsReading.state = SENSOR_NORMAL;
        if (debug) Serial.println(">> SENSORS: TDS sensor recovered");
      }
      
      // Update global variable
      tdsValue = tds;
      
    } else {
      handleTDSError();
    }
  } else {
    handleTDSError();
  }
}

bool validateTDSVoltage(float voltage) {
  return (voltage >= TDS_VOLTAGE_MIN && voltage <= TDS_VOLTAGE_MAX);
}

float compensateTDSTemperature(float voltage) {
  // Simple temperature compensation
  // In real application, this would use actual temperature
  float tempCoeff = 1.0 + (currentTemp - 25.0) * 0.02; // 2% per degree
  return voltage * tempCoeff;
}

float calculateEC(float voltage) {
  // Polynomial curve fitting for EC calculation
  return (133.42 * voltage * voltage * voltage
          - 255.86 * voltage * voltage
          + 857.39 * voltage);
}

void handleTDSError() {
  tdsReading.errorCount++;
  tdsReading.isValid = false;
  
  if (tdsReading.errorCount >= SENSOR_ERROR_THRESHOLD) {
    tdsReading.state = SENSOR_ERROR;
    tdsValue = 0.0; // Safe fallback
    
    if (debug) {
      Serial.print(">> SENSORS: TDS sensor error (count: ");
      Serial.print(tdsReading.errorCount);
      Serial.println("), using fallback");
    }
  } else {
    tdsReading.state = SENSOR_WARNING;
  }
}

// ======== FLOW SENSOR (OPTIMIZED) ========
void readFlowSensor(unsigned long currentTime) {
  // Calculate flow rate at regular intervals
  if (currentTime - lastFlowCalculation >= FLOW_CALCULATION_INTERVAL) {
    
    // Get pulse count safely
    noInterrupts();
    unsigned long currentPulseCount = flowPulseCount;
    interrupts();
    
    // Calculate pulses dalam interval ini
    unsigned long pulseDiff = currentPulseCount - lastFlowPulseCount;
    lastFlowPulseCount = currentPulseCount;
    
    // Convert to flow rate (L/min)
    float flowRate = pulseDiff / FLOW_CALIBRATION_FACTOR;
    
    // Validate flow reading
    if (flowRate >= 0 && flowRate <= 50) { // Reasonable flow range
      flowReading.value = flowRate;
      flowReading.timestamp = currentTime;
      flowReading.isValid = true;
      flowReading.errorCount = 0;
      
      if (flowReading.state == SENSOR_ERROR) {
        flowReading.state = SENSOR_NORMAL;
        if (debug) Serial.println(">> SENSORS: Flow sensor recovered");
      }
      
    } else {
      handleFlowError(flowRate);
    }
    
    lastFlowCalculation = currentTime;
  }
}

void handleFlowError(float invalidFlow) {
  flowReading.errorCount++;
  flowReading.isValid = false;
  
  if (flowReading.errorCount >= SENSOR_ERROR_THRESHOLD) {
    flowReading.state = SENSOR_ERROR;
    
    if (debug) {
      Serial.print(">> SENSORS: Flow sensor error - invalid rate: ");
      Serial.println(invalidFlow);
    }
  } else {
    flowReading.state = SENSOR_WARNING;
  }
}

// Flow sensor interrupt (OPTIMIZED)
void IRAM_ATTR flowISR() {
  flowPulseCount++;
  
  // Update global pulseCount for compatibility
  pulseCount = flowPulseCount;
}

// ======== SYSTEM HEALTH MONITORING ========
void updateSensorSystemHealth() {
  int errorSensors = 0;
  int warningSensors = 0;
  
  // Count sensor states
  if (temperatureReading.state == SENSOR_ERROR) errorSensors++;
  else if (temperatureReading.state == SENSOR_WARNING) warningSensors++;
  
  if (tdsReading.state == SENSOR_ERROR) errorSensors++;
  else if (tdsReading.state == SENSOR_WARNING) warningSensors++;
  
  if (flowReading.state == SENSOR_ERROR) errorSensors++;
  else if (flowReading.state == SENSOR_WARNING) warningSensors++;
  
  // Update system health
  bool previousHealth = sensorSystemHealthy;
  sensorSystemHealthy = (errorSensors == 0);
  
  // Log health changes
  if (previousHealth != sensorSystemHealthy) {
    if (debug) {
      Serial.print(">> SENSORS: System health changed - Healthy: ");
      Serial.print(sensorSystemHealthy ? "YES" : "NO");
      Serial.print(" | Errors: ");
      Serial.print(errorSensors);
      Serial.print(" | Warnings: ");
      Serial.println(warningSensors);
    }
  }
}

// ======== CLOCK DISPLAY (OPTIMIZED) ========
void updateClockDisplay(unsigned long currentTime) {
  if (currentTime - lastClockUpdate >= CLOCK_UPDATE_INTERVAL) {
    DateTime now = rtc.now();
    
    // Format time string efficiently
    char timeString[6]; // "HH:MM\0"
    sprintf(timeString, "%02d:%02d", now.hour(), now.minute());
    
    // Send to Nextion
    Serial2.print("tClock.txt=\"");
    Serial2.print(timeString);
    Serial2.print("\"");
    sendFF();
    
    lastClockUpdate = currentTime;
  }
}

// ======== TIME SETTING MODE (IMPROVED) ========
void handleSerialCommands() {
  if (!Serial.available()) return;
  
  String input = Serial.readString();
  input.trim();
  input.toUpperCase();
  
  if (input.equals("SETTIME")) {
    activateTimeSettingMode();
  } else if (timeMode.active) {
    processTimeSettingInput(input);
  }
}

void activateTimeSettingMode() {
  timeMode.active = true;
  timeMode.step = 1;
  timeMode.tempHour = 0;
  timeMode.tempMinute = 0;
  timeMode.lastPrompt = millis();
  
  Serial.println(">> TIME SETTING: Mode activated");
  Serial.println(">> TIME SETTING: Enter hour (0-23):");
}

void processTimeSettingInput(String input) {
  int value = input.toInt();
  
  switch (timeMode.step) {
    case 1: // Hour input
      if (value >= 0 && value <= 23) {
        timeMode.tempHour = value;
        timeMode.step = 2;
        timeMode.lastPrompt = millis();
        Serial.print(">> TIME SETTING: Hour set to ");
        Serial.println(timeMode.tempHour);
        Serial.println(">> TIME SETTING: Enter minute (0-59):");
      } else {
        Serial.println(">> TIME SETTING: Invalid hour! Enter 0-23:");
      }
      break;
      
    case 2: // Minute input
      if (value >= 0 && value <= 59) {
        timeMode.tempMinute = value;
        timeMode.step = 3;
        timeMode.lastPrompt = millis();
        Serial.print(">> TIME SETTING: Minute set to ");
        Serial.println(timeMode.tempMinute);
        Serial.print(">> TIME SETTING: Confirm time ");
        Serial.print(timeMode.tempHour);
        Serial.print(":");
        if (timeMode.tempMinute < 10) Serial.print("0");
        Serial.print(timeMode.tempMinute);
        Serial.println("? (Y/N)");
      } else {
        Serial.println(">> TIME SETTING: Invalid minute! Enter 0-59:");
      }
      break;
      
    case 3: // Confirmation
      if (input.equals("Y") || input.equals("YES")) {
        confirmTimeChange();
      } else if (input.equals("N") || input.equals("NO")) {
        cancelTimeChange();
      } else {
        Serial.println(">> TIME SETTING: Please enter Y/YES or N/NO");
      }
      break;
  }
}

void confirmTimeChange() {
  DateTime now = rtc.now();
  rtc.adjust(DateTime(now.year(), now.month(), now.day(), 
                      timeMode.tempHour, timeMode.tempMinute, 0));
  
  Serial.print(">> TIME SETTING: Time successfully set to ");
  Serial.print(timeMode.tempHour);
  Serial.print(":");
  if (timeMode.tempMinute < 10) Serial.print("0");
  Serial.println(timeMode.tempMinute);
  
  deactivateTimeSettingMode();
}

void cancelTimeChange() {
  Serial.println(">> TIME SETTING: Time change cancelled");
  deactivateTimeSettingMode();
}

void deactivateTimeSettingMode() {
  timeMode.active = false;
  timeMode.step = 0;
  timeMode.tempHour = 0;
  timeMode.tempMinute = 0;
  timeMode.lastPrompt = 0;
}

void handleTimeSettingMode() {
  unsigned long currentTime = millis();
  
  // Provide periodic reminders
  if (currentTime - timeMode.lastPrompt >= TIME_PROMPT_INTERVAL) {
    switch (timeMode.step) {
      case 1:
        Serial.println(">> TIME SETTING: Still waiting for hour (0-23)");
        break;
      case 2:
        Serial.println(">> TIME SETTING: Still waiting for minute (0-59)");
        break;
      case 3:
        Serial.println(">> TIME SETTING: Still waiting for confirmation (Y/N)");
        break;
    }
    timeMode.lastPrompt = currentTime;
  }
}

// ======== DEBUG OUTPUT (OPTIMIZED) ========
void debugSensorReadings(unsigned long currentTime) {
  static unsigned long lastSensorDebug = 0;
  
  if (!debug || (currentTime - lastSensorDebug < 5000)) return;
  
  Serial.print(">> SENSORS: Temp=");
  Serial.print(currentTemp, 1);
  Serial.print("°C");
  
  if (temperatureReading.state != SENSOR_NORMAL) {
    Serial.print(" (");
    Serial.print(getSensorStateString(temperatureReading.state));
    Serial.print(")");
  }
  
  Serial.print(" | TDS=");
  Serial.print(tdsValue, 1);
  
  if (tdsReading.state != SENSOR_NORMAL) {
    Serial.print(" (");
    Serial.print(getSensorStateString(tdsReading.state));
    Serial.print(")");
  }
  
  Serial.print(" | Flow=");
  Serial.print(flowReading.value, 2);
  Serial.print(" L/min");
  
  if (flowReading.state != SENSOR_NORMAL) {
    Serial.print(" (");
    Serial.print(getSensorStateString(flowReading.state));
    Serial.print(")");
  }
  
  Serial.print(" | System Health: ");
  Serial.println(sensorSystemHealthy ? "OK" : "DEGRADED");
  
  lastSensorDebug = currentTime;
}

const char* getSensorStateString(SensorState state) {
  switch (state) {
    case SENSOR_INITIALIZING: return "INIT";
    case SENSOR_NORMAL: return "OK";
    case SENSOR_WARNING: return "WARN";
    case SENSOR_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// ======== SENSOR STATUS FUNCTIONS ========
bool isTemperatureSensorHealthy() {
  return temperatureReading.state == SENSOR_NORMAL;
}

bool isTDSSensorHealthy() {
  return tdsReading.state == SENSOR_NORMAL;
}

bool isFlowSensorHealthy() {
  return flowReading.state == SENSOR_NORMAL;
}

bool isSensorSystemHealthy() {
  return sensorSystemHealthy;
}

// ======== SENSOR DIAGNOSTIC FUNCTIONS ========
void printSensorDiagnostics() {
  Serial.println(">> SENSORS: Diagnostic Report");
  Serial.print("  Temperature: ");
  Serial.print(currentTemp);
  Serial.print("°C - State: ");
  Serial.print(getSensorStateString(temperatureReading.state));
  Serial.print(" - Errors: ");
  Serial.println(temperatureReading.errorCount);
  
  Serial.print("  TDS: ");
  Serial.print(tdsValue);
  Serial.print(" ppm - State: ");
  Serial.print(getSensorStateString(tdsReading.state));
  Serial.print(" - Errors: ");
  Serial.println(tdsReading.errorCount);
  
  Serial.print("  Flow: ");
  Serial.print(flowReading.value);
  Serial.print(" L/min - State: ");
  Serial.print(getSensorStateString(flowReading.state));
  Serial.print(" - Errors: ");
  Serial.println(flowReading.errorCount);
  
  Serial.print("  System Health: ");
  Serial.println(sensorSystemHealthy ? "HEALTHY" : "DEGRADED");
}