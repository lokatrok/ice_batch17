// ======== OPTIMIZED MANUAL.INO ========
// Optimized manual system functions with better performance and reliability

// Enum untuk state filling (lebih readable dan efficient)
enum FillingState {
  FILLING_IDLE = 0,
  FILLING_DRAINING = 1,
  FILLING_INLET = 2,
  FILLING_COMPLETED = 3
};

// Enum untuk temperature control mode
enum TempControlMode {
  TEMP_OFF = 0,
  TEMP_INITIAL_COOLING = 1,
  TEMP_HYSTERESIS = 2
};

// State variables (mengganti beberapa boolean dengan state yang lebih jelas)
static FillingState currentFillingState = FILLING_IDLE;
static TempControlMode currentTempMode = TEMP_OFF;

// Timing optimization - gunakan const untuk interval
const unsigned long DRAIN_DURATION = 5000;           // 5 detik draining
const unsigned long FILLING_SAFETY_TIMEOUT = 600000; // 10 menit max filling
const unsigned long FLOW_CHECK_INTERVAL = 1000;      // 1 detik flow check
const unsigned long TEMP_CHECK_INTERVAL = 3000;      // 3 detik temp check
const unsigned long FLOAT_DEBOUNCE_DELAY = 100;      // 100ms debounce

// Flow monitoring constants
const unsigned long GRACE_PERIOD = 5000;             // 5 detik grace period
const float MIN_FLOW_RATE = 0.1;                     // Minimum flow rate L/min
const float FLOW_DETECTION_THRESHOLD = 0.3;          // Flow detection threshold
const int LOW_FLOW_THRESHOLD = 3;                    // Consecutive low flow readings

// Temperature control constants  
const float TEMP_HYSTERESIS = 2.0;                   // 2 derajat hysteresis
const int TEMP_SENSOR_ERROR_THRESHOLD = 5;           // Max consecutive temp errors

// Error tracking
static int tempSensorErrorCount = 0;

// ======== MAIN MANUAL SYSTEM CONTROLLER ========
void processManualSystem() {
  // HANYA proses jika di mode manual
  if (autoMode) return;
  
  // Process active operations berdasarkan priority
  if (fillingActive) {
    processFillingStateMachine();
  }
  
  if (drainingActive) {
    processDrainingWithFlowControl();
  }
  
  // Temperature control dengan interval checking
  controlTemperatureManual();
}

// ======== OPTIMIZED FILLING WITH STATE MACHINE ========
void processFillingStateMachine() {
  static unsigned long stateStartTime = 0;
  unsigned long currentTime = millis();
  
  // Debug output dengan throttling
  static unsigned long lastDebugTime = 0;
  if (debug && (currentTime - lastDebugTime >= 2000)) {
    Serial.print(">> FILLING State: ");
    Serial.print(currentFillingState);
    Serial.print(" | Time: ");
    Serial.print((currentTime - stateStartTime) / 1000);
    Serial.println("s");
    lastDebugTime = currentTime;
  }
  
  switch (currentFillingState) {
    case FILLING_IDLE:
      // Initialize filling process
      startFillingProcess();
      stateStartTime = currentTime;
      break;
      
    case FILLING_DRAINING:
      // Wait for drain duration, then switch to inlet
      if (currentTime - stateStartTime >= DRAIN_DURATION) {
        switchToInletMode();
        stateStartTime = currentTime;
      }
      break;
      
    case FILLING_INLET:
      // Monitor float sensor and safety timeout
      if (checkFillingCompletion() || 
          (currentTime - stateStartTime >= FILLING_SAFETY_TIMEOUT)) {
        completeFillingProcess();
      }
      break;
      
    case FILLING_COMPLETED:
      // Should not reach here, but safety fallback
      if (debug) Serial.println(">> WARNING: Filling in completed state");
      resetFillingState();
      break;
  }
}

// Helper functions for filling state machine
void startFillingProcess() {
  digitalWrite(VALVE_DRAIN_PIN, HIGH);
  digitalWrite(VALVE_INLET_PIN, LOW);
  currentFillingState = FILLING_DRAINING;
  fillingStarted = true;
  fillingStage = 1; // Keep compatibility
  
  if (debug) Serial.println(">> FILLING: Started - Draining phase");
}

void switchToInletMode() {
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  delay(500); // Brief pause for valve switching
  digitalWrite(VALVE_INLET_PIN, HIGH);
  currentFillingState = FILLING_INLET;
  fillingStage = 2; // Keep compatibility
  
  if (debug) Serial.println(">> FILLING: Switched to inlet mode");
}

bool checkFillingCompletion() {
  // Debounced float sensor reading
  static bool lastStableReading = HIGH;
  static unsigned long lastChangeTime = 0;
  
  bool currentReading = digitalRead(FLOAT_SENSOR_PIN);
  
  if (currentReading != lastStableReading) {
    lastChangeTime = millis();
    lastStableReading = currentReading;
  }
  
  // Only trigger if reading is stable for debounce period
  if ((millis() - lastChangeTime >= FLOAT_DEBOUNCE_DELAY) && 
      (currentReading == LOW)) {
    if (debug) Serial.println(">> FILLING: Float sensor triggered (debounced)");
    return true;
  }
  
  return false;
}

void completeFillingProcess() {
  currentFillingState = FILLING_COMPLETED;
  
  // Determine completion reason
  bool timeoutReached = (millis() - drainStartTime >= FILLING_SAFETY_TIMEOUT);
  if (debug) {
    Serial.print(">> FILLING: Completed - Reason: ");
    Serial.println(timeoutReached ? "Timeout" : "Float sensor");
  }
  
  stopFillingBySensor();
}

void resetFillingState() {
  currentFillingState = FILLING_IDLE;
  fillingActive = false;
  fillingStarted = false;
  fillingStage = 0;
}

// ======== OPTIMIZED DRAINING WITH FLOW CONTROL ========
void processDrainingWithFlowControl() {
  static unsigned long lastFlowCheck = 0;
  static unsigned long drainStarted = 0;
  static int consecutiveLowFlow = 0;
  unsigned long currentTime = millis();
  
  // Initialize draining if just started
  if (!graceActive && !flowOK) {
    initializeDraining();
    drainStarted = currentTime;
  }
  
  // Check flow rate at regular intervals
  if (currentTime - lastFlowCheck >= FLOW_CHECK_INTERVAL) {
    float flowRate = calculateFlowRate();
    
    if (debug) {
      Serial.print(">> DRAINING: Flow rate: ");
      Serial.print(flowRate, 2);
      Serial.println(" L/min");
    }
    
    handleFlowState(flowRate, currentTime - drainStarted);
    lastFlowCheck = currentTime;
  }
}

void initializeDraining() {
  graceActive = true;
  graceStart = millis();
  digitalWrite(VALVE_DRAIN_PIN, HIGH);
  digitalWrite(PUMP_UV_PIN, HIGH);
  
  // Reset flow monitoring
  noInterrupts();
  pulseCount = 0;
  interrupts();
  lowFlowCount = 0;
  lastFlowCheckTime = millis();
  
  if (debug) Serial.println(">> DRAINING: Started with grace period");
}

float calculateFlowRate() {
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();
  
  return pulses / 7.5; // Convert pulses to L/min
}

void handleFlowState(float flowRate, unsigned long elapsedTime) {
  // Grace period handling
  if (graceActive && !flowOK) {
    if (flowRate >= FLOW_DETECTION_THRESHOLD) {
      flowOK = true;
      graceActive = false;
      lowFlowCount = 0;
      if (debug) Serial.println(">> DRAINING: Flow detected, grace period ended");
    } else if (elapsedTime >= GRACE_PERIOD) {
      // Grace period expired, check if we have any flow
      if (flowRate < MIN_FLOW_RATE) {
        if (debug) Serial.println(">> DRAINING: No flow after grace period");
        stopDraining();
      } else {
        flowOK = true;
        graceActive = false;
        if (debug) Serial.println(">> DRAINING: Minimal flow detected after grace");
      }
    }
    return;
  }
  
  // Normal operation flow monitoring
  if (flowOK) {
    if (flowRate < MIN_FLOW_RATE) {
      lowFlowCount++;
      if (debug) {
        Serial.print(">> DRAINING: Low flow count: ");
        Serial.println(lowFlowCount);
      }
      
      // Stop if consistently low flow
      if (lowFlowCount >= LOW_FLOW_THRESHOLD) {
        if (debug) Serial.println(">> DRAINING: Tank empty, stopping");
        stopDraining();
      }
    } else {
      lowFlowCount = 0; // Reset counter on good flow
    }
  }
}

// ======== OPTIMIZED TEMPERATURE CONTROL ========
void controlTemperatureManual() {
  static unsigned long lastTempCheck = 0;
  unsigned long currentTime = millis();
  
  // Check temperature at defined intervals only
  if (currentTime - lastTempCheck < TEMP_CHECK_INTERVAL) {
    return;
  }
  lastTempCheck = currentTime;
  
  // Validate temperature reading
  if (!isTemperatureValid()) {
    return;
  }
  
  // Control system based on activeProcess
  if (activeProcess == 1) {
    if (!sistemAktif) {
      activateTemperatureControl();
    }
    executeTemperatureControl();
  } else {
    if (sistemAktif) {
      deactivateTemperatureControl();
    }
  }
}

bool isTemperatureValid() {
  if (currentTemp < -50 || currentTemp > 100) {
    tempSensorErrorCount++;
    if (tempSensorErrorCount >= TEMP_SENSOR_ERROR_THRESHOLD) {
      if (debug) Serial.println(">> ERROR: Temperature sensor consistently invalid!");
      // Could trigger safety shutdown here
    }
    return false;
  }
  
  tempSensorErrorCount = 0; // Reset error count on valid reading
  return true;
}

void activateTemperatureControl() {
  sistemAktif = true;
  digitalWrite(PUMP_UV_PIN, HIGH);
  currentTempMode = TEMP_INITIAL_COOLING;
  
  if (debug) Serial.println(">> TEMP CONTROL: Activated - Initial cooling mode");
}

void executeTemperatureControl() {
  float tempDifference = currentTemp - suhuTarget;
  
  // Debug output with throttling
  static unsigned long lastDebugTemp = 0;
  if (debug && (millis() - lastDebugTemp >= 5000)) {
    Serial.print(">> TEMP: Mode=");
    Serial.print(currentTempMode);
    Serial.print(" Target=");
    Serial.print(suhuTarget);
    Serial.print("°C Actual=");
    Serial.print(currentTemp);
    Serial.print("°C Comp=");
    Serial.println(statusKompresor ? "ON" : "OFF");
    lastDebugTemp = millis();
  }
  
  switch (currentTempMode) {
    case TEMP_INITIAL_COOLING:
      handleInitialCooling(tempDifference);
      break;
      
    case TEMP_HYSTERESIS:
      handleHysteresisControl(tempDifference);
      break;
      
    default:
      if (debug) Serial.println(">> WARNING: Invalid temp control mode");
      break;
  }
}

void handleInitialCooling(float tempDifference) {
  if (tempDifference > 0) {
    // Still above target - keep cooling
    if (!statusKompresor) {
      activateCompressor();
    }
  } else {
    // Target reached - switch to hysteresis mode
    if (statusKompresor) {
      deactivateCompressor();
    }
    currentTempMode = TEMP_HYSTERESIS;
    initialCoolingMode = false; // Keep compatibility
    targetReached = true;       // Keep compatibility
    if (debug) Serial.println(">> TEMP: Target reached - Switching to hysteresis mode");
  }
}

void handleHysteresisControl(float tempDifference) {
  if (tempDifference > TEMP_HYSTERESIS) {
    // Too hot - activate cooling
    if (!statusKompresor) {
      activateCompressor();
    }
  } else if (tempDifference < 0) {
    // Below target - stop cooling
    if (statusKompresor) {
      deactivateCompressor();
    }
  }
  // Within hysteresis range - maintain current state
}

void activateCompressor() {
  digitalWrite(COMPRESSOR_PIN, HIGH);
  statusKompresor = true;
  if (debug) Serial.println(">> COMP: Activated");
}

void deactivateCompressor() {
  digitalWrite(COMPRESSOR_PIN, LOW);
  statusKompresor = false;
  if (debug) Serial.println(">> COMP: Deactivated");
}

void deactivateTemperatureControl() {
  sistemAktif = false;
  digitalWrite(COMPRESSOR_PIN, LOW);
  digitalWrite(PUMP_UV_PIN, LOW);
  statusKompresor = false;
  currentTempMode = TEMP_OFF;
  initialCoolingMode = false; // Keep compatibility
  targetReached = false;      // Keep compatibility
  
  if (debug) Serial.println(">> TEMP CONTROL: Deactivated");
}

// ======== STOPPING FUNCTIONS (OPTIMIZED) ========
void stopFillingBySensor() {
  // Clean shutdown sequence
  digitalWrite(VALVE_INLET_PIN, LOW);
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  
  // Reset all filling states
  resetFillingState();
  
  // Handle pre-fill completion if needed
  if (preFillScheduled) {
    preFillExecuted = true;
    preFillScheduled = false;
    lastPreFillDate = rtc.now();
    
    if (debug) {
      Serial.print(">> Pre-fill completed on: ");
      Serial.print(lastPreFillDate.day());
      Serial.print("/");
      Serial.print(lastPreFillDate.month());
      Serial.print("/");
      Serial.println(lastPreFillDate.year());
    }
  }
  
  if (debug) Serial.println(">> FILLING: Stopped by sensor");
  
  // Update Nextion display efficiently
  updateNextionFillingStatus(false);
}

void stopFillingManual() {
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(VALVE_INLET_PIN, LOW);
  resetFillingState();
  
  if (debug) Serial.println(">> FILLING: Stopped manually");
  updateNextionFillingStatus(false);
}

void stopDraining() {
  digitalWrite(VALVE_DRAIN_PIN, LOW);
  digitalWrite(PUMP_UV_PIN, LOW);
  
  // Reset draining states
  drainingActive = false;
  graceActive = false;
  flowOK = false;
  lowFlowCount = 0;
  
  if (debug) Serial.println(">> DRAINING: Stopped");
  updateNextionDrainingStatus(false);
}

// ======== NEXTION UPDATE HELPERS (BATCH OPERATIONS) ========
void updateNextionFillingStatus(bool isActive) {
  // Batch Nextion commands to reduce communication overhead
  if (isActive) {
    Serial2.print("blinkingFM.val=1");
    sendFF();
    Serial2.print("tBlinkFM.en=1");
    sendFF();
    Serial2.print("pFillingMan.pic=7");
    sendFF();
    Serial2.print("activeProcess.val=2");
    sendFF();
  } else {
    Serial2.print("activeProcess.val=0");
    sendFF();
    Serial2.print("blinkingFM.val=0");
    sendFF();
    Serial2.print("tBlinkFM.en=0");
    sendFF();
    Serial2.print("pFillingMan.pic=6");
    sendFF();
    
    // Auto mode status update
    if (autoMode) {
      Serial2.print("tStatus.txt=\"Auto Mode\"");
      sendFF();
      Serial2.print("tStatus.pic=15");
      sendFF();
    }
  }
}

void updateNextionDrainingStatus(bool isActive) {
  if (isActive) {
    Serial2.print("blinkingDM.val=1");
    sendFF();
    Serial2.print("tBlinkDM.en=1");
    sendFF();
    Serial2.print("pDrainingMan.pic=11");
    sendFF();
    Serial2.print("activeProcess.val=3");
    sendFF();
  } else {
    Serial2.print("activeProcess.val=0");
    sendFF();
    Serial2.print("blinkingDM.val=0");
    sendFF();
    Serial2.print("tBlinkDM.en=0");
    sendFF();
    Serial2.print("pDrainingMan.pic=10");
    sendFF();
  }
}