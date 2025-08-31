// ======== OPTIMIZED NEXTION.INO ========
// Optimized Nextion communication with better parsing, error handling, and performance

// Communication state management
enum NextionCommState {
  NEXTION_IDLE = 0,
  NEXTION_RECEIVING = 1,
  NEXTION_PROCESSING = 2,
  NEXTION_ERROR = 3
};

// Command parsing optimization
struct CommandBuffer {
  String data;
  int ffCount;
  unsigned long lastActivity;
  NextionCommState state;
};

// Constants untuk optimization
const unsigned long NEXTION_TIMEOUT = 5000;              // 5s command timeout
const unsigned long NEXTION_READ_INTERVAL = 500;         // 0.5s read interval (optimized)
const unsigned long PARAMETER_READ_INTERVAL = 2000;      // 2s parameter read interval
const unsigned long TARGET_CHANGE_DEBOUNCE = 2000;       // 2s debounce untuk target change
const unsigned long DISPLAY_UPDATE_INTERVAL = 1000;      // 1s display update
const int MAX_COMMAND_LENGTH = 100;                      // Limit command length
const int MAX_FF_COUNT = 10;                             // Max FF bytes dalam sequence

// Global communication state
static CommandBuffer cmdBuffer = {"", 0, 0, NEXTION_IDLE};
static unsigned long lastParameterRead = 0;
static unsigned long lastDisplayUpdate = 0;
static bool parameterReadingEnabled = true;

// Error tracking
static int communicationErrors = 0;
static int consecutiveReadErrors = 0;

// Display update throttling
static struct {
  int lastTemp = -999;
  int lastTDS = -999;
  float lastFlow = -999.0;
} lastDisplayValues;

// ======== MAIN NEXTION COMMUNICATION HANDLER ========
void handleNextionCommunication() {
  unsigned long currentTime = millis();
  
  // Process incoming commands dengan throttling
  processNextionInputStream();
  
  // Read parameters dengan interval yang reasonable
  if (currentTime - lastParameterRead >= PARAMETER_READ_INTERVAL && parameterReadingEnabled) {
    readParametersFromNextion();
    lastParameterRead = currentTime;
  }
  
  // Update display dengan throttling
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateNextionDisplayOptimized();
    lastDisplayUpdate = currentTime;
  }
  
  // Handle communication timeout
  handleCommunicationTimeout(currentTime);
}

// ======== OPTIMIZED INPUT STREAM PROCESSING ========
void processNextionInputStream() {
  while (Serial2.available() && cmdBuffer.data.length() < MAX_COMMAND_LENGTH) {
    uint8_t ch = Serial2.read();
    cmdBuffer.lastActivity = millis();
    
    if (ch == 0xFF) {
      cmdBuffer.ffCount++;
      
      // Validate FF sequence
      if (cmdBuffer.ffCount > MAX_FF_COUNT) {
        // Too many FF bytes, reset buffer
        resetCommandBuffer();
        continue;
      }
      
      // Complete command jika 3 FF berturut-turut
      if (cmdBuffer.ffCount == 3) {
        processCompleteCommand();
        resetCommandBuffer();
      }
    } else {
      // Reset FF count jika ada byte lain
      cmdBuffer.ffCount = 0;
      cmdBuffer.data += (char)ch;
      cmdBuffer.state = NEXTION_RECEIVING;
    }
  }
}

void processCompleteCommand() {
  String command = cmdBuffer.data;
  command.trim();
  
  if (command.length() == 0) return;
  
  cmdBuffer.state = NEXTION_PROCESSING;
  
  // Debug dengan throttling
  static unsigned long lastCommandDebug = 0;
  if (debug && (millis() - lastCommandDebug >= 1000)) {
    Serial.print(">> NEXTION CMD: '");
    Serial.print(command);
    Serial.println("'");
    lastCommandDebug = millis();
  }
  
  // Process command dengan error handling
  if (!processNextionCommand(command)) {
    communicationErrors++;
    cmdBuffer.state = NEXTION_ERROR;
  } else {
    cmdBuffer.state = NEXTION_IDLE;
    communicationErrors = 0; // Reset on successful command
  }
}

void resetCommandBuffer() {
  cmdBuffer.data = "";
  cmdBuffer.ffCount = 0;
  cmdBuffer.state = NEXTION_IDLE;
}

void handleCommunicationTimeout(unsigned long currentTime) {
  if (cmdBuffer.state != NEXTION_IDLE && 
      (currentTime - cmdBuffer.lastActivity >= NEXTION_TIMEOUT)) {
    
    if (debug) Serial.println(">> NEXTION: Command timeout, resetting buffer");
    resetCommandBuffer();
    communicationErrors++;
  }
  
  // Disable parameter reading jika terlalu banyak error
  if (communicationErrors >= 10) {
    parameterReadingEnabled = false;
    if (debug) Serial.println(">> NEXTION: Too many errors, disabling parameter reading");
  } else if (communicationErrors == 0 && !parameterReadingEnabled) {
    parameterReadingEnabled = true;
    if (debug) Serial.println(">> NEXTION: Errors cleared, re-enabling parameter reading");
  }
}

// ======== OPTIMIZED COMMAND PROCESSOR ========
bool processNextionCommand(String command) {
  // Trim dan validate command
  command.trim();
  if (command.length() == 0) return false;
  
  try {
    // Manual control commands
    if (command.indexOf("FILLING_ON") >= 0) {
      return handleFillingCommand(true);
    }
    else if (command.indexOf("FILLING_OFF") >= 0) {
      return handleFillingCommand(false);
    }
    else if (command.indexOf("COOLING_ON") >= 0) {
      return handleCoolingCommand(true);
    }
    else if (command.indexOf("COOLING_OFF") >= 0) {
      return handleCoolingCommand(false);
    }
    else if (command.indexOf("DRAINING_ON") >= 0) {
      return handleDrainingCommand(true);
    }
    else if (command.indexOf("DRAINING_OFF") >= 0) {
      return handleDrainingCommand(false);
    }
    // Auto mode commands
    else if (command.indexOf("AUTO_ON") >= 0) {
      return handleAutoModeCommand(true, command);
    }
    else if (command.indexOf("AUTO_OFF") >= 0) {
      return handleAutoModeCommand(false, command);
    }
    // Circulation commands
    else if (command.indexOf("CIRCULATION_ON") >= 0) {
      return handleCirculationCommand(true);
    }
    else if (command.indexOf("CIRCULATION_OFF") >= 0) {
      return handleCirculationCommand(false);
    }
    // Auto parameter commands
    else if (receivingAutoParams) {
      return handleAutoParameterCommand(command);
    }
    else {
      // Unknown command
      if (debug) {
        Serial.print(">> NEXTION: Unknown command: ");
        Serial.println(command);
      }
      return false;
    }
  } catch (...) {
    if (debug) Serial.println(">> NEXTION: Command processing error");
    return false;
  }
}

// ======== COMMAND HANDLERS ========
bool handleFillingCommand(bool activate) {
  if (activate) {
    if (debug) Serial.println(">> NEXTION: Filling ON command");
    if (!fillingActive) {
      fillingActive = true;
      fillingStarted = false;
      fillingStage = 0;
      lastFloatState = digitalRead(FLOAT_SENSOR_PIN);
      
      // Update Nextion feedback
      batchNextionUpdate({
        "blinkingFM.val=1",
        "tBlinkFM.en=1", 
        "pFillingMan.pic=7",
        "activeProcess.val=2"
      });
      
      return true;
    }
  } else {
    if (debug) Serial.println(">> NEXTION: Filling OFF command");
    stopFillingManual();
    return true;
  }
  return false;
}

bool handleCoolingCommand(bool activate) {
  if (activate) {
    if (debug) Serial.println(">> NEXTION: Cooling ON command");
    if (activeProcess != 1) {
      activeProcess = 1;
      batchNextionUpdate({
        "pCoolingMan.pic=9",
        "activeProcess.val=1"
      });
      return true;
    }
  } else {
    if (debug) Serial.println(">> NEXTION: Cooling OFF command");
    if (activeProcess == 1) {
      activeProcess = 0;
      batchNextionUpdate({
        "pCoolingMan.pic=8",
        "activeProcess.val=0"
      });
      return true;
    }
  }
  return false;
}

bool handleDrainingCommand(bool activate) {
  if (activate) {
    if (debug) Serial.println(">> NEXTION: Draining ON command");
    if (!drainingActive) {
      digitalWrite(VALVE_DRAIN_PIN, HIGH);
      drainingActive = true;
      pulseCount = 0;
      lastFlowCheckTime = millis();
      
      batchNextionUpdate({
        "blinkingDM.val=1",
        "tBlinkDM.en=1",
        "pDrainingMan.pic=11",
        "activeProcess.val=3"
      });
      return true;
    }
  } else {
    if (debug) Serial.println(">> NEXTION: Draining OFF command");
    stopDraining();
    return true;
  }
  return false;
}

bool handleAutoModeCommand(bool activate, String command) {
  if (activate) {
    if (debug) Serial.println(">> NEXTION: Auto mode ON command");
    
    // Reset system state
    autoMode = true;
    circulationActive = false;
    fillingActive = false;
    drainingActive = false;
    preFillScheduled = false;
    preFillExecuted = false;
    
    // Turn off all actuators
    digitalWrite(VALVE_DRAIN_PIN, LOW);
    digitalWrite(VALVE_INLET_PIN, LOW);
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(PUMP_UV_PIN, LOW);
    statusKompresor = false;
    sistemAktif = false;
    
    // Parse parameters jika ada
    parseAutoParameters(command);
    
    // Read settings dari Nextion
    readAutoSettings();
    
    // Update display
    batchNextionUpdate({
      "tStatus.txt=\"Auto Mode\"",
      "tStatus.pic=15",
      "pAutoProcess.pic=15"
    });
    
    if (debug) Serial.println(">> Auto mode activated successfully");
    return true;
    
  } else {
    if (debug) Serial.println(">> NEXTION: Auto mode OFF command");
    
    // Disable auto mode dan reset states
    autoMode = false;
    circulationActive = false;
    fillingActive = false;
    fillingStarted = false;
    fillingStage = 0;
    drainingActive = false;
    graceActive = false;
    flowOK = false;
    
    // Turn off all actuators
    digitalWrite(VALVE_DRAIN_PIN, LOW);
    digitalWrite(VALVE_INLET_PIN, LOW);
    digitalWrite(COMPRESSOR_PIN, LOW);
    digitalWrite(PUMP_UV_PIN, LOW);
    
    // Reset flags
    statusKompresor = false;
    sistemAktif = false;
    receivingAutoParams = false;
    paramIndex = 0;
    manualCirculationOff = false;
    manualCirculationOn = false;
    
    // Update display
    batchNextionUpdate({
      "tStatus.txt=\"Manual Mode\"",
      "tStatus.pic=0",
      "pAutoProcess.pic=14",
      "pFillingMan.pic=6",
      "pCoolingMan.pic=8",
      "pDrainingMan.pic=10"
    });
    
    return true;
  }
}

bool handleCirculationCommand(bool activate) {
  if (activate) {
    if (debug) Serial.println(">> NEXTION: Circulation ON command");
    if (autoMode) {
      startManualCirculation();
      return true;
    } else {
      if (debug) Serial.println(">> NEXTION: Circulation ignored - not in auto mode");
      return false;
    }
  } else {
    if (debug) Serial.println(">> NEXTION: Circulation OFF command");
    if (autoMode && circulationActive) {
      stopCirculation();
      manualCirculationOff = true;
      return true;
    }
    return false;
  }
}

bool handleAutoParameterCommand(String command) {
  autoParams[paramIndex] = command;
  paramIndex++;
  
  if (debug) {
    Serial.print(">> NEXTION: Auto param ");
    Serial.print(paramIndex);
    Serial.print(": '");
    Serial.print(command);
    Serial.println("'");
  }
  
  // Process parameters jika sudah lengkap
  if (paramIndex >= 6) {
    processAutoParameters();
    receivingAutoParams = false;
    paramIndex = 0;
  }
  
  return true;
}

// ======== PARAMETER PARSING ========
void parseAutoParameters(String command) {
  if (command.length() <= 7) return; // "AUTO_ON" = 7 chars
  
  String params = command.substring(8); // Skip "AUTO_ON,"
  if (debug) {
    Serial.print(">> NEXTION: Parsing auto params: ");
    Serial.println(params);
  }
  
  // Simple parameter parsing
  int paramCount = 0;
  int lastIndex = 0;
  
  for (int i = 0; i <= params.length(); i++) {
    if (i == params.length() || params.charAt(i) == ',') {
      String param = params.substring(lastIndex, i);
      param.trim();
      
      if (paramCount == 0) {
        // Temperature parameter
        param.replace("°C", "");
        param.replace(" ", "");
        int temp = param.toInt();
        if (temp >= 1 && temp <= 30) {
          autoTempTarget = temp;
        }
      } else if (paramCount == 1) {
        // Day parameter
        param.replace("hari", "");
        param.replace(" ", "");
        int days = param.toInt();
        if (days >= 1 && days <= 365) {
          autoChangeDay = days;
        }
      }
      
      paramCount++;
      lastIndex = i + 1;
    }
  }
  
  if (debug) {
    Serial.print(">> NEXTION: Parsed temp=");
    Serial.print(autoTempTarget);
    Serial.print("°C, days=");
    Serial.println(autoChangeDay);
  }
}

// ======== OPTIMIZED PARAMETER READING ========
void readParametersFromNextion() {
  if (!parameterReadingEnabled) return;
  
  // Read status
  int newActiveProcess = readNextionValueSafe("activeProcess.val", activeProcess);
  if (newActiveProcess != cachedActiveProcess) {
    activeProcess = newActiveProcess;
    cachedActiveProcess = newActiveProcess;
    
    if (debug) {
      Serial.print(">> NEXTION: Status changed to ");
      Serial.println(activeProcess);
    }
  }
  
  // Read temperature target dengan validation
  int newSuhuTarget = readNextionValueSafe("nTargetTempMan.val", suhuTarget);
  validateAndUpdateTemperatureTarget(newSuhuTarget);
}

int readNextionValueSafe(String component, int fallbackValue) {
  // Flush buffer untuk clean read
  while (Serial2.available()) {
    Serial2.read();
    delay(1); // Small delay untuk buffer clearing
  }
  
  Serial2.print("get ");
  Serial2.print(component);
  sendFF();
  delay(100); // Wait for response
  
  if (Serial2.available() >= 4) {
    uint8_t header = Serial2.read();
    if (header == 0x71) {
      uint8_t lowByte = Serial2.read();
      uint8_t highByte = Serial2.read();
      Serial2.read(); // Ignore last byte
      
      int value = lowByte | (highByte << 8);
      
      // Basic validation
      if (value >= -1000 && value <= 1000) {
        consecutiveReadErrors = 0;
        return value;
      }
    }
  }
  
  // Error handling
  consecutiveReadErrors++;
  if (consecutiveReadErrors >= 5) {
    if (debug) Serial.println(">> NEXTION: Multiple read errors, using cached values");
  }
  
  return fallbackValue;
}

void validateAndUpdateTemperatureTarget(int newTarget) {
  static unsigned long lastTargetChange = 0;
  unsigned long currentTime = millis();
  
  // Validate range
  if (newTarget < 1 || newTarget > 70) {
    return; // Invalid target, keep current
  }
  
  // Debounce rapid changes
  if (newTarget != cachedSuhuTarget) {
    if (currentTime - lastTargetChange >= TARGET_CHANGE_DEBOUNCE) {
      // Reset temperature control mode jika target berubah saat sistem aktif
      if (sistemAktif && newTarget != suhuTarget) {
        initialCoolingMode = true;
        targetReached = false;
        if (debug) Serial.println(">> NEXTION: Target changed - Reset to initial cooling mode");
      }
      
      lastValidTarget = newTarget;
      suhuTarget = newTarget;
      cachedSuhuTarget = newTarget;
      lastTargetChange = currentTime;
      
      if (debug) {
        Serial.print(">> NEXTION: Temperature target updated to ");
        Serial.print(suhuTarget);
        Serial.println("°C");
      }
    }
  }
}

// ======== OPTIMIZED DISPLAY UPDATE ========
void updateNextionDisplayOptimized() {
  // Only update values yang berubah significantly
  bool needsUpdate = false;
  
  int currentTempInt = (int)currentTemp;
  int currentTDSInt = (int)tdsValue;
  
  // Calculate flow rate
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();
  float currentFlow = pulses / 7.5;
  
  // Check jika ada perubahan significant
  if (abs(currentTempInt - lastDisplayValues.lastTemp) >= 1) {
    Serial2.print("nTemp.val=");
    Serial2.print(currentTempInt);
    sendFF();
    lastDisplayValues.lastTemp = currentTempInt;
    needsUpdate = true;
  }
  
  if (abs(currentTDSInt - lastDisplayValues.lastTDS) >= 5) {
    Serial2.print("nTDS.val=");
    Serial2.print(currentTDSInt);
    sendFF();
    lastDisplayValues.lastTDS = currentTDSInt;
    needsUpdate = true;
  }
  
  if (abs(currentFlow - lastDisplayValues.lastFlow) >= 0.1) {
    char flowStr[10];
    dtostrf(currentFlow, 4, 1, flowStr);
    Serial2.print("tFlow.txt=\"");
    Serial2.print(flowStr);
    Serial2.print("\"");
    sendFF();
    lastDisplayValues.lastFlow = currentFlow;
    needsUpdate = true;
  }
  
  if (debug && needsUpdate) {
    Serial.println(">> NEXTION: Display updated");
  }
}

// ======== BATCH NEXTION UPDATE ========
void batchNextionUpdate(std::initializer_list<const char*> commands) {
  for (const char* cmd : commands) {
    Serial2.print(cmd);
    sendFF();
    delay(50); // Small delay between commands
  }
}

// ======== HELPER FUNCTIONS ========
void sendFF() {
  Serial2.write(0xFF);
  Serial2.write(0xFF);
  Serial2.write(0xFF);
}

// ======== TIME COMPONENT READER (OPTIMIZED) ========
int bacaKomponenWaktu(String namaKomponen, int minVal, int maxVal) {
  return readNextionValueSafe(namaKomponen + ".val", minVal);
}

// ======== AUTO PARAMETER PROCESSOR (OPTIMIZED) ========
void processAutoParameters() {
  if (debug) Serial.println(">> NEXTION: Processing auto parameters");
  
  // Validate dan parse semua parameters
  autoHour1 = validateAndParseParam(autoParams[0], 0, 2, 0);
  autoHour2 = validateAndParseParam(autoParams[1], 0, 9, 1);
  autoMin1 = validateAndParseParam(autoParams[2], 0, 5, 0);
  autoMin2 = validateAndParseParam(autoParams[3], 0, 9, 0);
  
  // Temperature parameter
  String tempStr = autoParams[4];
  tempStr.replace("°C", "");
  tempStr.replace(" ", "");
  autoTempTarget = validateAndParseParam(tempStr, 1, 30, 3);
  
  // Day parameter
  String dayStr = autoParams[5];
  dayStr.replace("hari", "");
  dayStr.replace(" ", "");
  autoChangeDay = validateAndParseParam(dayStr, 1, 365, 7);
  
  if (debug) {
    Serial.print(">> NEXTION: Final auto settings - Time: ");
    Serial.print(autoHour1);
    Serial.print(autoHour2);
    Serial.print(":");
    Serial.print(autoMin1);
    Serial.print(autoMin2);
    Serial.print(" | Temp: ");
    Serial.print(autoTempTarget);
    Serial.print("°C | Days: ");
    Serial.println(autoChangeDay);
  }
}

int validateAndParseParam(String param, int minVal, int maxVal, int defaultVal) {
  param.trim();
  if (param.length() == 0) return defaultVal;
  
  int value = param.toInt();
  if (value < minVal || value > maxVal) {
    if (debug) {
      Serial.print(">> NEXTION: Invalid param '");
      Serial.print(param);
      Serial.print("', using default: ");
      Serial.println(defaultVal);
    }
    return defaultVal;
  }
  
  return value;
}