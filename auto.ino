// ======== OPTIMIZED AUTO.INO ========
// Simplified and optimized auto system with better scheduling logic

// Enum untuk Auto System State (lebih jelas dari boolean flags)
enum AutoSystemState {
  AUTO_IDLE = 0,
  AUTO_PRE_FILL = 1,
  AUTO_CIRCULATION = 2,
  AUTO_COOLING = 3,
  AUTO_COMPLETED = 4
};

// Enum untuk Schedule Status
enum ScheduleStatus {
  SCHEDULE_WAITING = 0,
  SCHEDULE_PRE_FILL_READY = 1,
  SCHEDULE_OPERATION_READY = 2,
  SCHEDULE_COMPLETED = 3
};

// Auto system state variables
static AutoSystemState currentAutoState = AUTO_IDLE;
static ScheduleStatus currentScheduleStatus = SCHEDULE_WAITING;

// Constants untuk auto system
const unsigned long AUTO_CHECK_INTERVAL = 60000;        // 1 menit check interval
const unsigned long PRE_FILL_HOURS_BEFORE = 3;          // 3 jam sebelum operasi
const unsigned long OPERATION_DURATION_HOURS = 1;       // 1 jam operasi
const unsigned long TEMP_CONTROL_INTERVAL = 5000;       // 5 detik temp check
const float AUTO_TEMP_HYSTERESIS = 1.0;                 // 1 derajat hysteresis

// Schedule calculation helpers
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

// Error tracking
static int scheduleCalculationErrors = 0;
static bool rtcValidated = false;

// ======== MAIN AUTO SYSTEM CONTROLLER ========
void processAutoSystem() {
  static unsigned long lastAutoCheck = 0;
  unsigned long currentTime = millis();
  
  // Check schedule setiap interval saja, tidak setiap loop
  if (currentTime - lastAutoCheck >= AUTO_CHECK_INTERVAL) {
    if (validateRTC()) {
      processAutoSchedule();
    } else {
      handleRTCError();
    }
    lastAutoCheck = currentTime;
  }
  
  // Process active operations berdasarkan state
  processActiveAutoOperations();
}

// ======== RTC VALIDATION ========
bool validateRTC() {
  if (!rtcValidated) {
    if (!rtc.begin()) {
      if (debug) Serial.println(">> AUTO ERROR: RTC not found");
      return false;
    }
    
    DateTime now = rtc.now();
    if (now.year() < 2020 || now.year() > 2030) {
      if (debug) Serial.println(">> AUTO ERROR: RTC time invalid");
      return false;
    }
    
    rtcValidated = true;
    if (debug) Serial.println(">> AUTO: RTC validated successfully");
  }
  
  return true;
}

void handleRTCError() {
  scheduleCalculationErrors++;
  if (scheduleCalculationErrors >= 5) {
    // After 5 consecutive errors, disable auto mode
    autoMode = false;
    if (debug) Serial.println(">> AUTO: Disabled due to RTC errors");
    
    // Update Nextion
    Serial2.print("tStatus.txt=\"RTC Error - Manual Mode\"");
    sendFF();
    Serial2.print("pAutoProcess.pic=14");
    sendFF();
  }
}

// ======== OPTIMIZED RTC SETUP ========
void setupRTC() {
  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  
  if (!rtc.begin()) {
    Serial.println(">> ERROR: Couldn't find RTC");
    while (1) delay(1000); // Halt with watchdog-friendly delay
  }
  
  if (rtc.lostPower()) {
    Serial.println(">> RTC lost power, setting to compile time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Initialize schedule dates to past (safe startup)
  DateTime pastDate = DateTime(2020, 1, 1);
  lastChangeDate = pastDate;
  lastPreFillDate = pastDate;
  
  rtcValidated = true;
  
  if (debug) Serial.println(">> RTC initialized with fresh schedule");
}

// ======== SIMPLIFIED AUTO SETTINGS READER ========
void readAutoSettings() {
  // Skip reading jika sedang menerima parameter
  if (receivingAutoParams) return;
  
  // Baca dengan error handling yang lebih baik
  int newHour1 = readTimeComponentSafe("nHourAuto1", 0, 2, 0);
  int newHour2 = readTimeComponentSafe("nHourAuto2", 0, 9, 1);
  int newMin1 = readTimeComponentSafe("nMinAuto1", 0, 5, 0);
  int newMin2 = readTimeComponentSafe("nMinAuto2", 0, 9, 0);
  
  // Validate combined time
  int totalHour = newHour1 * 10 + newHour2;
  int totalMin = newMin1 * 10 + newMin2;
  
  if (totalHour <= 23 && totalMin <= 59) {
    autoHour1 = newHour1;
    autoHour2 = newHour2;
    autoMin1 = newMin1;
    autoMin2 = newMin2;
  } else {
    // Use safe defaults
    autoHour1 = 0;
    autoHour2 = 1;
    autoMin1 = 0;
    autoMin2 = 0;
    if (debug) Serial.println(">> AUTO: Invalid time, using 01:00");
  }
  
  if (debug) {
    Serial.print(">> Auto Settings - Time: ");
    if (totalHour < 10) Serial.print("0");
    Serial.print(totalHour);
    Serial.print(":");
    if (totalMin < 10) Serial.print("0");
    Serial.print(totalMin);
    Serial.print(" | Temp: ");
    Serial.print(autoTempTarget);
    Serial.print("°C | Change every: ");
    Serial.print(autoChangeDay);
    Serial.println(" days");
  }
}

// Safe time component reader dengan fallback
int readTimeComponentSafe(String componentName, int minVal, int maxVal, int defaultVal) {
  int value = bacaKomponenWaktu(componentName, minVal, maxVal);
  if (value < minVal || value > maxVal) {
    if (debug) {
      Serial.print(">> AUTO: Invalid ");
      Serial.print(componentName);
      Serial.print(", using default: ");
      Serial.println(defaultVal);
    }
    return defaultVal;
  }
  return value;
}

// ======== SIMPLIFIED SCHEDULE PROCESSOR ========
void processAutoSchedule() {
  ScheduleInfo schedule = calculateScheduleInfo();
  
  if (schedule.daysSinceLastChange < 0) {
    if (debug) Serial.println(">> AUTO: Schedule calculation error");
    return;
  }
  
  // Debug scheduling info (throttled)
  static unsigned long lastScheduleDebug = 0;
  if (debug && (millis() - lastScheduleDebug >= 30000)) { // Every 30 seconds
    printScheduleDebug(schedule);
    lastScheduleDebug = millis();
  }
  
  // Process schedule berdasarkan current state
  processScheduleState(schedule);
}

ScheduleInfo calculateScheduleInfo() {
  ScheduleInfo info = {};
  DateTime now = rtc.now();
  
  // Current time info
  info.current.hour = now.hour();
  info.current.minute = now.minute();
  info.current.totalMinutes = info.current.hour * 60 + info.current.minute;
  
  // Target time info
  info.target.hour = autoHour1 * 10 + autoHour2;
  info.target.minute = autoMin1 * 10 + autoMin2;
  info.target.totalMinutes = info.target.hour * 60 + info.target.minute;
  
  // Pre-fill time calculation (simplified)
  int preFillMinutes = info.target.totalMinutes - (PRE_FILL_HOURS_BEFORE * 60);
  if (preFillMinutes < 0) {
    // Pre-fill crosses midnight
    preFillMinutes += 1440; // Add 24 hours
    info.isPreFillToday = false;
  } else {
    info.isPreFillToday = true;
  }
  
  info.preFill.totalMinutes = preFillMinutes;
  info.preFill.hour = preFillMinutes / 60;
  info.preFill.minute = preFillMinutes % 60;
  
  // Operation timing
  info.isOperationToday = (info.current.totalMinutes < info.target.totalMinutes);
  
  // Days since last change (simplified calculation)
  uint32_t daysDiff = (now.unixtime() - lastPreFillDate.unixtime()) / 86400L;
  info.daysSinceLastChange = (int)daysDiff;
  
  return info;
}

void processScheduleState(ScheduleInfo& schedule) {
  // Determine current schedule status
  ScheduleStatus newStatus = determineScheduleStatus(schedule);
  
  if (newStatus != currentScheduleStatus) {
    currentScheduleStatus = newStatus;
    if (debug) {
      Serial.print(">> AUTO: Schedule status changed to ");
      Serial.println(currentScheduleStatus);
    }
  }
  
  // Process berdasarkan status
  switch (currentScheduleStatus) {
    case SCHEDULE_PRE_FILL_READY:
      handlePreFillSchedule(schedule);
      break;
      
    case SCHEDULE_OPERATION_READY:
      handleOperationSchedule(schedule);
      break;
      
    case SCHEDULE_COMPLETED:
      handleScheduleCompletion();
      break;
      
    default:
      // SCHEDULE_WAITING - do nothing
      break;
  }
}

ScheduleStatus determineScheduleStatus(ScheduleInfo& schedule) {
  // Check if we're in pre-fill window
  bool inPreFillWindow = isInTimeWindow(schedule.current.totalMinutes, 
                                        schedule.preFill.totalMinutes,
                                        schedule.target.totalMinutes,
                                        schedule.isPreFillToday);
  
  // Check if we're in operation window  
  bool inOperationWindow = isInTimeWindow(schedule.current.totalMinutes,
                                          schedule.target.totalMinutes, 
                                          schedule.target.totalMinutes + 60,
                                          schedule.isOperationToday);
  
  // Check if schedule is completed for today
  bool scheduleCompleted = (schedule.current.totalMinutes > schedule.target.totalMinutes + 60);
  
  if (scheduleCompleted) {
    return SCHEDULE_COMPLETED;
  } else if (inOperationWindow) {
    return SCHEDULE_OPERATION_READY;
  } else if (inPreFillWindow && schedule.daysSinceLastChange >= autoChangeDay) {
    return SCHEDULE_PRE_FILL_READY;
  } else {
    return SCHEDULE_WAITING;
  }
}

bool isInTimeWindow(int currentMin, int startMin, int endMin, bool isToday) {
  if (isToday) {
    return (currentMin >= startMin && currentMin < endMin);
  } else {
    // Handle cross-midnight scenarios
    return (currentMin >= startMin || currentMin < (endMin - 1440));
  }
}

// ======== SCHEDULE HANDLERS ========
void handlePreFillSchedule(ScheduleInfo& schedule) {
  // Check if pre-fill already done today
  DateTime now = rtc.now();
  if (isPreFillDoneToday(now)) {
    if (debug) Serial.println(">> AUTO: Pre-fill already done today");
    return;
  }
  
  // Check if system is ready for pre-fill
  if (fillingActive || drainingActive || circulationActive) {
    if (debug) Serial.println(">> AUTO: System busy, delaying pre-fill");
    return;
  }
  
  // Start pre-fill process
  startAutoPreFill(now);
}

void handleOperationSchedule(ScheduleInfo& schedule) {
  // Skip if circulation manually turned off
  if (manualCirculationOff) {
    if (debug) Serial.println(">> AUTO: Circulation manually disabled");
    return;
  }
  
  // Check if already running
  if (circulationActive) {
    return; // Already running
  }
  
  // Start auto circulation
  startAutoCirculation();
}

void handleScheduleCompletion() {
  // Stop circulation if running
  if (circulationActive) {
    stopCirculation();
  }
  
  // Reset daily flags
  if (preFillExecuted) {
    preFillScheduled = false;
    preFillExecuted = false;
  }
  
  currentScheduleStatus = SCHEDULE_WAITING; // Ready for next cycle
}

// ======== AUTO OPERATION FUNCTIONS ========
void startAutoPreFill(DateTime& now) {
  if (debug) Serial.println(">> AUTO: Starting scheduled pre-fill");
  
  // Set flags
  preFillScheduled = true;
  preFillExecuted = true;
  lastPreFillDate = now;
  
  // Start filling process (reuse manual filling logic)
  fillingActive = true;
  fillingStarted = false;
  fillingStage = 0;
  currentAutoState = AUTO_PRE_FILL;
  
  // Update Nextion
  updateAutoStatus("Pre-Fill", 4);
}

void startAutoCirculation() {
  if (debug) Serial.println(">> AUTO: Starting scheduled circulation");
  
  circulationActive = true;
  manualCirculationOn = false; // This is automatic activation
  digitalWrite(PUMP_UV_PIN, HIGH);
  
  // Reset temperature control
  initialCoolingMode = true;
  targetReached = false;
  currentAutoState = AUTO_CIRCULATION;
  
  // Reset manual override flag
  manualCirculationOff = false;
  
  // Update Nextion
  updateAutoStatus("Circulation", 8);
  updateNextionCirculation(true);
}

void startManualCirculation() {
  if (debug) Serial.println(">> AUTO: Starting manual circulation");
  
  if (!autoMode) {
    if (debug) Serial.println(">> AUTO: Not in auto mode, ignoring manual circulation");
    return;
  }
  
  // Reset pre-fill flags (manual override)
  preFillScheduled = false;
  preFillExecuted = false;
  
  // Start circulation
  circulationActive = true;
  manualCirculationOn = true; // Mark as manual activation
  manualCirculationOff = false;
  
  // Reset temperature control
  initialCoolingMode = true;
  targetReached = false;
  currentAutoState = AUTO_CIRCULATION;
  
  // Activate hardware
  digitalWrite(PUMP_UV_PIN, HIGH);
  
  // Update display
  updateNextionCirculation(true);
  
  if (debug) Serial.println(">> AUTO: Manual circulation activated");
}

void stopCirculation() {
  if (debug) Serial.println(">> AUTO: Stopping circulation");
  
  circulationActive = false;
  manualCirculationOn = false;
  digitalWrite(PUMP_UV_PIN, LOW);
  digitalWrite(COMPRESSOR_PIN, LOW);
  statusKompresor = false;
  currentAutoState = AUTO_IDLE;
  
  // Update display
  updateNextionCirculation(false);
}

// ======== ACTIVE OPERATIONS PROCESSOR ========
void processActiveAutoOperations() {
  // Process filling jika aktif (reuse manual logic)
  if (fillingActive) {
    processFillingStateMachine(); // From optimized manual.ino
  }
  
  // Process draining jika aktif (reuse manual logic)  
  if (drainingActive) {
    processDrainingWithFlowControl(); // From optimized manual.ino
  }
  
  // Process temperature control jika circulation aktif
  if (circulationActive) {
    controlTemperatureAuto();
  }
}

// ======== OPTIMIZED AUTO TEMPERATURE CONTROL ========
void controlTemperatureAuto() {
  static unsigned long lastTempCheck = 0;
  unsigned long currentTime = millis();
  
  // Check temperature at defined intervals
  if (currentTime - lastTempCheck < TEMP_CONTROL_INTERVAL) {
    return;
  }
  lastTempCheck = currentTime;
  
  // Validate temperature
  if (currentTemp < -50 || currentTemp > 100) {
    if (debug) Serial.println(">> AUTO: Temperature sensor error!");
    return;
  }
  
  // Debug output (throttled)
  static unsigned long lastDebugTemp = 0;
  if (debug && (currentTime - lastDebugTemp >= 10000)) {
    Serial.print(">> AUTO TEMP: Target=");
    Serial.print(autoTempTarget);
    Serial.print("°C Actual=");
    Serial.print(currentTemp);
    Serial.print("°C Comp=");
    Serial.println(statusKompresor ? "ON" : "OFF");
    lastDebugTemp = currentTime;
  }
  
  // Temperature control logic
  float tempDifference = currentTemp - autoTempTarget;
  
  if (initialCoolingMode) {
    handleAutoInitialCooling(tempDifference);
  } else {
    handleAutoHysteresisControl(tempDifference);
  }
}

void handleAutoInitialCooling(float tempDifference) {
  if (tempDifference > 0) {
    // Still above target - activate compressor
    if (!statusKompresor) {
      digitalWrite(COMPRESSOR_PIN, HIGH);
      statusKompresor = true;
      if (debug) Serial.println(">> AUTO: Initial cooling - Compressor ON");
    }
  } else {
    // Target reached - switch to hysteresis mode
    if (statusKompresor) {
      digitalWrite(COMPRESSOR_PIN, LOW);
      statusKompresor = false;
    }
    initialCoolingMode = false;
    targetReached = true;
    if (debug) Serial.println(">> AUTO: Target reached - Switching to hysteresis");
  }
}

void handleAutoHysteresisControl(float tempDifference) {
  if (tempDifference > AUTO_TEMP_HYSTERESIS) {
    // Too hot - activate compressor
    if (!statusKompresor) {
      digitalWrite(COMPRESSOR_PIN, HIGH);
      statusKompresor = true;
      if (debug) Serial.println(">> AUTO: Hysteresis - Compressor ON");
    }
  } else if (tempDifference < 0) {
    // Below target - deactivate compressor
    if (statusKompresor) {
      digitalWrite(COMPRESSOR_PIN, LOW);
      statusKompresor = false;
      if (debug) Serial.println(">> AUTO: Hysteresis - Compressor OFF");
    }
  }
  // Within hysteresis range - maintain current state
}

// ======== HELPER FUNCTIONS ========
bool isPreFillDoneToday(DateTime& now) {
  return (preFillExecuted && 
          lastPreFillDate.day() == now.day() &&
          lastPreFillDate.month() == now.month() &&
          lastPreFillDate.year() == now.year());
}

void updateAutoStatus(const char* statusText, uint8_t picNumber) {
  Serial2.print("tStatus.txt=\"");
  Serial2.print(statusText);
  Serial2.print("\"");
  sendFF();
  
  Serial2.print("tStatus.pic=");
  Serial2.print(picNumber);
  sendFF();
}

void updateNextionCirculation(bool isActive) {
  if (isActive) {
    Serial2.print("tStatus.txt=\"Circulation\"");
    sendFF();
    Serial2.print("tStatus.pic=8");
    sendFF();
    Serial2.print("bStopCir.pic=9");
    sendFF();
    Serial2.print("blinkingSC.val=1");
    sendFF();
    Serial2.print("tBlinkSC.en=1");
    sendFF();
    Serial2.print("cirActive.val=1");
    sendFF();
  } else {
    Serial2.print("tStatus.txt=\"Auto Mode\"");
    sendFF();
    Serial2.print("tStatus.pic=15");
    sendFF();
    Serial2.print("bStopCir.pic=10");
    sendFF();
    Serial2.print("blinkingSC.val=0");
    sendFF();
    Serial2.print("tBlinkSC.en=0");
    sendFF();
    Serial2.print("cirActive.val=0");
    sendFF();
  }
}

void printScheduleDebug(ScheduleInfo& schedule) {
  Serial.print(">> AUTO SCHEDULE: Current=");
  Serial.print(schedule.current.hour);
  Serial.print(":");
  if (schedule.current.minute < 10) Serial.print("0");
  Serial.print(schedule.current.minute);
  
  Serial.print(" | Target=");
  Serial.print(schedule.target.hour);
  Serial.print(":");
  if (schedule.target.minute < 10) Serial.print("0");
  Serial.print(schedule.target.minute);
  
  Serial.print(" | Pre-fill=");
  Serial.print(schedule.preFill.hour);
  Serial.print(":");
  if (schedule.preFill.minute < 10) Serial.print("0");
  Serial.print(schedule.preFill.minute);
  
  Serial.print(" | Days since last=");
  Serial.print(schedule.daysSinceLastChange);
  Serial.print("/");
  Serial.print(autoChangeDay);
  
  Serial.print(" | Status=");
  Serial.println(currentScheduleStatus);
}