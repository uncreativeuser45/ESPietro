// =========================================================================
// FINAL MASTER ESP32-S3 CONTROL CODE (CUSTOM PINOUT) - FIXED
// Fixes:
//  - stray closing brace in handleSingleClick() (sketch did not compile)
//  - duplicate NVS key "pRevDur" (now "pFwdDur" for forward duration)
//  - settings are now saved as soon as an edit is confirmed (accel etc.)
//  - encoder menu input is polled every loop, not only every 100 ms
// =========================================================================
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <FastAccelStepper.h>
#include <Preferences.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Preferences preferences;

// =========================================================================
// PIN LAYOUT
// =========================================================================
const int OLED_SDA    = 8;
const int OLED_SCL    = 9;
const int ALARM_PIN   = 4;
const int STEP_PIN    = 5;
const int DIR_PIN     = 6;
const int ENA_PIN     = 7;
const int ENCODER_CLK = 15;
const int ENCODER_DT  = 16;
const int BUTTON_PIN  = 17;

const bool ALARM_ACTIVE_LOW = true;
const bool ENA_ACTIVE_LOW = false;

// =========================================================================
// MECHANICAL & GEARBOX METRICS (4000 PPR RESOLUTION)
// =========================================================================
const float DRIVER_PPR = 4000.0;
const float GEAR_RATIO = 5.0;
const float BURR_PPR   = DRIVER_PPR * GEAR_RATIO;

int targetRPM = 60;
int minRpmLimit = 30;
int maxRpmLimit = 300;
int defaultStartupRpm = 60;
uint32_t stepperAccelHzPerS = 20000;

bool grindByTimeEnabled = false;
uint32_t grindTimeMs = 15000;        // grind-by-time duration in milliseconds
unsigned long grindStartTime = 0;

int purgeRevRpm = 40;
int purgeRevDurationSec = 5;
int purgeFwdRpm = 500;
int purgeFwdDurationSec = 8;
int purgeCycleCount = 1;
int currentPurgeCycle = 0;

enum UIState {
  UI_MAIN_SCREEN,
  UI_MENU_MAIN,
  UI_MENU_BASIC_SETTINGS,
  UI_MENU_PURGE_SETTINGS,
  UI_MENU_GTT_SETTINGS,
  UI_EDIT_VALUE
};
UIState currentUIState = UI_MAIN_SCREEN;

int mainMenuSelection = 0;
int basicMenuSelection = 0;
int purgeMenuSelection = 0;
int gttMenuSelection = 0;
bool isEditing = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
unsigned long lastOLEDUpdateTime = 0;

enum GrinderState { IDLE, RAMP_UP, GRINDING, JAMMED, PURGE_REV, PURGE_FWD, RAMP_DOWN };
GrinderState systemState = IDLE;
GrinderState nextStateAfterRamp = IDLE;

unsigned long lastButtonRelease = 0;
bool buttonPressedFlag = false;
int clickCount = 0;
const int DOUBLE_CLICK_WINDOW = 350;
bool resumeGrindingAfterPurge = false;

void loadSettings() {
  preferences.begin("grinder", false);
  minRpmLimit         = preferences.getInt("minRpm", 30);
  maxRpmLimit         = preferences.getInt("maxRpm", 300);
  defaultStartupRpm   = preferences.getInt("startRpm", 60);
  stepperAccelHzPerS  = preferences.getULong("accel", 20000);
  purgeRevRpm         = preferences.getInt("pRevRpm", 40);
  purgeRevDurationSec = preferences.getInt("pRevDur", 5);
  purgeFwdRpm         = preferences.getInt("pFwdRpm", 500);
  purgeFwdDurationSec = preferences.getInt("pFwdDur", 8);   // FIX: was "pRevDur"
  purgeCycleCount     = preferences.getInt("pCycles", 1);
  grindByTimeEnabled  = preferences.getBool("gttEn", false);
  grindTimeMs         = preferences.getUInt("gttMs", 15000);
  preferences.end();
  grindTimeMs = constrain((long)grindTimeMs, 100L, 120000L);

  // Sanity limits in case stored values are out of range
  stepperAccelHzPerS = constrain((long)stepperAccelHzPerS, 5000L, 50000L);
  targetRPM = defaultStartupRpm;
  targetRPM = constrain(targetRPM, minRpmLimit, maxRpmLimit);

  Serial.print(F("Loaded accel: "));
  Serial.println(stepperAccelHzPerS);
}

void saveSettings() {
  preferences.begin("grinder", false);
  preferences.putInt("minRpm", minRpmLimit);
  preferences.putInt("maxRpm", maxRpmLimit);
  preferences.putInt("startRpm", defaultStartupRpm);
  preferences.putULong("accel", stepperAccelHzPerS);
  preferences.putInt("pRevRpm", purgeRevRpm);
  preferences.putInt("pRevDur", purgeRevDurationSec);
  preferences.putInt("pFwdRpm", purgeFwdRpm);
  preferences.putInt("pFwdDur", purgeFwdDurationSec);       // FIX: was "pRevDur"
  preferences.putInt("pCycles", purgeCycleCount);
  preferences.putBool("gttEn", grindByTimeEnabled);
  preferences.putUInt("gttMs", grindTimeMs);
  preferences.end();

  Serial.print(F("Saved accel: "));
  Serial.print(stepperAccelHzPerS);
  Serial.print(F("  grindTimeMs: "));
  Serial.println(grindTimeMs);
}

// Called whenever the user finishes editing a value
void commitEdit() {
  if (stepper) stepper->setAcceleration(stepperAccelHzPerS);
  saveSettings();
}

volatile int8_t encoderAccum = 0;
void IRAM_ATTR updateEncoder() {
  if (systemState == PURGE_REV || systemState == PURGE_FWD) return;
  static uint8_t lastEncoded = 0;
  uint8_t msb = digitalRead(ENCODER_CLK);
  uint8_t lsb = digitalRead(ENCODER_DT);
  uint8_t encoded = (msb << 1) | lsb;
  uint8_t transition = (lastEncoded << 2) | encoded;
  if (transition == 0b1101 || transition == 0b0100 || transition == 0b0010 || transition == 0b1011) {
    encoderAccum++;
  } else if (transition == 0b1110 || transition == 0b0111 || transition == 0b0001 || transition == 0b1000) {
    encoderAccum--;
  }
  lastEncoded = encoded;
  if (encoderAccum >= 4) {
    if (currentUIState == UI_MAIN_SCREEN) {
      targetRPM += 5;
      targetRPM = constrain(targetRPM, minRpmLimit, maxRpmLimit);
    }
    encoderAccum = 0;
  } else if (encoderAccum <= -4) {
    if (currentUIState == UI_MAIN_SCREEN) {
      targetRPM -= 5;
      targetRPM = constrain(targetRPM, minRpmLimit, maxRpmLimit);
    }
    encoderAccum = 0;
  }
}

int readEncoderDelta() {
  static int lastClk = HIGH;
  static unsigned long lastTransitionTime = 0;
  const unsigned long ENCODER_DEBOUNCE_MS = 3; // Filter out high-frequency noise

  int currentClk = digitalRead(ENCODER_CLK);
  int delta = 0;

  if (currentClk != lastClk && currentClk == LOW) {
    if (millis() - lastTransitionTime > ENCODER_DEBOUNCE_MS) {
      if (digitalRead(ENCODER_DT) == HIGH) {
        delta = 1;
      } else {
        delta = -1;
      }
      lastTransitionTime = millis();
    }
  }
  lastClk = currentClk;
  return delta;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  if (!display.begin(0x3C, true)) {
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);

  pinMode(ALARM_PIN, INPUT_PULLUP);
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, ENA_ACTIVE_LOW ? HIGH : LOW);

  loadSettings();
  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  if (!stepper) for (;;);
  stepper->setDirectionPin(DIR_PIN, false);
  stepper->setAcceleration(stepperAccelHzPerS);

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), updateEncoder, CHANGE);
  updateOLEDDisplay();
}

void loop() {
  monitorButtonClicks();
  handleMenuInput();      // FIX: encoder polled every loop, not every 100 ms
  executeMotionState();
  updateOLEDDisplay();
}

void monitorButtonClicks() {
  static bool lastRawState = HIGH;
  static bool debouncedState = HIGH;
  static unsigned long lastEdgeTime = 0;
  bool rawState = digitalRead(BUTTON_PIN);
  if (rawState != lastRawState) lastEdgeTime = millis();
  if ((millis() - lastEdgeTime) > 30 && rawState != debouncedState) {
    debouncedState = rawState;
    if (debouncedState == LOW) buttonPressedFlag = true;
    else if (buttonPressedFlag) {
      clickCount++;
      lastButtonRelease = millis();
      buttonPressedFlag = false;
    }
  }
  lastRawState = rawState;
  if (clickCount > 0 && (millis() - lastButtonRelease > 350)) {
    if (clickCount == 1) handleSingleClick();
    else if (clickCount == 2) handleDoubleClick();
    else if (clickCount >= 3) handleTripleClick();
    clickCount = 0;
  }
}

void handleSingleClick() {
  if (currentUIState == UI_MAIN_SCREEN) {
    if (systemState == IDLE || systemState == JAMMED) {
      digitalWrite(ENA_PIN, ENA_ACTIVE_LOW ? LOW : HIGH);
      stepper->setSpeedInHz((uint32_t)roundf(((float)targetRPM / 60.0) * BURR_PPR));
      stepper->runForward();
      systemState = RAMP_UP;
      grindStartTime = millis();
    } else {
      stepper->stopMove();
      systemState = RAMP_DOWN;
      nextStateAfterRamp = IDLE;
    }
  } else if (currentUIState == UI_MENU_MAIN) {
    isEditing = false;
    if (mainMenuSelection == 0) { currentUIState = UI_MENU_BASIC_SETTINGS; basicMenuSelection = 0; }
    else if (mainMenuSelection == 1) { currentUIState = UI_MENU_PURGE_SETTINGS; purgeMenuSelection = 0; }
    else if (mainMenuSelection == 2) { currentUIState = UI_MENU_GTT_SETTINGS; gttMenuSelection = 0; }
    else if (mainMenuSelection == 3) { currentUIState = UI_MAIN_SCREEN; commitEdit(); }
  } else if (currentUIState == UI_MENU_BASIC_SETTINGS) {
    if (basicMenuSelection == 4) {
      isEditing = false;
      currentUIState = UI_MENU_MAIN;
      commitEdit();
    } else {
      if (isEditing) commitEdit();   // FIX: save when an edit is confirmed
      isEditing = !isEditing;
    }
  } else if (currentUIState == UI_MENU_PURGE_SETTINGS) {
    if (purgeMenuSelection == 5) {
      isEditing = false;
      currentUIState = UI_MENU_MAIN;
      commitEdit();
    } else {
      if (isEditing) commitEdit();
      isEditing = !isEditing;
    }
  } else if (currentUIState == UI_MENU_GTT_SETTINGS) {
    if (gttMenuSelection == 2) {
      isEditing = false;
      currentUIState = UI_MENU_MAIN;
      commitEdit();
    } else {
      if (isEditing) commitEdit();
      isEditing = !isEditing;
    }
  }
}

void handleDoubleClick() {
  if (currentUIState == UI_MAIN_SCREEN) {
    currentPurgeCycle = 0;
    resumeGrindingAfterPurge = (systemState == GRINDING || systemState == RAMP_UP);
    digitalWrite(ENA_PIN, ENA_ACTIVE_LOW ? LOW : HIGH);
    stepper->stopMove();
    systemState = RAMP_DOWN;
    nextStateAfterRamp = PURGE_REV;
  }
}

void handleTripleClick() {
  if (systemState == IDLE && currentUIState == UI_MAIN_SCREEN) {
    currentUIState = UI_MENU_MAIN;
    mainMenuSelection = 0;
    isEditing = false;
  }
}

// Encoder input for all menu screens (separated from drawing so no
// detents are lost while the display refresh is rate limited)
void handleMenuInput() {
  if (currentUIState == UI_MAIN_SCREEN) return;
  int delta = readEncoderDelta();
  if (delta == 0) return;

  if (currentUIState == UI_MENU_MAIN) {
    mainMenuSelection += delta;
    if (mainMenuSelection < 0) mainMenuSelection = 3;
    if (mainMenuSelection > 3) mainMenuSelection = 0;
  }
  else if (currentUIState == UI_MENU_BASIC_SETTINGS) {
    if (!isEditing) {
      basicMenuSelection += delta;
      if (basicMenuSelection < 0) basicMenuSelection = 4;
      if (basicMenuSelection > 4) basicMenuSelection = 0;
    } else {
      if (basicMenuSelection == 0) minRpmLimit = constrain(minRpmLimit + delta * 5, 10, maxRpmLimit);
      else if (basicMenuSelection == 1) maxRpmLimit = constrain(maxRpmLimit + delta * 5, minRpmLimit, 500);
      else if (basicMenuSelection == 2) defaultStartupRpm = constrain(defaultStartupRpm + delta * 5, minRpmLimit, maxRpmLimit);
      else if (basicMenuSelection == 3) {
        long v = (long)stepperAccelHzPerS + (long)delta * 1000L;
        stepperAccelHzPerS = (uint32_t)constrain(v, 5000L, 50000L);
      }
    }
  }
  else if (currentUIState == UI_MENU_PURGE_SETTINGS) {
    if (!isEditing) {
      purgeMenuSelection += delta;
      if (purgeMenuSelection < 0) purgeMenuSelection = 5;
      if (purgeMenuSelection > 5) purgeMenuSelection = 0;
    } else {
      if (purgeMenuSelection == 0) purgeRevRpm = constrain(purgeRevRpm + delta * 5, 10, 200);
      else if (purgeMenuSelection == 1) purgeRevDurationSec = constrain(purgeRevDurationSec + delta, 1, 30);
      else if (purgeMenuSelection == 2) purgeFwdRpm = constrain(purgeFwdRpm + delta * 10, 100, 600);
      else if (purgeMenuSelection == 3) purgeFwdDurationSec = constrain(purgeFwdDurationSec + delta, 1, 30);
      else if (purgeMenuSelection == 4) purgeCycleCount = constrain(purgeCycleCount + delta, 1, 5);
    }
  }
  else if (currentUIState == UI_MENU_GTT_SETTINGS) {
    if (!isEditing) {
      gttMenuSelection += delta;
      if (gttMenuSelection < 0) gttMenuSelection = 2;
      if (gttMenuSelection > 2) gttMenuSelection = 0;
    } else {
      if (gttMenuSelection == 0) grindByTimeEnabled = !grindByTimeEnabled;
      else if (gttMenuSelection == 1) {
        // Speed-sensitive step: slow turn = 10 ms, medium = 100 ms, fast = 1000 ms
        static unsigned long lastTurn = 0;
        unsigned long now = millis();
        unsigned long gap = now - lastTurn;
        lastTurn = now;
        long step = (gap < 60) ? 1000 : (gap < 150) ? 100 : 10;
        long v = (long)grindTimeMs + (long)delta * step;
        grindTimeMs = (uint32_t)constrain(v, 100L, 120000L);
      }
    }
  }
}

void executeMotionState() {
  static unsigned long stateTimer = 0;
  bool alarmActive = ALARM_ACTIVE_LOW ? (digitalRead(ALARM_PIN) == LOW) : (digitalRead(ALARM_PIN) == HIGH);
  if (alarmActive && systemState != IDLE) {
    systemState = JAMMED;
    stepper->forceStopAndNewPosition(0);
    digitalWrite(ENA_PIN, ENA_ACTIVE_LOW ? HIGH : LOW);
  }

  float burrPpr = BURR_PPR;
  switch (systemState) {
    case RAMP_UP:
      stepper->setSpeedInHz((uint32_t)roundf(((float)targetRPM / 60.0) * burrPpr));
      if ((uint32_t)labs(stepper->getCurrentSpeedInMilliHz()) >= (uint32_t)roundf(((float)targetRPM / 60.0) * burrPpr) * 1000UL) {
        systemState = GRINDING;
      }
      break;
    case GRINDING:
      stepper->setSpeedInHz((uint32_t)roundf(((float)targetRPM / 60.0) * burrPpr));
      if (grindByTimeEnabled && (millis() - grindStartTime >= (unsigned long)grindTimeMs)) {
        stepper->stopMove();
        systemState = RAMP_DOWN;
        nextStateAfterRamp = IDLE;
      }
      break;
    case RAMP_DOWN:
      if (!stepper->isRunning()) {
        systemState = nextStateAfterRamp;
        stateTimer = millis();
        if (systemState == PURGE_REV) {
          stepper->setSpeedInHz((uint32_t)roundf(((float)purgeRevRpm / 60.0) * burrPpr));
          stepper->runBackward();
        } else if (systemState == PURGE_FWD) {
          stepper->setSpeedInHz((uint32_t)roundf(((float)purgeFwdRpm / 60.0) * burrPpr));
          stepper->runForward();
        } else if (systemState == RAMP_UP) {
          stepper->setSpeedInHz((uint32_t)roundf(((float)targetRPM / 60.0) * burrPpr));
          stepper->runForward();
          grindStartTime = millis();
        } else if (systemState == IDLE) {
          digitalWrite(ENA_PIN, ENA_ACTIVE_LOW ? HIGH : LOW);
        }
      }
      break;
    case PURGE_REV:
      if (millis() - stateTimer > (unsigned long)(purgeRevDurationSec * 1000UL)) {
        stepper->stopMove();
        systemState = RAMP_DOWN;
        nextStateAfterRamp = PURGE_FWD;
      }
      break;
    case PURGE_FWD:
      if (millis() - stateTimer > (unsigned long)(purgeFwdDurationSec * 1000UL)) {
        stepper->stopMove();
        currentPurgeCycle++;
        if (currentPurgeCycle < purgeCycleCount) {
          systemState = RAMP_DOWN;
          nextStateAfterRamp = PURGE_REV;
        } else {
          systemState = RAMP_DOWN;
          nextStateAfterRamp = resumeGrindingAfterPurge ? RAMP_UP : IDLE;
        }
      }
      break;
    case IDLE:
    case JAMMED:
      break;
  }
}

void updateOLEDDisplay() {
  if (millis() - lastOLEDUpdateTime < 100) return;
  lastOLEDUpdateTime = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  if (currentUIState == UI_MAIN_SCREEN) {
    float currentHz = labs(stepper->getCurrentSpeedInMilliHz()) / 1000.0;
    int liveBurrRPM = (int)roundf((currentHz * 60.0) / BURR_PPR);

    switch (systemState) {
      case IDLE: display.print(F("STATUS: STANDBY")); break;
      case RAMP_UP: display.print(F("STATUS: STARTING")); break;
      case GRINDING: display.print(F("STATUS: GRINDING")); break;
      case RAMP_DOWN: display.print(F("STATUS: SLOWING")); break;
      case PURGE_REV: display.print(F("STATUS: PURGING [REV]")); break;
      case PURGE_FWD: display.print(F("STATUS: FLUSHING [FWD]")); break;
      case JAMMED: display.print(F("STATUS: SYSTEM JAMMED")); break;
    }

    display.setCursor(0, 22);
    display.setTextSize(2);
    display.print(F("CMD: "));
    display.print(liveBurrRPM);

    display.setTextSize(1);
    display.setCursor(0, 52);
    display.print(F("Target: "));
    display.print(targetRPM);
    display.print(F(" RPM"));
  }
  else if (currentUIState == UI_MENU_MAIN) {
    display.println(F("--- SETTINGS MENU ---"));
    const char* items[] = {"1. Basic & RPM Limits", "2. Purge Macro Config", "3. Grind By Time", "4. EXIT & SAVE"};
    for (int i = 0; i < 4; i++) {
      display.print(i == mainMenuSelection ? F("> ") : F("  "));
      display.println(items[i]);
    }
  }
  else if (currentUIState == UI_MENU_BASIC_SETTINGS) {
    display.print(isEditing ? F("[EDIT] ") : F("--- BASIC RPM ---"));
    display.println();
    char buf[32];
    snprintf(buf, sizeof(buf), "%sMin RPM: %d", (basicMenuSelection==0?">":" "), minRpmLimit); display.println(buf);
    snprintf(buf, sizeof(buf), "%sMax RPM: %d", (basicMenuSelection==1?">":" "), maxRpmLimit); display.println(buf);
    snprintf(buf, sizeof(buf), "%sStart RPM: %d", (basicMenuSelection==2?">":" "), defaultStartupRpm); display.println(buf);
    snprintf(buf, sizeof(buf), "%sAccel: %lu", (basicMenuSelection==3?">":" "), (unsigned long)stepperAccelHzPerS); display.println(buf);
    display.println(basicMenuSelection == 4 ? F("> Back to Main Menu") : F("  Back to Main Menu"));
  }
  else if (currentUIState == UI_MENU_PURGE_SETTINGS) {
    display.print(isEditing ? F("[EDIT] ") : F("--- PURGE MACRO ---"));
    display.println();
    char buf[32];
    snprintf(buf, sizeof(buf), "%sRev RPM: %d", (purgeMenuSelection==0?">":" "), purgeRevRpm); display.println(buf);
    snprintf(buf, sizeof(buf), "%sRev Dur: %ds", (purgeMenuSelection==1?">":" "), purgeRevDurationSec); display.println(buf);
    snprintf(buf, sizeof(buf), "%sFwd RPM: %d", (purgeMenuSelection==2?">":" "), purgeFwdRpm); display.println(buf);
    snprintf(buf, sizeof(buf), "%sFwd Dur: %ds", (purgeMenuSelection==3?">":" "), purgeFwdDurationSec); display.println(buf);
    snprintf(buf, sizeof(buf), "%sCycles: %d", (purgeMenuSelection==4?">":" "), purgeCycleCount); display.println(buf);
    display.println(purgeMenuSelection == 5 ? F("> Back to Main Menu") : F("  Back to Main Menu"));
  }
  else if (currentUIState == UI_MENU_GTT_SETTINGS) {
    display.print(isEditing ? F("[EDIT] ") : F("--- GRIND BY TIME ---"));
    display.println();
    display.print(gttMenuSelection == 0 ? F("> Active: ") : F("  Active: ")); display.println(grindByTimeEnabled ? F("ON") : F("OFF"));
    char buf[32];
    snprintf(buf, sizeof(buf), "%sTime: %lu.%03lus", (gttMenuSelection==1?">":" "),
             (unsigned long)(grindTimeMs / 1000), (unsigned long)(grindTimeMs % 1000)); display.println(buf);
             display.println(gttMenuSelection == 2 ? F("> Back to Main Menu") : F("  Back to Main Menu"));
  }
  display.display();
}
