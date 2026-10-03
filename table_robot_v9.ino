// ============================================================
// ESP8266 D1 MINI - TABLE ROBOT — V9 (Custom Realistic Eyes edition)
// ============================================================
//
// One wheel pans a head left/right.
//
// MOTOR (HEAD PAN):   D1->BIN1, D2->BIN2, D7->PWMB
// TOUCH 1:            D0 / GPIO16 -> TOUCH OUT (blink / surprise)
// TOUCH 2:            D4 / GPIO2  -> TOUCH OUT (happy / music)
// OLED (I2C 0x3C):    D5->SCL, D6->SDA
// BATTERY DIVIDER:    Battery+ -> R1 -> A0 -> R2 -> GND
// BUZZER:             RX / GPIO3 -> 1k -> BC547 base -> buzzer(-),
//                     buzzer(+) -> 3.3V, transistor emitter -> GND
// MODE BUTTON:        D3 / GPIO0 -> BTN -> GND
//
// IMPORTANT: because the buzzer uses GPIO3 (RX), this sketch can NO
// LONGER receive serial commands from your PC. Serial.print() for
// debugging on TX still works fine (one-way, out of the board only).
//
// IMPORTANT: D3/GPIO0 is also the ESP8266's "flash mode" strapping pin.
// Do NOT hold the mode button down while powering on / resetting the
// board, or it will drop into the bootloader instead of booting normally.
//
// ---------------------------------------------------------------
// LIBRARIES YOU NEED TO INSTALL (Arduino IDE -> Library Manager):
//   1) "Adafruit GFX Library"      by Adafruit
//   2) "Adafruit SSD1306"          by Adafruit
//   3) "FluxGarage RoboEyes"       by Dennis Hoelscher / FluxGarage
//      (kept in the sketch / still instantiated, see note below)
// ---------------------------------------------------------------
//
// ---------------------------------------------------------------
// V9 CHANGELOG (vs V8)
// ---------------------------------------------------------------
// * EYES: RoboEyes' built-in shapes only draw two plain rounded
//   rectangles with no pupil, so it can't give us organic eyelids,
//   a contained moving pupil, or per-mode expressions. RoboEyes is
//   still #included and instantiated (so nothing about the library
//   dependency is removed), but all actual eye drawing now goes
//   through a small custom GFX-based "FaceEngine" below
//   (see FACE ENGINE section) which supports: rounded organic eye
//   shapes, a moving pupil that stays contained inside the eye,
//   smooth eased look-direction transitions, slow natural blinking,
//   idle micro-saccades, asymmetric per-eye motion, and six
//   expressions (normal/happy/angry/sleepy/surprised/curious) driven
//   per-mode.
// * MOTORS: pan speeds and the PWM ramp step were reduced so the
//   head moves noticeably slower and smoother (finer, more frequent
//   ramp steps instead of big jumps).
// * RETURN-TO-CENTER: idle pan and guard mode already returned to
//   headPosition 0 between moves via driveTowardPosition(); dance
//   mode now also eases back through center on every pause beat
//   (previously it just let the ramp coast to a stop wherever
//   momentum left it), and every mode switch still recenters first.
//   Net effect: whatever mode is moving the head, it always settles
//   back to the same centered facing position between motions.
// * SCREENS: time/weather layout cleaned up slightly (bigger, more
//   legible groupings; long weather strings wrap by word instead of
//   splitting mid-character). Functionality unchanged.
//
// FILL THESE IN before uploading:
const char* WIFI_SSID     = "A100";
const char* WIFI_PASSWORD = "1234ACTS";
// ============================================================

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
// RoboEyes.h #defines DEFAULT 0, which collides with the Arduino core's own
// DEFAULT macro (=1, used for analogReference()/etc). We don't call
// analogReference() in this sketch, so this is a harmless warning either
// way, but restoring the core's value avoids any confusion if that ever
// changes.
#ifdef DEFAULT
#undef DEFAULT
#define DEFAULT 1
#endif

// ============================================================
// PINS
// ============================================================
#define BIN1 5          // D1
#define BIN2 4          // D2
#define PWMB 13         // D7
#define TOUCH_PIN 16    // D0 / GPIO16
#define TOUCH2_PIN 2    // D4 / GPIO2 - second touch sensor
#define BATTERY_PIN A0
#define BUZZER_PIN 3    // RX / GPIO3 -> transistor -> buzzer
#define MODE_BUTTON_PIN 0 // D3 / GPIO0 -> BTN -> GND

// ============================================================
// DISPLAY
// ============================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_I2C_ADDR 0x3C
#define OLED_SDA_PIN D6
#define OLED_SCL_PIN D5

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RoboEyes is kept instantiated (library requirement preserved) but its
// own draw routines are no longer used for the actual eye graphics -
// our custom FaceEngine (below) renders the eyes instead. Keeping the
// object alive costs a little RAM but nothing about the dependency was
// removed, per project constraints.
RoboEyes<Adafruit_SSD1306> roboEyes(display);

// ============================================================
// CONFIG
// ============================================================
#define REVERSE_PAN_DIRECTION 0
#define PAN_LEFT  -1
#define PAN_STOP   0
#define PAN_RIGHT  1

// --- Motor speeds: slowed down substantially from V8 for a calmer feel ---
const int MIN_PAN_SPEED  = 14;   // was 20
const int MAX_PAN_SPEED  = 55;   // was 90
const int IDLE_PAN_SPEED = 16;   // was 25

const unsigned long LONG_PRESS_MS  = 1200;
const unsigned long INFO_SCREEN_MS = 5000;

const float VOLTAGE_CALIBRATION = 0.006595;
const float CALIBRATION_FACTOR  = 1.0;
const float BATTERY_FULL_V  = 4.2;
const float BATTERY_EMPTY_V = 3.1;
const int BATTERY_CRITICAL_PERCENT = 8;
const unsigned long BATTERY_READ_INTERVAL = 4000;
const unsigned long WAKE_GRACE_MS         = 20000; 

const long  GMT_OFFSET_SEC   = 5 * 3600 + 1800;  // IST
const int   DST_OFFSET_SEC   = 0;
const char* NTP_SERVER_1     = "pool.ntp.org";
const char* NTP_SERVER_2     = "time.google.com";

const unsigned long WIFI_RETRY_INTERVAL   = 30000;
const unsigned long NTP_RESYNC_INTERVAL   = 3600000;
const unsigned long WEATHER_FETCH_INTERVAL= 900000;
const char* WEATHER_URL = "http://wttr.in/Lucknow?format=%25C+%25t+(feels+%25f)";

const unsigned long BUTTON_DEBOUNCE_MS = 50;

// ---- Dead-reckoning head position ----
// No encoder on this motor, so this is an *estimate* of how far off
// center the head is, built by accumulating actual applied PWM over
// time (see updateMotorRamp). +ve = panned right of center.
long headPosition = 0;
const long CENTER_DEADBAND = 400;                 // "close enough to arrived" in position-ticks
const unsigned long POSITION_TRAVEL_TIMEOUT_MS = 4200; // safety cap per directed move (slower moves need more time)

// Guard mode timing/targets (open-loop mood timing, position-based motion)
const int GUARD_PAN_SPEED           = 18;   // was 28 - slower sweep
const long GUARD_SIDE_TARGET        = 9000; // "fully looking to one side" - tune to your mechanism's travel
const unsigned long GUARD_HOLD_MS        = 1600;
const unsigned long GUARD_CENTER_WAIT_MS = 2000;

// Dance mode timing (slowed + now recenters every pause beat)
const unsigned long DANCE_MOVE_MIN_MS  = 220;   // was 150
const unsigned long DANCE_MOVE_MAX_MS  = 480;   // was 350
const unsigned long DANCE_PAUSE_MIN_MS = 150;   // was 80
const unsigned long DANCE_PAUSE_MAX_MS = 320;   // was 180
const int DANCE_MIN_SPEED = MIN_PAN_SPEED;
const int DANCE_MAX_SPEED = 40;                 // capped well below MAX_PAN_SPEED for smoothness

// ============================================================
// TYPES
// ============================================================
enum ScreenMode { SCREEN_FACE, SCREEN_TIME, SCREEN_WEATHER, SCREEN_BATTERY };
enum RobotMode  { MODE_DEFAULT, MODE_GUARD, MODE_CLOCK, MODE_DANCE, MODE_COUNT };
const char* MODE_NAMES[MODE_COUNT] = { "DEFAULT", "GUARD", "CLOCK", "DANCE" };

// ============================================================
// GLOBAL STATE
// ============================================================
ScreenMode screenMode  = SCREEN_FACE;
unsigned long screenModeUntil = 0;
RobotMode currentMode = MODE_DEFAULT;

int currentPWM = 0, targetPWM = 0, motorDir = 0, lastNonZeroDir = 1;
unsigned long lastRampStep = 0;
const unsigned long RAMP_INTERVAL_MS = 16;  // was 12 - slightly slower ramp cadence
const int RAMP_STEP_SIZE = 14;              // was 40 - much finer steps = smoother accel

// Recentering (runs right after a mode switch if the head has drifted)
bool recentering = false;
unsigned long recenterStart = 0;

unsigned long lastIdlePan = 0;
unsigned long idlePanInterval = 7000;
int idlePanPhase = 0;
unsigned long idlePanPhaseStart = 0;
const unsigned long IDLE_PAN_GLANCE_MS = 650;

bool isSleeping = false;
unsigned long manualWakeGraceUntil = 0;

bool lastTouchState = false;
unsigned long touchDownTime = 0;
bool lastTouch2State = false;
unsigned long touch2DownTime = 0;
unsigned long faceEffectUntil = 0;
uint8_t faceEffect = 0; // 0 none, 1 pulse, 2 glitch, 3 love
unsigned long nextEffectBeepAt = 0;

// Mode button debounce
int buttonLastReading = HIGH;
int buttonStableState = HIGH;
unsigned long buttonLastDebounceTime = 0;

// Mode-change on-screen label
char modeLabelText[16] = "";
unsigned long modeLabelUntil = 0;

// Guard mode state
enum GuardPhase { GUARD_WAIT, GUARD_GOTO_LEFT, GUARD_HOLD_LEFT, GUARD_GOTO_RIGHT, GUARD_HOLD_RIGHT, GUARD_GOTO_CENTER };
GuardPhase guardPhase = GUARD_WAIT;
unsigned long guardPhaseStart = 0;

// Dance mode state
enum DancePhase { DANCE_MOVE, DANCE_PAUSE };
DancePhase dancePhase = DANCE_MOVE;
unsigned long dancePhaseStart = 0;
unsigned long dancePhaseDuration = 300;
int lastDanceDir = 1;
int danceBeatCount = 0;
const unsigned long DISCO_BEEP_MS[] = {100, 100, 100, 250};
const unsigned long DISCO_GAP_MS[]  = {100, 100, 250, 300};
const int DISCO_STEPS = 4;
int discoStep = 0;
unsigned long discoNextBeepAt = 0;

float batteryVoltage = 0;
int batteryPct = 0;
unsigned long lastBatteryRead = 0;

bool wifiEverConnected = false;
bool timeSynced = false;
unsigned long lastWifiAttempt = 0;
unsigned long lastNtpSync = 0;
unsigned long lastWeatherFetch = 0;
String lastWeatherText = "Weather not fetched yet";
unsigned long lastWeatherFetchedAt = 0;

// Buzzer (non-blocking)
bool buzzerActive = false;
unsigned long buzzerOffAt = 0;

// ============================================================
// BUZZER
// ============================================================
void buzzerOff() { digitalWrite(BUZZER_PIN, LOW); buzzerActive = false; }

void beep(unsigned long durationMs) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerActive = true;
  buzzerOffAt = millis() + durationMs;
}

void updateBuzzer() {
  if (buzzerActive && millis() >= buzzerOffAt) buzzerOff();
}

// ============================================================
// HELPERS
// ============================================================
int clampSpeed(int s) { return constrain(s, MIN_PAN_SPEED, MAX_PAN_SPEED); }

int speedToPWM(int speedPercent) {
  speedPercent = constrain(speedPercent, 0, 100);
  return map(speedPercent, 0, 100, 0, 1023);
}

void enterSleep();

// ============================================================
// FACE ENGINE  (custom realistic eyes drawn with Adafruit_GFX)
// ------------------------------------------------------------
// RoboEyes only gives two plain rounded-rect eyes with no pupil
// control, so this replaces the actual drawing with a small hand
// rolled face renderer: organic rounded eye shapes, a pupil that
// stays contained inside the eye and eases toward a look target,
// slow natural blinking, idle micro-saccades, per-eye asymmetry,
// and six expressions.
// ============================================================
// Named EyeExpr (not "Expression") to avoid clashing with a macro/type of
// the same name pulled in by FluxGarage_RoboEyes.h.
// Arduino IDE 1.8.x can generate function prototypes before enum declarations.
// Use uint8_t for the expression type to avoid that prototype-order problem.

enum EyeExpression {
  EXPR_NORMAL,
  EXPR_HAPPY,
  EXPR_ANGRY,
  EXPR_SLEEPY,
  EXPR_SURPRISED,
  EXPR_CURIOUS,
  EXPR_EXCITED
};

uint8_t currentExpression = EXPR_NORMAL;

// Smoothed look direction, range roughly -1..1 for x and y
float curLookX = 0, curLookY = 0;
float targetLookX = 0, targetLookY = 0;
const float LOOK_EASE = 0.06f;   // lower = slower/smoother eye travel

// Blink state: 0 = fully open, 1 = fully closed
float blinkAmount = 0;
float blinkTarget = 0;
const float BLINK_EASE = 0.22f;
unsigned long nextBlinkAt = 0;
unsigned long touchReactionUntil = 0;

// Idle micro-saccade state machine
enum SaccadePhase { SAC_CENTER_WAIT, SAC_OUT, SAC_HOLD, SAC_BACK };
SaccadePhase saccadePhase = SAC_CENTER_WAIT;
unsigned long saccadePhaseStart = 0;
unsigned long saccadeWaitDuration = 3000;

// Base geometry (128x64 OLED)
const int EYE_CY = 30;
const int EYE_L_CX = 34;
const int EYE_R_CX = 94;
const int EYE_BASE_W = 40;
const int EYE_BASE_H = 34;

unsigned long lastFaceStep = 0;
const unsigned long FACE_STEP_MS = 20; // ~50fps easing update, drawing throttled separately

void scheduleNextBlink() {
  nextBlinkAt = millis() + random(3200, 7500); // slow, natural blink cadence
}

void faceSetLookTarget(float x, float y) {
  targetLookX = constrain(x, -1.0f, 1.0f);
  targetLookY = constrain(y, -1.0f, 1.0f);
}

void faceSetExpression(uint8_t e) {
  currentExpression = e;
}

void faceRequestBlink() {
  blinkTarget = 1.0f;
}

void startTouchReaction() {
  touchReactionUntil = millis() + 1900;
  faceEffect = 1;
  faceEffectUntil = touchReactionUntil;
  faceSetExpression(EXPR_EXCITED);
  faceSetLookTarget(random(-70, 71) / 100.0f, -0.25f);
  faceRequestBlink();
  beep(45);
}

void startFaceEffect(uint8_t effect, unsigned long durationMs) {
  faceEffect = effect;
  faceEffectUntil = millis() + durationMs;
  if (effect == 1) { faceSetExpression(EXPR_SURPRISED); faceRequestBlink(); beep(55); }
  else if (effect == 2) { faceSetExpression(EXPR_EXCITED); faceSetLookTarget(random(-100, 101) / 100.0f, 0); nextEffectBeepAt = millis(); }
  else { faceSetExpression(EXPR_HAPPY); faceSetLookTarget(0, -0.2f); beep(80); }
}

void updateFaceEffect() {
  if (!faceEffect) return;
  if (millis() >= faceEffectUntil) { faceEffect = 0; faceSetExpression(EXPR_NORMAL); faceSetLookTarget(0, 0); return; }
  if (faceEffect == 2 && millis() >= nextEffectBeepAt) { beep(random(18, 45)); nextEffectBeepAt = millis() + random(110, 260); }
}

void updateTouchReaction() {
  if (touchReactionUntil == 0 || millis() >= touchReactionUntil) {
    if (touchReactionUntil != 0) {
      touchReactionUntil = 0;
      faceSetExpression(EXPR_NORMAL);
      faceSetLookTarget(0, 0);
      scheduleNextBlink();
    }
    return;
  }
  unsigned long elapsed = millis() - (touchReactionUntil - 1900);
  if (elapsed > 650 && elapsed < 900) faceSetLookTarget(-0.55f, 0.1f);
  if (elapsed > 1150 && elapsed < 1400) faceSetLookTarget(0.45f, -0.1f);
}

// Idle behaviour: center -> tiny glance -> center -> (independent) blink -> repeat
void updateIdleSaccades() {
  unsigned long now = millis();
  unsigned long elapsed = now - saccadePhaseStart;
  switch (saccadePhase) {
    case SAC_CENTER_WAIT:
      faceSetLookTarget(0, 0);
      if (elapsed > saccadeWaitDuration) {
        float gx = (random(-100, 101) / 100.0f) * 0.7f;
        float gy = (random(-100, 101) / 100.0f) * 0.35f;
        faceSetLookTarget(gx, gy);
        saccadePhase = SAC_OUT;
        saccadePhaseStart = now;
      }
      break;
    case SAC_OUT:
      if (elapsed > random(350, 650)) {
        saccadePhase = SAC_HOLD;
        saccadePhaseStart = now;
      }
      break;
    case SAC_HOLD:
      if (elapsed > random(500, 1100)) {
        faceSetLookTarget(0, 0);
        saccadePhase = SAC_BACK;
        saccadePhaseStart = now;
      }
      break;
    case SAC_BACK:
      if (elapsed > random(400, 700)) {
        saccadePhase = SAC_CENTER_WAIT;
        saccadePhaseStart = now;
        saccadeWaitDuration = random(3500, 9000);
      }
      break;
  }
}

void updateFaceTiming() {
  if (millis() - lastFaceStep < FACE_STEP_MS) return;
  lastFaceStep = millis();

  // ease look direction toward target
  curLookX += (targetLookX - curLookX) * LOOK_EASE;
  curLookY += (targetLookY - curLookY) * LOOK_EASE;

  // ease blink amount toward target, then release the target back open
  blinkAmount += (blinkTarget - blinkAmount) * BLINK_EASE;
  if (blinkTarget > 0.5f && blinkAmount > 0.9f) {
    blinkTarget = 0.0f; // fully closed, now let it ease back open
  }

  if (currentMode == MODE_DEFAULT || currentMode == MODE_CLOCK) {
    if (millis() >= nextBlinkAt && blinkTarget < 0.5f) {
      faceRequestBlink();
      scheduleNextBlink();
    }
  }
}

// Draws one eye. cx/cy = center, w/h = base size, look = -1..1,
// blink = 0(open)..1(closed), expr = expression to apply.
void drawEye(int cx, int cy, int w, int h,
             float lookX, float lookY, float blink,
             uint8_t expr, bool isLeftEye) {
  // Expression-driven size tweaks
  int wEff = w;
  int hEff = h;
  if (expr == EXPR_SURPRISED) { wEff = w + 4; hEff = h + 6; }
  if (expr == EXPR_CURIOUS)   { if (isLeftEye) { wEff += 3; hEff += 3; } }

  // Blink closes the eye down to a thin slit rather than vanishing
  int minH = 4;
  int drawH = (int)(hEff - (hEff - minH) * blink);
  if (drawH < minH) drawH = minH;

  int radius = drawH / 3;
  if (radius < 2) radius = 2;
  int x0 = cx - wEff / 2;
  int y0 = cy - drawH / 2;

  display.fillRoundRect(x0, y0, wEff, drawH, radius, SSD1306_WHITE);

  // Pupil - only meaningful once the eye is open enough
  if (drawH > minH + 4) {
    int pupilR = (expr == EXPR_SURPRISED) ? 8 : (expr == EXPR_SLEEPY ? 5 : 7);
    int maxOffX = wEff / 2 - pupilR - 3;
    int maxOffY = drawH / 2 - pupilR - 2;
    if (maxOffX < 1) maxOffX = 1;
    if (maxOffY < 1) maxOffY = 1;

    int px = cx + (int)(lookX * maxOffX);
    int py = cy + (int)(lookY * maxOffY);

    display.fillCircle(px, py, pupilR, SSD1306_BLACK);
    // tiny life-like highlight
    if (pupilR > 4) {
      display.fillCircle(px - pupilR / 3, py - pupilR / 3, 1, SSD1306_WHITE);
    }
  }

  // Expression overlays (drawn in BLACK to "cut" eyelid shapes into the eye)
  switch (expr) {
    case EXPR_HAPPY: {
      // rising lower lid - cheerful squint
      int coverH = drawH * 4 / 10;
      if (coverH > 0) {
        display.fillRoundRect(x0 - 1, y0 + drawH - coverH, wEff + 2, coverH + 2, radius, SSD1306_BLACK);
      }
      break;
    }
    case EXPR_ANGRY: {
      // slanted brow toward the nose: triangle over the top-inner corner
      int innerX = isLeftEye ? (x0 + wEff) : x0;
      int outerX = isLeftEye ? x0 : (x0 + wEff);
      display.fillTriangle(outerX, y0 - 1,
                            innerX, y0 - 1,
                            innerX, y0 + drawH / 2,
                            SSD1306_BLACK);
      break;
    }
    case EXPR_SLEEPY: {
      int coverH = drawH * 45 / 100;
      display.fillRect(x0 - 1, y0 - 1, wEff + 2, coverH, SSD1306_BLACK);
      break;
    }
    case EXPR_EXCITED:
      display.fillRoundRect(x0 - 1, y0 + drawH - drawH / 5, wEff + 2, drawH / 5 + 2, radius, SSD1306_BLACK);
      break;
    default:
      break;
  }
}

// Renders the whole face for the current state. Call this once per
// loop iteration when the eyes should be visible (not during sleep).
void renderFace() {
  updateFaceTiming();
  display.clearDisplay();

  // very subtle asymmetry so the two eyes don't feel like identical
  // rectangles: right eye lags the look-target slightly.
  float leftLookX = curLookX;
  float leftLookY = curLookY;
  float rightLookX = curLookX * 0.92f;
  float rightLookY = curLookY * 0.92f;

  float leftBlink = blinkAmount;
  float rightBlink = blinkAmount * 0.96f + 0.02f; // eyes don't blink in perfect lockstep

  drawEye(EYE_L_CX, EYE_CY, EYE_BASE_W, EYE_BASE_H, leftLookX, leftLookY, leftBlink, currentExpression, true);
  drawEye(EYE_R_CX, EYE_CY, EYE_BASE_W, EYE_BASE_H, rightLookX, rightLookY, rightBlink, currentExpression, false);
  if (faceEffect == 1) { int r = (millis() % 360) / 3; display.drawCircle(64, 30, r, SSD1306_WHITE); display.drawCircle(64, 30, r + 2, SSD1306_WHITE); }
  else if (faceEffect == 2 && random(10) > 3) display.drawFastHLine(random(0, 100), random(8, 56), random(8, 30), SSD1306_WHITE);
  else if (faceEffect == 3) { display.fillCircle(58, 53, 4, SSD1306_WHITE); display.fillCircle(70, 53, 4, SSD1306_WHITE); display.fillTriangle(54, 54, 74, 54, 64, 63, SSD1306_WHITE); }

  drawWifiHUD();
  drawModeLabelOverlayIfNeeded();
  display.display();
}

void renderSleepFace() {
  display.clearDisplay();
  // eyes drawn as thin closed slits regardless of easing state
  int y0L = EYE_CY - 2, y0R = EYE_CY - 2;
  display.fillRoundRect(EYE_L_CX - EYE_BASE_W / 2, y0L, EYE_BASE_W, 4, 2, SSD1306_WHITE);
  display.fillRoundRect(EYE_R_CX - EYE_BASE_W / 2, y0R, EYE_BASE_W, 4, 2, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(96, 6);
  display.print("Zzz");
  drawWifiHUD();
  display.display();
}

// ============================================================
// WIFI + TIME
// ============================================================
void startWifiConnect() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
}

void beginTimeSync() {
  configTime(GMT_OFFSET_SEC, DST_OFFSET_SEC, NTP_SERVER_1, NTP_SERVER_2);
}

bool isTimeSynced() {
  time_t now = time(nullptr);
  return now > 8 * 3600 * 2;
}

void manageWifiAndTime() {
  wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    if (!wifiEverConnected) {
      wifiEverConnected = true;
      Serial.println();
      Serial.println("WiFi connected: " + WiFi.localIP().toString());
      beginTimeSync();
      lastNtpSync = millis();
    }
    if (!timeSynced && isTimeSynced()) {
      timeSynced = true;
      Serial.println("Time synced.");
    }
    if (millis() - lastNtpSync > NTP_RESYNC_INTERVAL) {
      beginTimeSync();
      lastNtpSync = millis();
    }
  } else {
    if (millis() - lastWifiAttempt > WIFI_RETRY_INTERVAL) {
      lastWifiAttempt = millis();
      Serial.println("WiFi not connected, retrying...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }
}

void getTimeString(char* buf, size_t len) {
  if (!timeSynced) { snprintf(buf, len, "Syncing..."); return; }
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  snprintf(buf, len, "%02d:%02d", t->tm_hour, t->tm_min);
}

void getDateString(char* buf, size_t len) {
  if (!timeSynced) { buf[0] = '\0'; return; }
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  const char* days[]   = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  const char* months[] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  snprintf(buf, len, "%s %d %s", days[t->tm_wday], t->tm_mday, months[t->tm_mon]);
}

void getUptimeString(char* buf, size_t len) {
  unsigned long s = millis() / 1000;
  unsigned long h = s / 3600; s %= 3600;
  unsigned long m = s / 60;
  snprintf(buf, len, "Up %luh %lum", h, m);
}

// ============================================================
// WEATHER
// ============================================================
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClient client;
  HTTPClient http;
  http.setTimeout(6000);

  if (http.begin(client, WEATHER_URL)) {
    int code = http.GET();
    if (code == 200) {
      String payload = http.getString();
      payload.trim();
      if (payload.length() > 0 && payload.length() < 60) {
        lastWeatherText = payload;
        lastWeatherFetchedAt = millis();
        Serial.println("Weather: " + lastWeatherText);
      }
    } else {
      Serial.print("Weather fetch failed, code=");
      Serial.println(code);
    }
    http.end();
  }
}

void manageWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (lastWeatherFetchedAt == 0 || millis() - lastWeatherFetch > WEATHER_FETCH_INTERVAL) {
    lastWeatherFetch = millis();
    fetchWeather();
  }
}

void getWeatherAgeString(char* buf, size_t len) {
  if (lastWeatherFetchedAt == 0) { snprintf(buf, len, "never updated"); return; }
  unsigned long mins = (millis() - lastWeatherFetchedAt) / 60000;
  if (mins < 1) snprintf(buf, len, "just now");
  else if (mins == 1) snprintf(buf, len, "1 min ago");
  else snprintf(buf, len, "%lu min ago", mins);
}

// ============================================================
// BATTERY
// ============================================================
int batteryPercent(float voltage) {
  float pct = (voltage - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V) * 100.0;
  return constrain((int)pct, 0, 100);
}

void updateBattery(bool force) {
  if (!force && millis() - lastBatteryRead < BATTERY_READ_INTERVAL) return;
  lastBatteryRead = millis();

  int raw = analogRead(BATTERY_PIN);
  float newVoltage = raw * VOLTAGE_CALIBRATION * CALIBRATION_FACTOR;
  batteryVoltage = (batteryVoltage == 0) ? newVoltage : (batteryVoltage * 0.8 + newVoltage * 0.2);
  batteryPct = batteryPercent(batteryVoltage);

  bool graceActive = millis() < manualWakeGraceUntil;
  if (batteryPct <= BATTERY_CRITICAL_PERCENT && !isSleeping && !graceActive) {
    enterSleep();
    Serial.println("Battery critical - auto-sleeping.");
  }
}

// ============================================================
// GFX DRAWING HELPERS (text, wifi dot, overlays - no battery icon)
// ============================================================
void centerText(const char* text, int16_t y) {
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, y);
  display.print(text);
}

void drawWifiHUD() {
  int x = 4, y = 4;
  if (WiFi.status() == WL_CONNECTED) display.fillCircle(x, y, 3, SSD1306_WHITE);
  else display.drawCircle(x, y, 3, SSD1306_WHITE);
}

void drawModeLabelOverlayIfNeeded() {
  if (millis() >= modeLabelUntil) return;
  display.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(modeLabelText, 0, 0, &x1, &y1, &w, &h);
  int bx = (SCREEN_WIDTH - (int)w) / 2 - 4;
  int by = 24;
  display.fillRect(bx, by, w + 8, h + 8, SSD1306_BLACK);
  display.drawRect(bx, by, w + 8, h + 8, SSD1306_WHITE);
  display.setCursor((SCREEN_WIDTH - (int)w) / 2, by + 4);
  display.print(modeLabelText);
}

// ============================================================
// TIME / WEATHER / STATUS SCREENS (MODE_DEFAULT tap-cycle + MODE_CLOCK)
// ============================================================
void renderTimeScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  centerText("INDIA TIME (IST)", 2);
  display.drawFastHLine(0, 13, SCREEN_WIDTH, SSD1306_WHITE);

  char t[16];
  getTimeString(t, sizeof(t));
  display.setTextSize(3);
  centerText(t, 20);

  char d[24];
  getDateString(d, sizeof(d));
  display.setTextSize(1);
  centerText(d, 52);

  drawWifiHUD();
  drawModeLabelOverlayIfNeeded();
  display.display();
}

// Breaks a string on the nearest space at/after a max width instead of
// splitting mid-character, so long wttr.in strings wrap cleanly.
void splitWeatherLine(const String& s, String& line1, String& line2, int maxCharsPerLine) {
  if ((int)s.length() <= maxCharsPerLine) {
    line1 = s;
    line2 = "";
    return;
  }
  int breakAt = s.lastIndexOf(' ', maxCharsPerLine);
  if (breakAt <= 0) breakAt = maxCharsPerLine;
  line1 = s.substring(0, breakAt);
  line2 = s.substring(breakAt + 1);
}

void renderWeatherScreen() {
  display.clearDisplay();
  int ix = 14, iy = 29;
  bool rainy = lastWeatherText.indexOf("rain") >= 0 || lastWeatherText.indexOf("Rain") >= 0;
  bool cloudy = lastWeatherText.indexOf("cloud") >= 0 || lastWeatherText.indexOf("Cloud") >= 0 || rainy;
  if (!cloudy) {
    display.drawCircle(ix, iy, 8, SSD1306_WHITE);
    display.drawPixel(ix - 12, iy, SSD1306_WHITE); display.drawPixel(ix + 12, iy, SSD1306_WHITE);
    display.drawPixel(ix, iy - 12, SSD1306_WHITE); display.drawPixel(ix, iy + 12, SSD1306_WHITE);
  } else {
    display.fillCircle(ix - 6, iy + 2, 6, SSD1306_WHITE);
    display.fillCircle(ix + 2, iy - 2, 8, SSD1306_WHITE);
    display.fillRoundRect(ix - 12, iy + 2, 27, 10, 5, SSD1306_WHITE);
    if (rainy && ((millis() / 350) % 2 == 0)) { display.drawLine(ix - 6, iy + 14, ix - 9, iy + 20, SSD1306_WHITE); display.drawLine(ix + 4, iy + 14, ix + 1, iy + 20, SSD1306_WHITE); }
  }
  display.setTextSize(1);
  centerText("LUCKNOW WEATHER", 2);
  display.drawFastHLine(0, 13, SCREEN_WIDTH, SSD1306_WHITE);

  String line1, line2;
  splitWeatherLine(lastWeatherText, line1, line2, 14);

  display.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;
  if (line2.length() == 0) {
    display.getTextBounds(line1.c_str(), 0, 0, &x1, &y1, &w, &h);
    display.setCursor(38 + max(0, (int)(90 - w) / 2), 29);
    display.print(line1);
  } else {
    display.getTextBounds(line1.c_str(), 0, 0, &x1, &y1, &w, &h);
    display.setCursor(38 + max(0, (int)(90 - w) / 2), 22);
    display.print(line1);
    display.getTextBounds(line2.c_str(), 0, 0, &x1, &y1, &w, &h);
    display.setCursor(38 + max(0, (int)(90 - w) / 2), 34);
    display.print(line2);
  }

  char age[24];
  getWeatherAgeString(age, sizeof(age));
  char ageLine[34];
  snprintf(ageLine, sizeof(ageLine), "Updated: %s", age);
  centerText(ageLine, 54);

  drawWifiHUD();
  drawModeLabelOverlayIfNeeded();
  display.display();
}

void renderBatteryScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  centerText("ENERGY", 2);
  display.drawFastHLine(0, 13, SCREEN_WIDTH, SSD1306_WHITE);

  char pctStr[8];
  snprintf(pctStr, sizeof(pctStr), "%d%%", batteryPct);
  display.setTextSize(2);
  centerText(pctStr, 17);
  display.drawRoundRect(18, 37, 92, 10, 3, SSD1306_WHITE);
  int fillW = map(batteryPct, 0, 100, 0, 86);
  if (fillW > 0) display.fillRoundRect(21, 40, fillW, 4, 2, SSD1306_WHITE);

  char voltStr[24];
  snprintf(voltStr, sizeof(voltStr), "Battery: %.2fV", batteryVoltage);
  display.setTextSize(1);
  centerText(voltStr, 50);

  char wifiStr[34];
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(wifiStr, sizeof(wifiStr), "WiFi: %s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(wifiStr, sizeof(wifiStr), "WiFi: offline");
  }
  centerText(wifiStr, 59);

  drawModeLabelOverlayIfNeeded();
  display.display();
}

// ============================================================
// BOOT SEQUENCE
// ============================================================
void bootSequence() {
  display.clearDisplay();
  display.setTextSize(2);
  centerText("HI!", 24);
  display.display();
  delay(1200);

  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 80); // kept for library completeness (unused for drawing)

  faceSetExpression(EXPR_SLEEPY);
  blinkAmount = 1.0f;
  blinkTarget = 1.0f;
  renderFace();

  startWifiConnect();
  lastWifiAttempt = millis();

  // gentle open
  unsigned long openStart = millis();
  while (millis() - openStart < 900) {
    blinkTarget = 0.0f;
    updateFaceTiming();
    renderFace();
    yield();
  }
  faceSetExpression(EXPR_NORMAL);

  unsigned long wifiWaitStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiWaitStart < 9000) {
    updateFaceTiming();
    renderFace();
    display.fillRect(0, SCREEN_HEIGHT - 12, SCREEN_WIDTH, 12, SSD1306_BLACK);
    display.setTextSize(1);
    centerText("Connecting WiFi...", SCREEN_HEIGHT - 10);
    display.display();
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiEverConnected = true;
    Serial.println();
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
    beginTimeSync();
    lastNtpSync = millis();

    unsigned long timeWaitStart = millis();
    while (!isTimeSynced() && millis() - timeWaitStart < 5000) {
      updateFaceTiming();
      renderFace();
      display.fillRect(0, SCREEN_HEIGHT - 12, SCREEN_WIDTH, 12, SSD1306_BLACK);
      centerText("Syncing time...", SCREEN_HEIGHT - 10);
      display.display();
      yield();
    }
    timeSynced = isTimeSynced();

    updateFaceTiming();
    renderFace();
    display.fillRect(0, SCREEN_HEIGHT - 12, SCREEN_WIDTH, 12, SSD1306_BLACK);
    centerText("Fetching weather...", SCREEN_HEIGHT - 10);
    display.display();
    fetchWeather();
    lastWeatherFetch = millis();
  } else {
    faceSetExpression(EXPR_SLEEPY);
    renderFace();
    display.fillRect(0, SCREEN_HEIGHT - 12, SCREEN_WIDTH, 12, SSD1306_BLACK);
    centerText("Offline mode", SCREEN_HEIGHT - 10);
    display.display();
    delay(900);
    faceSetExpression(EXPR_NORMAL);
  }

  faceSetExpression(EXPR_HAPPY);
  unsigned long readyStart = millis();
  while (millis() - readyStart < 700) { updateFaceTiming(); renderFace(); yield(); }
  faceSetExpression(EXPR_NORMAL);

  scheduleNextBlink();
  saccadePhaseStart = millis();

  beep(80);
}

// ============================================================
// MOTOR RAMP  (also updates the dead-reckoning headPosition estimate)
// ============================================================
void setMotorTarget(int dir, int speedPercent) {
  int actualDir = dir;
#if REVERSE_PAN_DIRECTION
  if (actualDir != 0) actualDir = -actualDir;
#endif
  motorDir = actualDir;
  if (actualDir != 0) lastNonZeroDir = actualDir;
  targetPWM = (dir == 0) ? 0 : speedToPWM(clampSpeed(speedPercent));
}

void updateMotorRamp() {
  if (millis() - lastRampStep < RAMP_INTERVAL_MS) return;
  lastRampStep = millis();

  if (currentPWM < targetPWM) currentPWM = min(currentPWM + RAMP_STEP_SIZE, targetPWM);
  else if (currentPWM > targetPWM) currentPWM = max(currentPWM - RAMP_STEP_SIZE, targetPWM);

  int dirToUse = (motorDir != 0) ? motorDir : lastNonZeroDir;

  if (currentPWM <= 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  } else if (dirToUse == 1) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  analogWrite(PWMB, currentPWM);

  // Dead reckoning: no encoder exists, so this accumulates actual applied
  // effort (not just elapsed time) as our best estimate of head position.
  if (currentPWM > 0) {
    headPosition += (long)((dirToUse == 1) ? currentPWM : -currentPWM);
  }
}

// Drives toward a target position using the headPosition estimate instead
// of a fixed duration, so ramp-up/ramp-down asymmetry can't silently drift
// the head off-center. Returns true once arrived (or safety-timeout hit).
bool driveTowardPosition(long target, int speedPercent, unsigned long phaseStart, unsigned long timeoutMs) {
  long error = target - headPosition;
  if (abs(error) <= CENTER_DEADBAND) {
    setMotorTarget(0, 0);
    return true;
  }
  if (millis() - phaseStart > timeoutMs) {
    // Safety fallback: stop here and accept this as "arrived" rather than
    // risk driving indefinitely if the estimate or deadband is off.
    setMotorTarget(0, 0);
    headPosition = target;
    return true;
  }
  setMotorTarget(error > 0 ? PAN_RIGHT : PAN_LEFT, speedPercent);
  return false;
}

// ============================================================
// GENTLE IDLE PAN (MODE_DEFAULT, head only)
// ============================================================
void updateIdlePan() {
  if (idlePanPhase == 0) {
    if (millis() - lastIdlePan > idlePanInterval) {
      idlePanPhase = 1;
      idlePanPhaseStart = millis();
      int dir = random(0, 2) == 0 ? PAN_LEFT : PAN_RIGHT;
      setMotorTarget(dir, IDLE_PAN_SPEED);
      faceSetLookTarget(dir == PAN_LEFT ? -0.6f : 0.6f, 0.05f);
    }
    return;
  }
  unsigned long elapsed = millis() - idlePanPhaseStart;
  if (idlePanPhase == 1 && elapsed > IDLE_PAN_GLANCE_MS) {
    idlePanPhase = 2; idlePanPhaseStart = millis();
  }
  if (idlePanPhase == 2) {
    if (driveTowardPosition(0, IDLE_PAN_SPEED, idlePanPhaseStart, POSITION_TRAVEL_TIMEOUT_MS)) {
      idlePanPhase = 0; idlePanPhaseStart = millis();
      lastIdlePan = millis();
      idlePanInterval = random(9000, 18000);
      faceSetLookTarget(0, 0);
    }
  }
}

// ============================================================
// GUARD MODE MOTION (position-targeted, not timer-based)
// ============================================================
void resetGuardMode() {
  guardPhase = GUARD_WAIT;
  guardPhaseStart = millis();
  setMotorTarget(0, 0);
  faceSetExpression(EXPR_NORMAL);
  faceSetLookTarget(0, 0);
}

void updateGuardMotion() {
  unsigned long elapsed = millis() - guardPhaseStart;
  switch (guardPhase) {
    case GUARD_WAIT:
      faceSetExpression(EXPR_CURIOUS);
      if (elapsed > GUARD_CENTER_WAIT_MS) {
        guardPhase = GUARD_GOTO_LEFT; guardPhaseStart = millis();
        faceSetLookTarget(-1.0f, -0.1f);
      }
      break;
    case GUARD_GOTO_LEFT:
      if (driveTowardPosition(-GUARD_SIDE_TARGET, GUARD_PAN_SPEED, guardPhaseStart, POSITION_TRAVEL_TIMEOUT_MS)) {
        guardPhase = GUARD_HOLD_LEFT; guardPhaseStart = millis();
        faceSetExpression(EXPR_ANGRY);
      }
      break;
    case GUARD_HOLD_LEFT:
      if (elapsed > GUARD_HOLD_MS) {
        guardPhase = GUARD_GOTO_RIGHT; guardPhaseStart = millis();
        faceSetExpression(EXPR_CURIOUS);
        faceSetLookTarget(0, 0);
      }
      break;
    case GUARD_GOTO_RIGHT:
      if (driveTowardPosition(GUARD_SIDE_TARGET, GUARD_PAN_SPEED, guardPhaseStart, POSITION_TRAVEL_TIMEOUT_MS * 2)) {
        guardPhase = GUARD_HOLD_RIGHT; guardPhaseStart = millis();
        faceSetExpression(EXPR_ANGRY);
      }
      break;
    case GUARD_HOLD_RIGHT:
      if (elapsed > GUARD_HOLD_MS) {
        guardPhase = GUARD_GOTO_CENTER; guardPhaseStart = millis();
        faceSetExpression(EXPR_NORMAL);
        faceSetLookTarget(0, 0);
      }
      break;
    case GUARD_GOTO_CENTER:
      if (driveTowardPosition(0, GUARD_PAN_SPEED, guardPhaseStart, POSITION_TRAVEL_TIMEOUT_MS)) {
        guardPhase = GUARD_WAIT; guardPhaseStart = millis();
      }
      break;
  }
}

// ============================================================
// DANCE MODE MOTION + BUZZER
// ------------------------------------------------------------
// Dance keeps a lively feel but every pause beat now actively eases
// the head back through the centered position (driveTowardPosition)
// instead of just letting the ramp coast to a stop wherever momentum
// left it, so drift can't accumulate beat-to-beat.
// ============================================================
void resetDanceMode() {
  dancePhase = DANCE_MOVE;
  dancePhaseStart = millis();
  dancePhaseDuration = random(DANCE_MOVE_MIN_MS, DANCE_MOVE_MAX_MS);
  lastDanceDir = (random(0, 2) == 0) ? PAN_LEFT : PAN_RIGHT;
  setMotorTarget(lastDanceDir, random(DANCE_MIN_SPEED, DANCE_MAX_SPEED + 1));
  discoStep = 0;
  discoNextBeepAt = 0;
  danceBeatCount = 0;
  faceSetExpression(EXPR_HAPPY);
  faceSetLookTarget(lastDanceDir == PAN_LEFT ? -0.8f : 0.8f, -0.2f);
}

void updateDanceMotion() {
  unsigned long elapsed = millis() - dancePhaseStart;
  if (dancePhase == DANCE_MOVE && elapsed > dancePhaseDuration) {
    dancePhase = DANCE_PAUSE; dancePhaseStart = millis();
    dancePhaseDuration = random(DANCE_PAUSE_MIN_MS, DANCE_PAUSE_MAX_MS);
    faceSetLookTarget(0, 0.1f);
  } else if (dancePhase == DANCE_PAUSE) {
    // ease back to center every beat instead of an abrupt stop
    bool arrived = driveTowardPosition(0, DANCE_MIN_SPEED, dancePhaseStart, POSITION_TRAVEL_TIMEOUT_MS);
    if (arrived && elapsed > dancePhaseDuration) {
      dancePhase = DANCE_MOVE; dancePhaseStart = millis();
      dancePhaseDuration = random(DANCE_MOVE_MIN_MS, DANCE_MOVE_MAX_MS);
      lastDanceDir = -lastDanceDir;
      setMotorTarget(lastDanceDir, random(DANCE_MIN_SPEED, DANCE_MAX_SPEED + 1));
      danceBeatCount++;
      faceSetLookTarget(lastDanceDir == PAN_LEFT ? -0.8f : 0.8f, -0.2f);
      if (danceBeatCount % 3 == 0) faceSetExpression(EXPR_SURPRISED);
      else faceSetExpression(EXPR_HAPPY);
    }
  }
}

void updateDanceBuzzer() {
  if (millis() >= discoNextBeepAt) {
    beep(DISCO_BEEP_MS[discoStep]);
    discoNextBeepAt = millis() + DISCO_BEEP_MS[discoStep] + DISCO_GAP_MS[discoStep];
    discoStep = (discoStep + 1) % DISCO_STEPS;
  }
}

// ============================================================
// SLEEP MODE
// ============================================================
void enterSleep() {
  isSleeping = true;
  screenMode = SCREEN_FACE;
  setMotorTarget(0, 0);
  faceSetExpression(EXPR_SLEEPY);
  faceSetLookTarget(0, 0);
  blinkAmount = 1.0f;
  blinkTarget = 1.0f;
  beep(120);
  Serial.println("Entering sleep. Tap to wake.");
}

void wakeUp() {
  isSleeping = false;
  manualWakeGraceUntil = millis() + WAKE_GRACE_MS;
  blinkTarget = 0.0f;
  applyModeEyeSettings(currentMode);
  beep(60);
  Serial.println("Waking up.");
}

// ============================================================
// MODE SWITCHING (D3 / GPIO0 button)
// ============================================================
void applyModeEyeSettings(RobotMode m) {
  switch (m) {
    case MODE_DEFAULT:
      faceSetExpression(EXPR_NORMAL);
      faceSetLookTarget(0, 0);
      scheduleNextBlink();
      break;
    case MODE_GUARD:
      faceSetExpression(EXPR_CURIOUS);
      faceSetLookTarget(0, 0);
      break;
    case MODE_CLOCK:
      faceSetExpression(EXPR_NORMAL);
      faceSetLookTarget(0, 0);
      scheduleNextBlink();
      break;
    case MODE_DANCE:
      faceSetExpression(EXPR_HAPPY);
      break;
    default:
      break;
  }
}

void enterMode(RobotMode m) {
  setMotorTarget(0, 0);
  screenMode = SCREEN_FACE;

  if (m == MODE_GUARD) resetGuardMode();
  if (m == MODE_DANCE) resetDanceMode();

  applyModeEyeSettings(m);

  // If the head has drifted from center, recenter before the new mode's
  // own animation starts, so the switch always ends with it facing you.
  if (abs(headPosition) > CENTER_DEADBAND) {
    recentering = true;
    recenterStart = millis();
  }

  snprintf(modeLabelText, sizeof(modeLabelText), "%s", MODE_NAMES[m]);
  modeLabelUntil = millis() + 1200;
  beep(60);
}

void onModeButtonPressed() {
  if (isSleeping) { wakeUp(); return; }
  currentMode = (RobotMode)((currentMode + 1) % MODE_COUNT);
  enterMode(currentMode);
}

void checkModeButton() {
  int reading = digitalRead(MODE_BUTTON_PIN);
  if (reading != buttonLastReading) {
    buttonLastDebounceTime = millis();
  }
  if (millis() - buttonLastDebounceTime > BUTTON_DEBOUNCE_MS) {
    if (reading != buttonStableState) {
      buttonStableState = reading;
      if (buttonStableState == LOW) onModeButtonPressed();
    }
  }
  buttonLastReading = reading;
}

// ============================================================
// TOUCH HANDLER
// ------------------------------------------------------------
// Short tap in DEFAULT triggers an emotional reaction. In CLOCK it cycles
// Time -> Weather -> Battery. Long press sleeps/wakes in any mode.
// ============================================================
void advanceInfoScreen() {
  if (currentMode != MODE_CLOCK) return;
  switch (screenMode) {
    case SCREEN_FACE: screenMode = SCREEN_TIME; break;
    case SCREEN_TIME: screenMode = SCREEN_WEATHER; break;
    case SCREEN_WEATHER: screenMode = SCREEN_BATTERY; manageWeather(); break;
    case SCREEN_BATTERY: screenMode = SCREEN_TIME; break;
  }
  screenModeUntil = millis() + INFO_SCREEN_MS;
}

void checkTouch() {
  bool currentTouch = digitalRead(TOUCH_PIN);
  bool currentTouch2 = digitalRead(TOUCH2_PIN);

  if (currentTouch == HIGH && lastTouchState == LOW) {
    touchDownTime = millis();
  }

  if (currentTouch == LOW && lastTouchState == HIGH) {
    unsigned long heldFor = millis() - touchDownTime;

    if (heldFor >= LONG_PRESS_MS) {
      if (isSleeping) wakeUp(); else enterSleep();
    } else if (isSleeping) {
      wakeUp();
    } else {
      if (currentMode == MODE_DEFAULT) startTouchReaction();
      else if (currentMode == MODE_CLOCK) { beep(25); advanceInfoScreen(); }
    }
  }
  lastTouchState = currentTouch;

  if (currentTouch2 == HIGH && lastTouch2State == LOW) touch2DownTime = millis();
  if (currentTouch2 == LOW && lastTouch2State == HIGH) {
    unsigned long heldFor = millis() - touch2DownTime;
    if (heldFor >= LONG_PRESS_MS) { if (isSleeping) wakeUp(); else enterSleep(); }
    else if (isSleeping) wakeUp();
    else if (currentMode == MODE_DEFAULT) { startFaceEffect(2, 2300); setMotorTarget(PAN_RIGHT, IDLE_PAN_SPEED); faceSetLookTarget(0.75f, -0.15f); }
    else if (currentMode == MODE_CLOCK) { beep(40); advanceInfoScreen(); }
  }
  lastTouch2State = currentTouch2;
  if (currentMode == MODE_DEFAULT && currentTouch == LOW && currentTouch2 == LOW && faceEffect != 3) { startFaceEffect(3, 1800); setMotorTarget(0, 0); beep(120); }

  if (screenMode != SCREEN_FACE && millis() > screenModeUntil) {
    screenMode = SCREEN_FACE;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(115200);
  delay(500);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);

  pinMode(TOUCH_PIN, INPUT);
  pinMode(TOUCH2_PIN, INPUT);
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  display.setTextColor(SSD1306_WHITE);
  // Defensive: some display-library init paths call a bare Wire.begin()
  // internally, which on ESP8266 defaults to GPIO4/GPIO5 - your BIN2/BIN1
  // motor pins. Re-assert our real pins/speed right after.
  Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
  Wire.setClock(400000);

  randomSeed(micros());
  updateBattery(true);

  bootSequence();

  currentMode = MODE_DEFAULT;
  applyModeEyeSettings(currentMode);
  headPosition = 0; // boot assumes the head starts physically centered
  lastIdlePan = millis();

  Serial.println("Table Robot V9 started.");
  Serial.println("Tap reacts in DEFAULT; CLOCK contains Time, Weather and Battery.");
  Serial.println("Hold = sleep/wake. D3 button = cycle mode.");
  Serial.println("NOTE: buzzer uses RX/GPIO3 - serial commands from PC disabled.");
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  checkModeButton();
  checkTouch();
  manageWifiAndTime();
  manageWeather();
  updateMotorRamp();
  updateBattery(false);
  updateBuzzer();

  if (isSleeping) {
    renderSleepFace();
    yield();
    return;
  }

  if (recentering) {
    bool arrived = driveTowardPosition(0, IDLE_PAN_SPEED, recenterStart, POSITION_TRAVEL_TIMEOUT_MS);
    renderFace();
    if (arrived) recentering = false;
    yield();
    return;
  }

  switch (currentMode) {
    case MODE_DEFAULT:
      if (screenMode == SCREEN_TIME) renderTimeScreen();
      else if (screenMode == SCREEN_WEATHER) renderWeatherScreen();
      else if (screenMode == SCREEN_BATTERY) renderBatteryScreen();
      else {
        updateTouchReaction();
        updateFaceEffect();
        updateIdlePan();
        if (idlePanPhase == 0) updateIdleSaccades();
        renderFace();
      }
      break;

    case MODE_GUARD:
      updateGuardMotion();
      renderFace();
      break;

    case MODE_CLOCK:
      if (screenMode == SCREEN_WEATHER) renderWeatherScreen();
      else if (screenMode == SCREEN_BATTERY) renderBatteryScreen();
      else renderTimeScreen();
      break;

    case MODE_DANCE:
      updateDanceMotion();
      updateDanceBuzzer();
      renderFace();
      break;

    default:
      break;
  }

  yield();
}
