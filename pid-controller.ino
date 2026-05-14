#include <Servo.h>
#include <math.h>
#include <hardware/watchdog.h>

// ===================== Pins =====================
#define RPM_PIN   22
#define POT_PIN   26
#define MODE_PIN  16
#define SERVO_PIN 15
#define LED_PIN   25

// ===================== Serial / RPM =====================
#define SERIAL_BAUD 115200
#define PULSES_PER_REV 4

const uint32_t MIN_PULSE_INTERVAL_US = 1500;
const uint32_t MAX_PULSE_INTERVAL_US = 500000;
const uint32_t RPM_TIMEOUT_US        = 2000000;

// ===================== Target RPM mapping =====================
const float RPM_MIN_TARGET = 530.0f;
const float RPM_MAX_TARGET = 2500.0f;

// ===================== Servo calibration =====================
const int SERVO_US_MIN = 1072;
const int SERVO_US_MAX = 1732;

// Manual anti-jitter
const int SERVO_STEP_US = 5;              // ignore changes smaller than this
const uint32_t SERVO_UPDATE_MS = 50;      // 20 Hz update rate

// Startup shake
const float STARTUP_SHAKE_PCT_MIN = 0.0f;
const float STARTUP_SHAKE_PCT_MAX = 10.0f;
const int STARTUP_SHAKE_CYCLES = 3;
const int STARTUP_SHAKE_DELAY_MS = 120;

// ===================== Timing =====================
const uint32_t CONTROL_INTERVAL_MS = 50;
const uint32_t PRINT_INTERVAL_MS = 200;

// ===================== RPM smoothing =====================
const float RPM_ALPHA = 0.25f;

// ===================== PID =====================
float Kp = 0.3f;

const float ERROR_DEADBAND_RPM = 10.0f;
const float I_ACTIVE_BAND_RPM  = 250.0f;
const float I_LEAK_PER_SEC     = 0.30f;
const float ITERM_US_MIN       = -180.0f;
const float ITERM_US_MAX       =  180.0f;

const float PID_OUT_MIN = -350.0f;
const float PID_OUT_MAX =  350.0f;

// ===================== Feed-forward =====================
struct Pt { float rpm; int us; };

const Pt FF_NEUTRAL[] = {
  {  530.0f, 1150 },
  {  800.0f, 1204 },
  { 1500.0f, 1402 },
  { 2000.0f, 1567 },
  { 2500.0f, 1732 },
};

// Adaptive offset
float ffOffsetUs = 0.0f;
const float FF_OFFSET_MIN_US = -120.0f;
const float FF_OFFSET_MAX_US =  300.0f;
const float FF_ADAPT_GAIN_US_PER_RPM_S = 0.06f;
const float FF_ADAPT_ACTIVE_BAND_RPM   = 200.0f;
const float FF_OFFSET_LEAK_PER_SEC     = 0.05f;

// ===================== RPM ISR state =====================
volatile uint32_t lastPulseMicros    = 0;
volatile uint32_t goodIntervalMicros = 0;
volatile uint32_t pulseCountAccepted = 0;
volatile bool newPulse = false;

// ===================== Runtime state =====================
Servo throttleServo;

float rpmFilt = 0.0f;
float iTermUs = 0.0f;
float prevError = 0.0f;

int lastServoUs = SERVO_US_MIN;

// ===================== Helpers =====================
static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int servoUsFromPercent(float pct) {
  pct = clampf(pct, 0.0f, 100.0f);
  return SERVO_US_MIN + (int)((pct / 100.0f) * (SERVO_US_MAX - SERVO_US_MIN));
}

uint16_t readPotAveraged() {
  uint32_t acc = 0;
  for (int i = 0; i < 16; i++) {
    acc += analogRead(POT_PIN);
  }
  return (uint16_t)(acc / 16);
}

static int ff_us(float targetRpm) {
  const int N = (int)(sizeof(FF_NEUTRAL) / sizeof(FF_NEUTRAL[0]));

  if (targetRpm <= FF_NEUTRAL[0].rpm) return FF_NEUTRAL[0].us;
  if (targetRpm >= FF_NEUTRAL[N - 1].rpm) return FF_NEUTRAL[N - 1].us;

  for (int i = 0; i < N - 1; i++) {
    float x0 = FF_NEUTRAL[i].rpm;
    float x1 = FF_NEUTRAL[i + 1].rpm;

    if (targetRpm >= x0 && targetRpm <= x1) {
      float t = (targetRpm - x0) / (x1 - x0);
      int y0 = FF_NEUTRAL[i].us;
      int y1 = FF_NEUTRAL[i + 1].us;
      return (int)(y0 + t * (y1 - y0));
    }
  }

  return FF_NEUTRAL[N - 1].us;
}

// ===================== RPM ISR =====================
void rpmPulseISR() {
  uint32_t now = micros();
  uint32_t dt = now - lastPulseMicros;
  lastPulseMicros = now;

  if (dt < MIN_PULSE_INTERVAL_US) return;
  if (dt > MAX_PULSE_INTERVAL_US) return;

  goodIntervalMicros = dt;
  pulseCountAccepted++;
  newPulse = true;
}

// ===================== RPM computation =====================
float computeRPM() {
  noInterrupts();
  uint32_t interval = goodIntervalMicros;
  uint32_t lastP = lastPulseMicros;
  bool hasNew = newPulse;
  newPulse = false;
  interrupts();

  uint32_t ageUs = micros() - lastP;

  float rpm;
  if (ageUs >= RPM_TIMEOUT_US) {
    rpm = 0.0f;
  } else if (hasNew && interval > 0) {
    rpm = 60.0e6f / (interval * PULSES_PER_REV);
  } else {
    rpm = rpmFilt;
  }

  if (rpmFilt == 0.0f) rpmFilt = rpm;
  rpmFilt = rpmFilt + RPM_ALPHA * (rpm - rpmFilt);

  return rpmFilt;
}

void writeServoFiltered(int desiredUs) {
  desiredUs = clampi(desiredUs, SERVO_US_MIN, SERVO_US_MAX);

  if (abs(desiredUs - lastServoUs) >= SERVO_STEP_US) {
    lastServoUs = desiredUs;
    throttleServo.writeMicroseconds(lastServoUs);
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  analogReadResolution(12);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(RPM_PIN, INPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpmPulseISR, FALLING);

  throttleServo.attach(SERVO_PIN);

  int us0  = servoUsFromPercent(STARTUP_SHAKE_PCT_MIN);
  int us10 = servoUsFromPercent(STARTUP_SHAKE_PCT_MAX);

  throttleServo.writeMicroseconds(us0);
  lastServoUs = us0;
  delay(200);

  for (int i = 0; i < STARTUP_SHAKE_CYCLES; i++) {
    throttleServo.writeMicroseconds(us10);
    lastServoUs = us10;
    delay(STARTUP_SHAKE_DELAY_MS);

    throttleServo.writeMicroseconds(us0);
    lastServoUs = us0;
    delay(STARTUP_SHAKE_DELAY_MS);
  }

  throttleServo.writeMicroseconds(SERVO_US_MIN);
  lastServoUs = SERVO_US_MIN;

  Serial.println("rpm,target_rpm,pot_percent,pot_raw,mode,servo_us,pid_us,ff_us,ff_offset_us,i_us,pulses,interval_us,age_us");

  watchdog_enable(500, true);  // 500ms timeout, auto-pause on debug
}

// ===================== Loop =====================
void loop() {
  static uint32_t lastControlMs = 0;
  static uint32_t lastPrintMs = 0;
  static uint32_t lastBlinkMs = 0;
  static uint32_t lastServoUpdateMs = 0;
  static bool ledState = false;

  watchdog_update();

  uint32_t nowMs = millis();

  // Heartbeat
  if (nowMs - lastBlinkMs >= 250) {
    lastBlinkMs = nowMs;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }

  if (lastControlMs == 0) lastControlMs = nowMs;
  if (lastPrintMs == 0) lastPrintMs = nowMs;

  // Pot
  uint16_t potRaw = readPotAveraged();
  float potPercent = (potRaw / 4095.0f) * 100.0f;
  potPercent = clampf(potPercent, 0.0f, 100.0f);

  float targetRPM = RPM_MIN_TARGET +
                    (potPercent / 100.0f) * (RPM_MAX_TARGET - RPM_MIN_TARGET);

  bool manualMode = (digitalRead(MODE_PIN) == LOW);

  float rpm = computeRPM();

  int ffBaseUs = ff_us(targetRPM);
  int servoNominalUs = clampi((int)(ffBaseUs + ffOffsetUs), SERVO_US_MIN, SERVO_US_MAX);

  int servoUs = servoNominalUs;
  float pidUs = 0.0f;

  if (nowMs - lastControlMs >= CONTROL_INTERVAL_MS) {
    float dt = (nowMs - lastControlMs) / 1000.0f;
    lastControlMs = nowMs;

    if (manualMode) {
      int manualUs = servoUsFromPercent(potPercent);
      servoUs = manualUs;

      iTermUs = 0.0f;
      prevError = 0.0f;
      ffOffsetUs = 0.0f;
      pidUs = 0.0f;
    } else {
      float error = targetRPM - rpm;

      if (error > -ERROR_DEADBAND_RPM && error < ERROR_DEADBAND_RPM) {
        error = 0.0f;
      }

      float pTermUs = Kp * error;

      bool inIband = (fabsf(error) < I_ACTIVE_BAND_RPM);

      float uPre = pTermUs + iTermUs;
      bool satHigh = (uPre >= PID_OUT_MAX);
      bool satLow  = (uPre <= PID_OUT_MIN);

      bool allowIntegrate = inIband &&
        !((satHigh && error > 0.0f) || (satLow && error < 0.0f));

      bool signFlip = (prevError != 0.0f) &&
        ((error > 0.0f) != (prevError > 0.0f));
      prevError = error;

      if (allowIntegrate && dt > 0) {
        const float KI_US_PER_RPM_S = 0.020f;
        iTermUs += (KI_US_PER_RPM_S * error) * dt;
      } else if (dt > 0) {
        float leak = 1.0f - clampf(I_LEAK_PER_SEC * dt, 0.0f, 0.95f);
        iTermUs *= leak;
      }

      if (signFlip && dt > 0) {
        float extra = clampf(3.0f * I_LEAK_PER_SEC * dt, 0.0f, 0.98f);
        iTermUs *= (1.0f - extra);
      }

      iTermUs = clampf(iTermUs, ITERM_US_MIN, ITERM_US_MAX);

      pidUs = pTermUs + iTermUs;
      pidUs = clampf(pidUs, PID_OUT_MIN, PID_OUT_MAX);

      int candidate = servoNominalUs + (int)pidUs;
      servoUs = clampi(candidate, SERVO_US_MIN, SERVO_US_MAX);

      if (servoUs != candidate) {
        iTermUs *= 0.97f;
      }

      // Adaptive feed-forward offset
      if (dt > 0) {
        float offLeak = 1.0f - clampf(FF_OFFSET_LEAK_PER_SEC * dt, 0.0f, 0.20f);
        ffOffsetUs *= offLeak;

        if (fabsf(error) < FF_ADAPT_ACTIVE_BAND_RPM) {
          ffOffsetUs += (FF_ADAPT_GAIN_US_PER_RPM_S * error) * dt;
          ffOffsetUs = clampf(ffOffsetUs, FF_OFFSET_MIN_US, FF_OFFSET_MAX_US);
        }
      }
    }

    // Rate-limited / deadbanded servo update
    if (nowMs - lastServoUpdateMs >= SERVO_UPDATE_MS) {
      lastServoUpdateMs = nowMs;
      writeServoFiltered(servoUs);
    }
  }

  // Serial output — only print if USB is connected and buffer has space
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;

    if (Serial && Serial.availableForWrite() > 80) {
      noInterrupts();
      uint32_t pc = pulseCountAccepted;
      uint32_t ivUs = goodIntervalMicros;
      uint32_t lastP = lastPulseMicros;
      interrupts();

      uint32_t ageUs = micros() - lastP;

      Serial.print(rpm, 1);
      Serial.print(",");
      Serial.print(targetRPM, 1);
      Serial.print(",");
      Serial.print(potPercent, 1);
      Serial.print(",");
      Serial.print(potRaw);
      Serial.print(",");
      Serial.print(manualMode ? "MANUAL" : "AUTO");
      Serial.print(",");
      Serial.print(lastServoUs);
      Serial.print(",");
      Serial.print(pidUs, 1);
      Serial.print(",");
      Serial.print(ffBaseUs);
      Serial.print(",");
      Serial.print(ffOffsetUs, 1);
      Serial.print(",");
      Serial.print(iTermUs, 1);
      Serial.print(",");
      Serial.print(pc);
      Serial.print(",");
      Serial.print(ivUs);
      Serial.print(",");
      Serial.println(ageUs);
    }
  }
}
