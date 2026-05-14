#include <Servo.h>

#define SERVO_PIN 15

const int START_US = 1000;
const int STEP_US  = 5;
const int MAX_US   = 2000;

Servo servo;
int currentUs;

void setup() {
  Serial.begin(115200);
  delay(1500);

  servo.attach(SERVO_PIN);
  currentUs = START_US;
  servo.writeMicroseconds(currentUs);

  Serial.println("=== Servo Calibration ===");
  Serial.print("Starting at ");
  Serial.print(currentUs);
  Serial.println(" us (min throttle)");
  Serial.print("Step size: ");
  Serial.print(STEP_US);
  Serial.println(" us");
  Serial.println("Press SPACE to step up, 'r' to reset to start");
  Serial.println("=========================================");
  printPosition();
}

void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();

  if (c == ' ') {
    int next = currentUs + STEP_US;
    if (next > MAX_US) {
      Serial.println("!! Already at hard limit (2000 us) !!");
      return;
    }
    currentUs = next;
    servo.writeMicroseconds(currentUs);
    printPosition();
  } else if (c == 'r' || c == 'R') {
    currentUs = START_US;
    servo.writeMicroseconds(currentUs);
    Serial.println(">> Reset to start");
    printPosition();
  }
}

void printPosition() {
  float pct = (currentUs - 1000.0f) / (2000.0f - 1000.0f) * 100.0f;
  Serial.print("Servo: ");
  Serial.print(currentUs);
  Serial.print(" us  (");
  Serial.print(pct, 1);
  Serial.println("%)");
}
