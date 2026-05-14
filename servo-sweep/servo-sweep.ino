#include <Servo.h>

#define SERVO_PIN 15

const int MIN_US     = 1000;
const int MAX_US     = 2000;
const int CYCLE_MS   = 3000;
const int UPDATE_MS  = 20;

Servo servo;
uint32_t cycleStart;

void setup() {
  Serial.begin(115200);
  delay(1500);

  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(MIN_US);

  Serial.println("=== Servo Sweep ===");
  Serial.print("Range: ");
  Serial.print(MIN_US);
  Serial.print(" - ");
  Serial.print(MAX_US);
  Serial.print(" us, cycle: ");
  Serial.print(CYCLE_MS / 1000.0f, 1);
  Serial.println(" s");

  cycleStart = millis();
}

void loop() {
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();

  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  uint32_t elapsed = (now - cycleStart) % (CYCLE_MS * 2);

  int us;
  if (elapsed < (uint32_t)CYCLE_MS) {
    float t = elapsed / (float)CYCLE_MS;
    us = MIN_US + (int)(t * (MAX_US - MIN_US));
  } else {
    float t = (elapsed - CYCLE_MS) / (float)CYCLE_MS;
    us = MAX_US - (int)(t * (MAX_US - MIN_US));
  }

  servo.writeMicroseconds(us);

  Serial.print("Servo: ");
  Serial.print(us);
  Serial.println(" us");
}
