#define IR_SENSOR 2
#define BUZZER 3
#define LED 4
#define TRIG_PIN 5
#define ECHO_PIN 6
#define TRIG_PIN2 7
#define ECHO_PIN2 8

bool isLocked = false;

void setup() {
  pinMode(IR_SENSOR, INPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  Serial.begin(9600);
}

void loop() {
  // Check IR for lock/unlock
  if (digitalRead(IR_SENSOR) == LOW) {
    isLocked = !isLocked;
    digitalWrite(BUZZER, HIGH);
    delay(300);
    digitalWrite(BUZZER, LOW);
    Serial.println("LOCK:" + String(isLocked));
    delay(1000);
  }

  // Handle CLICK signals from Python
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command == "LEFT_CLICK" || command == "RIGHT_CLICK") {
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED, HIGH);
        delay(200);
        digitalWrite(LED, LOW);
        delay(200);
      }
    }
  }

  // Read distance from first HC-SR04 (scroll)
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration1 = pulseIn(ECHO_PIN, HIGH);
  long distance1 = duration1 * 0.034 / 2;
  Serial.print("DIST1:");
  Serial.println(distance1);

  // Read distance from second HC-SR04 (volume)
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  long duration2 = pulseIn(ECHO_PIN2, HIGH);
  long distance2 = duration2 * 0.034 / 2;
  Serial.print("DIST2:");
  Serial.println(distance2);

  delay(200);
}