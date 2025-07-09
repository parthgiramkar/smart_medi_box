#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>

#define buzzerPin 7
#define emergencyBtn 9
#define irSensorPin 8

LiquidCrystal_I2C lcd(0x27, 16, 2);

// For 28BYJ-48 stepper motor with ULN2003 driver
const int stepsPerRevolution = 2048;  // Full revolution steps
Stepper myStepper(stepsPerRevolution, A0, A2, A1, A3);  // IN1, IN3, IN2, IN4

bool morning = false, afternoon = false, evening = false;
bool timingsEntered = false;

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.backlight();

  pinMode(buzzerPin, OUTPUT);
  pinMode(emergencyBtn, INPUT_PULLUP);
  pinMode(irSensorPin, INPUT);

  myStepper.setSpeed(15);  // Stepper motor speed (RPM)

  // Motor test
  Serial.println("Testing motor...");
  myStepper.step(stepsPerRevolution / 8);  // Rotate ~45 degrees
  delay(1000);
  myStepper.step(-stepsPerRevolution / 8); // Rotate back
  Serial.println("Motor test done.");

  welcomeMessage();
  getMedicationTimings();
}

void loop() {
  if (!timingsEntered) return;

  handleTiming("Morning", 10000, morning);
  handleTiming("Afternoon", 10000, afternoon);
  handleTiming("Evening", 10000, evening);

  emergencyCheck();

  morning = afternoon = evening = false;
  timingsEntered = false;

  getMedicationTimings();
}

void welcomeMessage() {
  lcd.setCursor(0, 0);
  lcd.print("   WELCOME");
  lcd.setCursor(0, 1);
  lcd.print("SMART MEDI BOX");
  delay(3000);
  lcd.clear();
}

void getMedicationTimings() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter timings...");
  Serial.println("Send A (Morning), B (Afternoon), C (Evening). Send D when done.");

  while (true) {
    if (Serial.available()) {
      char input = Serial.read();
      input = toupper(input);
      switch (input) {
        case 'A':
          morning = true;
          Serial.println("Morning time selected.");
          break;
        case 'B':
          afternoon = true;
          Serial.println("Afternoon time selected.");
          break;
        case 'C':
          evening = true;
          Serial.println("Evening time selected.");
          break;
        case 'D':
          Serial.println("Medication timings saved.");
          timingsEntered = true;
          lcd.clear();
          return;
      }
    }
  }
}

void handleTiming(String timing, unsigned long delayTime, bool active) {
  if (active) {
    delay(delayTime);
    beepBuzzer(2);
    rotateMotor(timing);
    checkIRSensor();
  }
}

void beepBuzzer(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(buzzerPin, HIGH);
    delay(200);
    digitalWrite(buzzerPin, LOW);
    delay(200);
  }
}

void rotateMotor(String timingLabel) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dispensing:");
  lcd.setCursor(0, 1);
  lcd.print(timingLabel);

  Serial.println("Dispensing: " + timingLabel);
  myStepper.step(stepsPerRevolution / 8);  // Rotate ~45 degrees

  delay(2000);
}

void checkIRSensor() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Waiting pickup...");
  Serial.println("Checking IR sensor...");

  unsigned long startTime = millis();
  bool taken = false;

  while (millis() - startTime < 10000) {
    if (digitalRead(irSensorPin) == LOW) {
      taken = true;
      break;
    }
  }

  lcd.clear();
  if (taken) {
    lcd.setCursor(0, 0);
    lcd.print("Pill Taken");
    Serial.println("Pill taken.");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Pill NOT Taken");
    Serial.println("Pill NOT taken.");
    beepBuzzer(3);
  }

  delay(3000);
  lcd.clear();
}

void emergencyCheck() {
  if (digitalRead(emergencyBtn) == LOW) {
    Serial.println("!!! EMERGENCY ALERT !!!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("!!!EMERGENCY!!!");
    beepBuzzer(5);
    delay(3000);
    lcd.clear();
  }
}