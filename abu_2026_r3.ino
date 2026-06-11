#include <Servo.h>

Servo myservo;

// {IN1, IN2, PWM}
const int motorPins[4][3] = {
  { 22, 23, 3 },   // Front Left หน้าซ้าย
  { 26, 27, 4 },   // Rear Left หลังซ้าย
  { 24, 25, 10 },  // Front Right หน้าขวา
  { 28, 29, 8 }    // Rear Right  หลังขวา
};

const int pwmSpeed = 20;
const int sensorPin = 7;
const int relayPin = 12;
int sensorState = 0;

// ตัวแปรสำหรับจัดการ millis()
unsigned long previousMillis = 0;
int currentStep = 0;
bool stepInit = true; 

void setup() {
  Serial.begin(115200);

  // ตั้งค่าขา GPIO และ PWM
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    pinMode(motorPins[i][2], OUTPUT);
  }

  // ตั้งค่าขา Relay เริ่มต้นให้เป็น HIGH (ยังไม่ทำงาน)
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  // Retrorefective
  pinMode(sensorPin, INPUT_PULLUP);

  // ตั้งค่าขา Servo motor
  myservo.attach(9);
  myservo.write(90);
}

void loop() {
  unsigned long currentMillis = millis();

  // อ่านค่าเซ็นเซอร์ตลอดเวลา (หากต้องการใช้ในเงื่อนไขอื่นเพิ่มเติม)
  sensorState = digitalRead(sensorPin);

  // State Machine สำหรับจัดการลำดับการทำงานแทน delay()
  switch (currentStep) {
    
    case 0: // 1. เดินหน้า (moveForward) เป็นเวลา 3040 ms
      if (stepInit) {
        moveForward();
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 3040) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 1: // 2. หยุดมอเตอร์ทั้งหมด (stopAllMotors) เป็นเวลา 500 ms
      if (stepInit) {
        stopAllMotors();
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 2: // 3. สั่งให้ Relay เป็น LOW (เริ่มทำงาน) เป็นเวลา 500 ms
      if (stepInit) {
        digitalWrite(relayPin, LOW); // <--- รีเลย์เริ่มเป็น LOW ที่นี่
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 3: // 4. หมุนเซอร์โวไปที่ 180 องศา (myservo.write) เป็นเวลา 500 ms
      if (stepInit) {
        myservo.write(180);
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 4: // 5. ถอยหลัง (moveBackward) เป็นเวลา 2000 ms
      if (stepInit) {
        moveBackward();
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 2000) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 5: // 6. หยุดมอเตอร์ชั่วครู่และเลี้ยวขวา (rotateRight) เป็นเวลา 2800 ms
      if (stepInit) {
        stopAllMotors(); 
        rotateRight();
        stepInit = false;
      }
      if (currentMillis - previousMillis >= 2800) {
        previousMillis = currentMillis;
        currentStep++;
        stepInit = true;
      }
      break;

    case 6: // 7. ขั้นตอนสุดท้าย: หยุดการทำงานทั้งหมดและสั่งรีเลย์เป็น HIGH
      if (stepInit) {
        stopAllMotors();              // หยุดมอเตอร์ทั้งหมด
        digitalWrite(relayPin, HIGH); // <--- รีเลย์กลับเป็น HIGH ที่นี่
        stepInit = false;
        Serial.println("Sequence Finished. System Stopped.");
        // ระบบจะค้างอยู่ที่ท่านีตลอดไป ไม่วนกลับไปสเตปแรก
      }
      break;
  }
}

// ==========================================
// ส่วนฟังก์ชันควบคุมมอเตอร์ คงไว้เหมือนเดิมทั้งหมด
// ==========================================

void moveMotor(int index, bool direction, int speed) {
  digitalWrite(motorPins[index][0], direction ? HIGH : LOW);
  digitalWrite(motorPins[index][1], direction ? LOW : HIGH);
  analogWrite(motorPins[index][2], speed);
}

void moveForward() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, false, pwmSpeed);  // RL
  moveMotor(2, true, pwmSpeed);   // FR
  moveMotor(3, true, pwmSpeed);   // RR
}

void moveBackward() {
  moveMotor(0, true, pwmSpeed);   // FL
  moveMotor(1, true, pwmSpeed);   // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, false, pwmSpeed);  // RR
}

void strafeLeft() {
  moveMotor(0, true, 40);   // FL
  moveMotor(1, false, 40);  // RL
  moveMotor(2, true, 40);   // FR
  moveMotor(3, true, 40);   // RR
}

void strafeRight() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, true, pwmSpeed);   // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, true, pwmSpeed);   // RR
}

void rotateRight() {
  moveMotor(0, true, 35);  // FL
  moveMotor(1, true, 35);  // RL
  moveMotor(2, true, 35);  // FR
  moveMotor(3, true, 35);  // RR
}

void rotateLeft() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, false, pwmSpeed);  // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, false, pwmSpeed);  // RR
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i][0], LOW);
    digitalWrite(motorPins[i][1], LOW);
    analogWrite(motorPins[i][2], 0);
  }
}