#include <Servo.h>

Servo myservo;

// {IN1, IN2, PWM}
const int motorPins[4][3] = {
  { 22, 23, 2 },  // Front Left หน้าซ้าย
  { 26, 27, 4 },  // Rear Left หลังซ้าย
  { 24, 25, 3 },  // Front Right หน้าขวา
  { 28, 29, 5 }   // Rear Right  หลังขวา
};

const int pwmSpeed = 20;

void setup() {
  Serial.begin(115200);

  // ตั้งค่าขา GPIO และ PWM
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    pinMode(motorPins[i][2], OUTPUT);
  }
  // ตั้งค่าขา Relay
  pinMode(12, OUTPUT);

  // ตั้งค่าขา Servo motor
  myservo.attach(9);
  myservo.write(90);
}

void loop() {
  if (Serial.available() > 0) {    // ตรวจสอบว่ามีข้อมูลส่งเข้ามาหรือไม่
    char command = Serial.read();  // อ่านตัวอักษรที่ส่งมา 1 ตัว

    if (command == 'f') {
      moveForward();
      delay(3300);
      stopAllMotors();
      delay(1500);
      strafeLeft();
      delay(2000);
      stopAllMotors();
    } else if (command == 'b') {
      moveBackward();
    } else if (command == 's') {
      stopAllMotors();
    }
  }
}

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

// void strafeLeft() {
//   moveMotor(0, true, 40);   // FL
//   moveMotor(1, false, 40);  // RL
//   moveMotor(2, true, 40);   // FR
//   moveMotor(3, false, 40);  // RR
// }

void strafeLeft() {
  moveMotor(0, true, 40);   // FL
  moveMotor(1, false, 40);  // RL
  moveMotor(2, true, 40);   // FR
  moveMotor(3, true, 40);  // RR
}

void strafeRight() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, true, pwmSpeed);   // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, true, pwmSpeed);   // RR
}

// void rotateRight() {
//   moveMotor(0, true, pwmSpeed);  // FL
//   moveMotor(1, true, pwmSpeed);  // RL
//   moveMotor(2, true, pwmSpeed);  // FR
//   moveMotor(3, true, pwmSpeed);  // RR
// }

// void rotateLeft() {
//   moveMotor(0, false, pwmSpeed);  // FL
//   moveMotor(1, false, pwmSpeed);  // RL
//   moveMotor(2, false, pwmSpeed);  // FR
//   moveMotor(3, false, pwmSpeed);  // RR
// }

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i][0], LOW);
    digitalWrite(motorPins[i][1], LOW);
    analogWrite(motorPins[i][2], 0);
  }
}
