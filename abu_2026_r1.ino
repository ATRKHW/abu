#include <ps5Controller.h>
#include <HardwareSerial.h>
#include <ESP32Servo.h>

HardwareSerial SerialPort(1);
Servo myServo;

const int rxPin = 26;
const int txPin = 27;

static const int servoPin = 32;

const int pwmSpeed = 255;  // ความเร็วมอเตอร์ (0-255)

// {IN1, IN2, PWM}
const int motorPins[4][3] = {
  { 4, 5, 33 },    // Front Left หน้าซ้าย
  { 18, 19, 13 },  // Rear Left หลังซ้าย
  { 21, 22, 14 },  // Front Right หน้าขวา
  { 23, 25, 15 }   // Rear Right  หลังขวา
};

unsigned long lastActionTime = 0;
const unsigned long actionDelay = 500;  // หน่วงการกดปุ่มซ้ำ

void setup() {
  ps5.begin("10:18:49:80:58:30");
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, rxPin, txPin);

  // ตั้งค่าขา GPIO และ PWM
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i][0], OUTPUT);
    pinMode(motorPins[i][1], OUTPUT);
    // ledcSetup(i, 5000, 8);
    // ledcAttachPin(motorPins[i][2], i);
    ledcAttach(motorPins[i][2], 5000, 8);
  }
  // อนุญาตให้ใช้ Timer สำหรับ PWM
  ESP32PWM::allocateTimer(3);
  myServo.setPeriodHertz(50);  // มาตรฐาน Servo คือ 50Hz

  // เชื่อมต่อ Servo เข้ากับขาที่กำหนด
  // RDS51160 มักมีช่วง Pulse width ที่ 500us - 2500us สำหรับ 180 หรือ 270 องศา
  myServo.attach(servoPin, 500, 2500);
  myServo.write(90);
}

// ===== ฟังก์ชันควบคุมการเคลื่อนไหวแบบ Mecanum =====
void moveMotor(int index, bool direction, int speed) {
  digitalWrite(motorPins[index][0], direction ? HIGH : LOW);
  digitalWrite(motorPins[index][1], direction ? LOW : HIGH);
  ledcWrite(motorPins[index][2], speed);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i][0], LOW);
    digitalWrite(motorPins[i][1], LOW);
    ledcWrite(motorPins[i][2], 0);
  }
}

void moveForward() {
  moveMotor(0, true, pwmSpeed);   // FL
  moveMotor(1, true, pwmSpeed);   // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, false, pwmSpeed);  // RR
}

void moveBackward() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, false, pwmSpeed);  // RL
  moveMotor(2, true, pwmSpeed);   // FR
  moveMotor(3, true, pwmSpeed);   // RR
}

void strafeLeft() {
  moveMotor(0, true, pwmSpeed);   // FL
  moveMotor(1, false, pwmSpeed);  // RL
  moveMotor(2, true, pwmSpeed);   // FR
  moveMotor(3, false, pwmSpeed);  // RR
}

void strafeRight() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, true, pwmSpeed);   // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, true, pwmSpeed);   // RR
}

void rotateRight() {
  moveMotor(0, true, pwmSpeed);  // FL
  moveMotor(1, true, pwmSpeed);  // RL
  moveMotor(2, true, pwmSpeed);  // FR
  moveMotor(3, true, pwmSpeed);  // RR
}

void rotateLeft() {
  moveMotor(0, false, pwmSpeed);  // FL
  moveMotor(1, false, pwmSpeed);  // RL
  moveMotor(2, false, pwmSpeed);  // FR
  moveMotor(3, false, pwmSpeed);  // RR
}

// ========== LOOP ==========
void loop() {
  if (!ps5.isConnected()) return;

  unsigned long now = millis();

  // กดปุ่มสื่อสารอื่นๆ (มี delay)
  if (now - lastActionTime > actionDelay) {
    if (ps5.Touchpad()) {
      SerialPort.println("Pad");
      myServo.write(90);
      lastActionTime = now;
    } else if (ps5.Square()) {
      SerialPort.println("Square");
      delay(900);
      myServo.write(150);
      lastActionTime = now;
    } else if (ps5.Triangle()) {
      SerialPort.println("Triangle");
      lastActionTime = now;
    } else if (ps5.Circle()) {
      SerialPort.println("Circle");
      lastActionTime = now;
    } else if (ps5.R1()) {
      SerialPort.println("Drop");
      lastActionTime = now;
    } else if (ps5.R2()) {
      SerialPort.println("Release");
      lastActionTime = now;
    } else if (ps5.Cross()) {
      if (myServo.read() == 90) {
        myServo.write(150);
      } else {
        myServo.write(90);
      }
      lastActionTime = now;
    }
  }

  // ควบคุมการเคลื่อนไหว
  if (ps5.Up()) {
    // Serial.println("Forward");
    moveForward();
  } else if (ps5.Down()) {
    // Serial.println("Backward");
    moveBackward();
  } else if (ps5.Left()) {
    // Serial.println("Strafe Left");
    strafeLeft();
  } else if (ps5.Right()) {
    // Serial.println("Strafe Right");
    strafeRight();
  } else if (ps5.LStickY() > 100) {
    SerialPort.println("M");
  } else if (ps5.LStickY() < -100) {
    SerialPort.println("W");
  } else if (ps5.RStickX() > 100) {
    // Serial.printf("Rotate Right: %d\n", ps5.LStickX());
    rotateRight();
  } else if (ps5.RStickX() < -100) {
    // Serial.printf("Rotate Left: %d\n", ps5.LStickX());
    rotateLeft();
  } else if (ps5.L1()) {
    SerialPort.println("O");
  } else {
    stopAllMotors();
    SerialPort.println("X");
  }
}
