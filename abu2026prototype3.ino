#include <ESP32Encoder.h>
#include <PID_v1.h>

// ==========================================
// พินสำหรับปุ่มกด (Buttons) - แบบกดติดปล่อยดับ
// ==========================================
const int btnSeq_Pin = 48;  // ปุ่ม Sequence

// ตัวแปรเก็บสถานะปุ่มก่อนหน้าเพื่อเช็คการกดแบบ Edge
bool lastBtnSeqState = HIGH;
unsigned long lastDebounceTime = 0;

// ==========================================
// พินที่เพิ่มใหม่ (Relay, Servo, Object Detect)
// ==========================================

// --- Relay 4 Channel (Output) ---
const int relay1_Pin = 39;
const int relay2_Pin = 40;
const int relay3_Pin = 41;
const int relay4_Pin = 42;

// --- Servo Motor (Output PWM) ---
const int servo_Pin = 20;

// --- Object Detect Sensor (Input) ---
const int objectDetect_Pin = 3;
int sensorState = 0;

// ====================================================================
// 1. กำหนดพินฮาร์ดแวร์สำหรับมอเตอร์ทั้ง 4 ล้อ
// ====================================================================

// --- หน้าซ้าย (Front Left - FL) ---
const int encFL_PinA = 16;
const int encFL_PinB = 17;
const int motorFL_IN1 = 4;
const int motorFL_IN2 = 5;
const int motorFL_PWM = 6;

// --- หน้าขวา (Front Right - FR) ---
const int encFR_PinA = 1;
const int encFR_PinB = 2;
const int motorFR_IN1 = 10;
const int motorFR_IN2 = 11;
const int motorFR_PWM = 12;

// --- หลังซ้าย (Rear Left - RL) ---
const int encRL_PinA = 18;
const int encRL_PinB = 21;
const int motorRL_IN1 = 7;
const int motorRL_IN2 = 8;
const int motorRL_PWM = 9;

// --- หลังขวา (Rear Right - RR) ---
const int encRR_PinA = 38;
const int encRR_PinB = 47;
const int motorRR_IN1 = 13;
const int motorRR_IN2 = 14;
const int motorRR_PWM = 15;

// ====================================================================
// 2. ประกาศออบเจ็กต์และตัวแปรสำหรับระบบควบคุม
// ====================================================================

ESP32Encoder encFL, encFR, encRL, encRR;

double inputFL = 0, outputFL = 0, setpointFL = 0;
double inputFR = 0, outputFR = 0, setpointFR = 0;
double inputRL = 0, outputRL = 0, setpointRL = 0;
double inputRR = 0, outputRR = 0, setpointRR = 0;

double Kp = 0.015;
double Ki = 0.05;
double Kd = 0.0;

PID pidFL(&inputFL, &outputFL, &setpointFL, Kp, Ki, Kd, DIRECT);
PID pidFR(&inputFR, &outputFR, &setpointFR, Kp, Ki, Kd, DIRECT);
PID pidRL(&inputRL, &outputRL, &setpointRL, Kp, Ki, Kd, DIRECT);
PID pidRR(&inputRR, &outputRR, &setpointRR, Kp, Ki, Kd, DIRECT);

int64_t lastCountFL = 0;
int64_t lastCountFR = 0;
int64_t lastCountRL = 0;
int64_t lastCountRR = 0;

unsigned long lastTime = 0;
const unsigned long sampleTime = 10;
bool isSystemRunning = true;  // เปิดระบบทันทีตั้งแต่เริ่มต้น

// ==========================================
// ตัวแปรและพารามิเตอร์สำหรับ Odometry
// ==========================================
double global_X = 0.0;
double global_Y = 0.0;
double global_Theta = 0.0;

const double WHEEL_RADIUS = 6.35;
const double PPR = 54000.0;
const double CM_PER_PULSE = (2.0 * PI * WHEEL_RADIUS) / PPR;
const double KINEMATIC_L = 36.5;

// ==========================================
// ตัวแปรสำหรับระบบนำทางอัตโนมัติ และ Sequence
// ==========================================
bool isAutoMoving = false;
double autoTargetDistance = 0.0;
double autoStartX = 0.0;
double autoStartY = 0.0;
bool isAutoTurning = false;
double autoTargetTheta = 0.0;

// ตัวแปรควบคุม State Machine (การทำงานตามลำดับ 9 ขั้นตอน)
int sequenceState = 0;
int sequenceSubState = 0;
unsigned long sequenceTimer = 0;

// ====================================================================
// ฟังก์ชันคำนวณตำแหน่งหุ่นยนต์ในโลกจริง
// ====================================================================
void updateOdometry() {
  double pFL = -inputFL;
  double pRL = -inputRL;
  double pFR = inputFR;
  double pRR = inputRR;

  double dFL = pFL * CM_PER_PULSE;
  double dFR = pFR * CM_PER_PULSE;
  double dRL = pRL * CM_PER_PULSE;
  double dRR = pRR * CM_PER_PULSE;

  double dX_robot = (dFL + dFR + dRL + dRR) / 4.0;
  double dY_robot = (-dFL + dFR + dRL - dRR) / 4.0;
  double dTheta_robot = (-dFL + dFR - dRL + dRR) / (4.0 * KINEMATIC_L);

  double cosTheta = cos(global_Theta);
  double sinTheta = sin(global_Theta);

  double dX_global = (dX_robot * cosTheta) - (dY_robot * sinTheta);
  double dY_global = (dX_robot * sinTheta) + (dY_robot * cosTheta);

  global_X += dX_global;
  global_Y += dY_global;
  global_Theta += dTheta_robot;

  if (global_Theta > PI) global_Theta -= 2.0 * PI;
  if (global_Theta < -PI) global_Theta += 2.0 * PI;
}

void setServoAngle(int angle) {
  if (angle < 40) angle = 40;
  if (angle > 110) angle = 110;
  // แปลงองศา 0-180 เป็นค่า PWM (ประมาณ 102 - 512 สำหรับ 12-bit)
  int pwmDuty = map(angle, 0, 180, 102, 512);
  ledcWrite(servo_Pin, pwmDuty);
}

void setup() {
  Serial.begin(115200);

  pinMode(btnSeq_Pin, INPUT);

  // ตั้งค่าพิน Relay และ Object Detect
  pinMode(relay1_Pin, OUTPUT);
  pinMode(relay2_Pin, OUTPUT);
  pinMode(relay3_Pin, OUTPUT);
  pinMode(relay4_Pin, OUTPUT);

  // ปิด Relay ไว้เริ่มต้น
  digitalWrite(relay1_Pin, HIGH);
  digitalWrite(relay2_Pin, HIGH);
  digitalWrite(relay3_Pin, HIGH);
  digitalWrite(relay4_Pin, HIGH);

  // ตั้งค่า Sensor
  pinMode(objectDetect_Pin, INPUT);

  // ตั้งค่า Encoder
  encFL.attachFullQuad(encFL_PinA, encFL_PinB);
  encFL.setFilter(100);
  encFL.clearCount();
  encFR.attachFullQuad(encFR_PinA, encFR_PinB);
  encFR.setFilter(100);
  encFR.clearCount();
  encRL.attachFullQuad(encRL_PinA, encRL_PinB);
  encRL.setFilter(100);
  encRL.clearCount();
  encRR.attachFullQuad(encRR_PinA, encRR_PinB);
  encRR.setFilter(100);
  encRR.clearCount();

  setupMotorPins(motorFL_IN1, motorFL_IN2, motorFL_PWM, 0);
  setupMotorPins(motorFR_IN1, motorFR_IN2, motorFR_PWM, 1);
  setupMotorPins(motorRL_IN1, motorRL_IN2, motorRL_PWM, 2);
  setupMotorPins(motorRR_IN1, motorRR_IN2, motorRR_PWM, 3);

  // ตั้งค่า Servo (ไม่ใช้ไลบรารี)
  pinMode(servo_Pin, OUTPUT);
  digitalWrite(servo_Pin, LOW);
  ledcAttachChannel(servo_Pin, 50, 12, 4);  // ตั้งความถี่ 50Hz, ความละเอียด 12-bit
  delay(200);
  setServoAngle(40);  // สั่งให้ไปที่องศาเริ่มต้น
  delay(500);

  initPID(&pidFL);
  initPID(&pidFR);
  initPID(&pidRL);
  initPID(&pidRR);

  // === เริ่มการทำงานของระบบทันทีเมื่อบอร์ดเริ่มทำงาน (Auto-Start) ===
  encFL.clearCount();
  encFR.clearCount();
  encRL.clearCount();
  encRR.clearCount();
  lastCountFL = 0;
  lastCountFR = 0;
  lastCountRL = 0;
  lastCountRR = 0;
  lastTime = millis();
  pidFL.SetMode(AUTOMATIC);
  pidFR.SetMode(AUTOMATIC);
  pidRL.SetMode(AUTOMATIC);
  pidRR.SetMode(AUTOMATIC);
  driveKinematics(0, 0, 0);

  Serial.println("==============================================");
  Serial.println("[STATUS] บอร์ด ESP32-S3 ระบบ 4 ล้อ พร้อม (AUTO-STARTED)");
  Serial.println("  'seq'       - เริ่มต้นทำงาน 9 ลำดับอัตโนมัติ");
  Serial.println("==============================================");

  delay(100);
  lastBtnSeqState = digitalRead(btnSeq_Pin);
}

void loop() {
  // ==========================================================
  // 1. ระบบตรวจสอบการกดปุ่ม (Button Edge Detection & Debounce)
  // ==========================================================
  bool currentBtnSeq = digitalRead(btnSeq_Pin);

  // --- ตรวจสอบปุ่ม Seq ---
  if (lastBtnSeqState == LOW && currentBtnSeq == HIGH) {

    // เช็คว่าการกดครั้งนี้ ห่างจากการกดครั้งที่แล้วมากกว่า 200 มิลลิวินาทีหรือไม่ (กรองสัญญาณสวิง)
    if (millis() - lastDebounceTime > 200) {
      lastDebounceTime = millis();  // บันทึกเวลาที่กดปุ่มล่าสุดเอาไว้

      if (isSystemRunning && sequenceState == 0) {
        Serial.println("\n[SEQ] === เริ่มต้นกระบวนการ (BUTTON) ===");
        sequenceState = 1;
        sequenceSubState = 0;
      }
    }
  }
  lastBtnSeqState = currentBtnSeq;

  // ==========================================================
  // 2. ระบบตรวจสอบคำสั่งจาก Serial
  // ==========================================================
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // เผื่อต้องการสั่งผ่านคอมพิวเตอร์แทนปุ่ม
    if (cmd.equalsIgnoreCase("seq")) {
      currentBtnSeq = LOW;
      lastBtnSeqState = HIGH;
    }
    // คำสั่งเคลื่อนที่แบบ Manual
    else if (isSystemRunning) {
      if (cmd.startsWith("go ")) {
        double target_cm = cmd.substring(3).toDouble();
        autoTargetDistance = target_cm;
        autoStartX = global_X;
        autoStartY = global_Y;
        isAutoMoving = true;
        resumePID();
        driveKinematics(1200.0, 0.0, 0.0);
      } else if (cmd.startsWith("turn ")) {
        double target_deg = cmd.substring(5).toDouble();
        double target_rad = target_deg * (PI / 180.0);
        autoTargetTheta = global_Theta + target_rad;

        if (autoTargetTheta > PI) autoTargetTheta -= 2.0 * PI;
        if (autoTargetTheta < -PI) autoTargetTheta += 2.0 * PI;

        isAutoTurning = true;
        isAutoMoving = false;
        resumePID();
        double turnSpeed = (target_deg > 0) ? 800.0 : -800.0;
        driveKinematics(0.0, 0.0, turnSpeed);
      }
    }
  }

  // ==========================================================
  // 3. ระบบควบคุมหลัก
  // ==========================================================
  if (!isSystemRunning) return;

  unsigned long now = millis();
  if (now - lastTime >= sampleTime) {

    int64_t currentCountFL = encFL.getCount();
    inputFL = (double)(currentCountFL - lastCountFL);
    lastCountFL = currentCountFL;
    pidFL.Compute();
    moveMotor(motorFL_IN1, motorFL_IN2, motorFL_PWM, outputFL);

    int64_t currentCountFR = encFR.getCount();
    inputFR = (double)(currentCountFR - lastCountFR);
    lastCountFR = currentCountFR;
    pidFR.Compute();
    moveMotor(motorFR_IN1, motorFR_IN2, motorFR_PWM, outputFR);

    int64_t currentCountRL = encRL.getCount();
    inputRL = (double)(currentCountRL - lastCountRL);
    lastCountRL = currentCountRL;
    pidRL.Compute();
    moveMotor(motorRL_IN1, motorRL_IN2, motorRL_PWM, outputRL);

    int64_t currentCountRR = encRR.getCount();
    inputRR = (double)(currentCountRR - lastCountRR);
    lastCountRR = currentCountRR;
    pidRR.Compute();
    moveMotor(motorRR_IN1, motorRR_IN2, motorRR_PWM, outputRR);

    lastTime = now;
    updateOdometry();

    // เช็คระยะทางเดินหน้า
    if (isAutoMoving) {
      double dX = global_X - autoStartX;
      double dY = global_Y - autoStartY;
      double currentDist = sqrt((dX * dX) + (dY * dY));

      if (currentDist >= autoTargetDistance) {
        hardStop();
        isAutoMoving = false;
        Serial.println("\n[AUTO] ---- ถึงระยะทางเป้าหมายแล้ว! ----");
      }
    }

    // เช็คมุมหมุนตัว
    if (isAutoTurning) {
      double errorTheta = autoTargetTheta - global_Theta;
      if (errorTheta > PI) errorTheta -= 2.0 * PI;
      if (errorTheta < -PI) errorTheta += 2.0 * PI;

      if (abs(errorTheta) < 0.05) {
        hardStop();
        isAutoTurning = false;
        Serial.println("\n[AUTO] ---- หมุนถึงองศาเป้าหมายแล้ว! ----");
      } else {
        double turnSpeed = errorTheta * 500.0;
        if (turnSpeed > 800.0) turnSpeed = 800.0;
        if (turnSpeed < -800.0) turnSpeed = -800.0;

        if (turnSpeed > 0 && turnSpeed < 200.0) turnSpeed = 200.0;
        if (turnSpeed < 0 && turnSpeed > -200.0) turnSpeed = -200.0;

        driveKinematics(0.0, 0.0, turnSpeed);
      }
    }

    // ดำเนินการตาม Sequence (ถ้ามีการกดปุ่ม seq)
    runSequence();
  }
}

// ====================================================================
// ฟังก์ชันทำงานตามลำดับ (State Machine)
// ====================================================================
void runSequence() {
  if (sequenceState == 0) return;

  switch (sequenceState) {
    case 1:  // ลำดับ 1 & 2: ตั้งค่า Servo เป็น 0 พร้อมเดินหน้า 82cm
      Serial.println("[SEQ] 1. ตั้งค่า Servo ไปที่ 0 องศา");
      autoStartX = global_X;
      autoStartY = global_Y;
      autoTargetDistance = 96.0;
      isAutoMoving = true;
      resumePID();
      driveKinematics(1100.0, 0.0, 0.0);
      Serial.println("[SEQ] 1. เริ่มเดินหน้า 82 cm...");
      sequenceState = 2;
      break;

    case 2:  // รอหุ่นยนต์เดินหน้าถึงเป้าหมาย
      if (!isAutoMoving) {
        sequenceSubState = 0;
        sequenceTimer = millis();
        sequenceState = 3;
      }
      break;

    case 3:  // ลำดับ 3 ถึง 5 (หลังจากเดินหน้าถึงจุดหมายแล้ว)
      if (sequenceSubState == 0 && millis() - sequenceTimer >= 1000) {
        Serial.println("[SEQ] 3. สั่ง Relay 39 เป็น HIGH");
        digitalWrite(relay1_Pin, LOW);
        sequenceTimer = millis();
        sequenceSubState = 1;
      } else if (sequenceSubState == 1 && millis() - sequenceTimer >= 1500) {  // หน่วงเวลา 1.5 วินาที
        Serial.println("[SEQ] 4. สั่ง Relay 40 เป็น HIGH");
        digitalWrite(relay2_Pin, LOW);
        sequenceTimer = millis();
        sequenceSubState = 2;
      } else if (sequenceSubState == 2 && millis() - sequenceTimer >= 500) {
        setServoAngle(105);
        Serial.println("[SEQ] 5. ตั้งค่า Servo ไปที่ 120 องศา");
        sequenceTimer = millis();
        sequenceSubState = 3;
      } else if (sequenceSubState == 3 && millis() - sequenceTimer >= 500) {  // รอเซอโวหมุนเสร็จ
        sequenceState = 4;
      }
      break;

    case 4:  // ลำดับ 6: ถอยหลัง 82cm
      autoStartX = global_X;
      autoStartY = global_Y;
      autoTargetDistance = 50.0;
      isAutoMoving = true;
      digitalWrite(relay2_Pin, HIGH);
      resumePID();
      driveKinematics(-1200.0, 0.0, 0.0);
      Serial.println("[SEQ] 6. เริ่มถอยหลัง 82 cm...");
      sequenceState = 5;
      break;

    case 5:  // รอหุ่นยนต์ถอยหลังถึงเป้าหมาย
      if (!isAutoMoving) {
        sequenceState = 6;
      }
      break;

    case 6:  // ลำดับ 7: หมุนตัวกลับด้าน 180 องศา
      autoTargetTheta = global_Theta + PI;
      if (autoTargetTheta > PI) autoTargetTheta -= 2.0 * PI;
      if (autoTargetTheta < -PI) autoTargetTheta += 2.0 * PI;

      isAutoTurning = true;
      resumePID();
      driveKinematics(0.0, 0.0, 800.0);
      Serial.println("[SEQ] 7. หมุนตัวกลับด้าน 180 องศา...");
      sequenceState = 7;
      break;

    case 7:  // รอจนกว่าจะหมุนเสร็จ
      if (!isAutoTurning) {
        Serial.println("[SEQ] 8. รอสัญญาณจาก Pin 48...");
        sequenceState = 8;
      }
      break;

    case 8:  // ลำดับ 8: รอสัญญาณจาก pin 3
      if (digitalRead(objectDetect_Pin) == HIGH) {
        Serial.println("[SEQ] ได้รับสัญญาณจาก Pin 3 แล้ว!");
        sequenceState = 9;
      }
      break;

    case 9:  // ลำดับ 9: สั่ง relay pin 39 เป็น HIGH
      Serial.println("[SEQ] 9. สั่ง Relay 39 เป็น HIGH");
      digitalWrite(relay1_Pin, HIGH);

      // เปลี่ยนจากเดิมที่ให้ sequenceState = 0 ไปทำสเต็ปการถอยหลังต่อไป
      sequenceState = 10;
      break;

    case 10:  // ลำดับ 10: ถอยหลัง 30 cm
      autoStartX = global_X;
      autoStartY = global_Y;
      autoTargetDistance = 30.0;  // ตั้งเป้าหมาย 30 cm
      isAutoMoving = true;
      resumePID();
      driveKinematics(-1200.0, 0.0, 0.0);  // ความเร็วแกน X ถอยหลัง
      Serial.println("[SEQ] 10. เริ่มถอยหลัง 30 cm...");
      sequenceState = 11;
      break;

    case 11:  // รอหุ่นยนต์ถอยหลังถึงเป้าหมาย
      if (!isAutoMoving) {
        sequenceState = 12;
      }
      break;

    case 12:                                        // ลำดับ 11: เลี้ยวขวา 45 องศา (-PI/4 เรเดียน)
      autoTargetTheta = global_Theta - (PI / 4.0);  // มุมปัจจุบัน ลบ 45 องศา

      // ปรับมุมให้อยู่ในช่วง -PI ถึง PI เสมอเพื่อป้องกันบั๊กคำนวณ
      if (autoTargetTheta > PI) autoTargetTheta -= 2.0 * PI;
      if (autoTargetTheta < -PI) autoTargetTheta += 2.0 * PI;

      isAutoTurning = true;
      resumePID();
      driveKinematics(0.0, 0.0, -800.0);  // ความเร็วในการเลี้ยวขวา (Omega เป็นค่าลบ)
      Serial.println("[SEQ] 11. หมุนเลี้ยวขวา 90 องศา...");
      sequenceState = 13;
      break;

    case 13:  // รอจนกว่าจะเลี้ยวเสร็จ
      if (!isAutoTurning) {
        sequenceState = 14;
      }
      break;

    case 14:  // ลำดับ 12: เดินหน้า 250 cm
      autoStartX = global_X;
      autoStartY = global_Y;
      autoTargetDistance = 250.0;  // ตั้งเป้าหมาย 250 cm
      isAutoMoving = true;
      resumePID();
      driveKinematics(1200.0, 0.0, 0.0);  // ความเร็วแกน X เดินหน้า
      Serial.println("[SEQ] 12. เริ่มเดินหน้า 300 cm...");
      sequenceState = 15;
      break;

    case 15:  // รอหุ่นยนต์เดินหน้า 250 cm จนถึงเป้าหมาย
      if (!isAutoMoving) {
        // เมื่อเดินหน้า 250 cm เสร็จแล้ว ให้ไปทำสเต็ปเลี้ยวขวาต่อทันที
        sequenceState = 16;
      }
      break;

    case 16:                                        // ลำดับ 13: หันขวาเพิ่มอีก 45 องศา (-PI/4 เรเดียน)
      autoTargetTheta = global_Theta - (PI / -4.0);  // เอาองศาปัจจุบันลบออก 45 องศา

      // ปรับช่วงมุมให้อยู่ในช่วง -PI ถึง PI เสมอ เพื่อความแม่นยำของ Odometry
      if (autoTargetTheta > PI) autoTargetTheta -= 2.0 * PI;
      if (autoTargetTheta < -PI) autoTargetTheta += 2.0 * PI;

      isAutoTurning = true;
      resumePID();
      driveKinematics(0.0, 0.0, -800.0);  // สั่งหมุนขวา (Omega เป็นค่าลบ)
      Serial.println("[SEQ] 13. หมุนเลี้ยวขวาเพิ่มอีก 45 องศา...");
      sequenceState = 17;
      break;

    case 17:  // รอจนกว่าจะหมุนเลี้ยวขวา 45 องศาเสร็จ
      if (!isAutoTurning) {
        sequenceState = 18;
      }
      break;

    case 18:  // ลำดับ 14: เดินหน้าต่อไปอีก 50 cm
      autoStartX = global_X;
      autoStartY = global_Y;
      autoTargetDistance = 50.0;  // ตั้งเป้าหมายระยะทาง 50 cm
      isAutoMoving = true;
      resumePID();
      driveKinematics(1200.0, 0.0, 0.0);  // เดินหน้าตรงตามแนวทิศทางใหม่
      Serial.println("[SEQ] 14. เริ่มเดินหน้าต่อไปอีก 50 cm...");
      sequenceState = 19;
      break;

    case 19:  // รอหุ่นยนต์เดินหน้า 50 cm จนถึงเป้าหมาย
      if (!isAutoMoving) {
        Serial.println("[SEQ] === จบการทำงานตามลำดับทั้งหมด (สมบูรณ์) ===");
        sequenceState = 0;  // เคลียร์สเตตกลับเป็น 0 เพื่อจบกระบวนการและรอรับปุ่มกดรอบใหม่
      }
      break;
  }
}

// ====================================================================
// ฟังก์ชันขับเคลื่อนและฟังก์ชันย่อยอื่นๆ
// ====================================================================
void driveKinematics(double Vx, double Vy, double Omega) {
  double targetFL = Vx - Vy - Omega;
  double targetFR = Vx + Vy + Omega;
  double targetRL = Vx + Vy - Omega;
  double targetRR = Vx - Vy + Omega;

  setpointFL = -targetFL;
  setpointRL = -targetRL;
  setpointFR = targetFR;
  setpointRR = targetRR;
}

void setupMotorPins(int in1, int in2, int pwmPin, int channel) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  ledcAttachChannel(pwmPin, 20000, 8, channel);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  ledcWrite(pwmPin, 0);
}

void initPID(PID* targetPID) {
  targetPID->SetMode(MANUAL);
  targetPID->SetSampleTime(sampleTime);
  targetPID->SetOutputLimits(-120, 120);
}

void moveMotor(int in1, int in2, int pwmPin, double pidOutput) {
  int pwmValue = abs((int)pidOutput);

  if (pidOutput > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, pwmValue);

  } else if (pidOutput < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(pwmPin, pwmValue);

  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(pwmPin, 0);
  }
}

void hardStop() {
  pidFL.SetMode(MANUAL);
  outputFL = 0;

  pidFR.SetMode(MANUAL);
  outputFR = 0;
  pidRL.SetMode(MANUAL);
  outputRL = 0;
  pidRR.SetMode(MANUAL);
  outputRR = 0;

  moveMotor(motorFL_IN1, motorFL_IN2, motorFL_PWM, 0);
  moveMotor(motorFR_IN1, motorFR_IN2, motorFR_PWM, 0);

  moveMotor(motorRL_IN1, motorRL_IN2, motorRL_PWM, 0);
  moveMotor(motorRR_IN1, motorRR_IN2, motorRR_PWM, 0);

  driveKinematics(0, 0, 0);
}

void resumePID() {
  if (pidFL.GetMode() == MANUAL) {
    pidFL.SetMode(AUTOMATIC);
    pidFR.SetMode(AUTOMATIC);
    pidRL.SetMode(AUTOMATIC);
    pidRR.SetMode(AUTOMATIC);
  }
}