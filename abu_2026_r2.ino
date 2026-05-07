#include <HardwareSerial.h>

HardwareSerial SerialPort(1);

const int rxPin = 27;
const int txPin = 26;

const int relayGPIO[] = { 18, 19, 22, 23 };
static bool cRelayState = false;
static bool dRelayState = false;

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, rxPin, txPin);

  //  รีเลย์
  for (int i = 0; i < 4; i++) {
    pinMode(relayGPIO[i], OUTPUT);
  }
  // digitalWrite(relayGPIO[2], LOW);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(33, OUTPUT);
  ledcSetup(0, 5000, 8);
  ledcAttachPin(33, 0);
}

void loop() {
  if (SerialPort.available()) {
    String cmd = SerialPort.readStringUntil('\n');
    cmd.trim();  // ลบ \r, \n และช่องว่างส่วนเกินออก
    char c = cmd.charAt(0);

    // Serial.print("Received: ");
    // Serial.println(cmd);

    switch (c) {
      case 'P':
        digitalWrite(relayGPIO[2], HIGH);

        break;

      case 'S':
        digitalWrite(relayGPIO[2], LOW);

        break;
      case 'T':
        cRelayState = !cRelayState;
        digitalWrite(relayGPIO[0], cRelayState ? HIGH : LOW);
        delay(500);
        dRelayState = !dRelayState;
        digitalWrite(relayGPIO[1], cRelayState ? HIGH : LOW);
        break;

      case 'C':
        cRelayState = !cRelayState;
        digitalWrite(relayGPIO[1], cRelayState ? HIGH : LOW);
        delay(500);
        break;

      case 'D':
        dRelayState = !dRelayState;
        digitalWrite(relayGPIO[0], dRelayState ? HIGH : LOW);
        delay(500);
        break;

      case 'M':
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
        ledcWrite(0, 100);
        break;

      case 'W':
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
        ledcWrite(0, 100);
        break;

      case 'X':
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        ledcWrite(0, 0);
        break;
    }
  }
}