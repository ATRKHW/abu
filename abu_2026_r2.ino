#include <HardwareSerial.h>

HardwareSerial SerialPort(1);

const int rxPin = 27;
const int txPin = 26;

const int relayGPIO[] = { 18, 19, 22, 23, 16, 17};
static bool cRelayState = false;
static bool rRelayState = false;
static bool dRelayState = false;
static bool relay1State = false;

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, rxPin, txPin);

  //  รีเลย์
  for (int i = 0; i < 6; i++) {
    pinMode(relayGPIO[i], OUTPUT);
    digitalWrite(relayGPIO[i], HIGH);
  }
  // digitalWrite(relayGPIO[2], LOW);

  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(33, OUTPUT);
  // ledcSetup(0, 5000, 8);
  // ledcAttachPin(33, 0);
  ledcAttach(33, 5000, 8); // ระบุแค่ Pin 33, Freq 5000 Hz, Resolution 8 bit

  pinMode(14, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  // ledcSetup(0, 5000, 8);
  // ledcAttachPin(33, 0);
  ledcAttach(13, 5000, 8); // ระบุแค่ Pin 33, Freq 5000 Hz, Resolution 8 bit
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
        digitalWrite(relayGPIO[2], LOW);
        digitalWrite(relayGPIO[3], LOW);
        digitalWrite(relayGPIO[5], LOW);

        break;

      case 'S':
        digitalWrite(relayGPIO[5], HIGH);
        delay(500);
        digitalWrite(relayGPIO[2], HIGH);

        break;

      case 'R':
        rRelayState = !rRelayState;
        digitalWrite(relayGPIO[0], rRelayState ? HIGH : LOW);

        break;

      case 'T':
        relay1State = !relay1State;
        digitalWrite(relayGPIO[3], relay1State ? HIGH : LOW);
        break;

      case 'C':
        cRelayState = !cRelayState;
        digitalWrite(relayGPIO[4], cRelayState ? HIGH : LOW);
        break;

      case 'D':
        dRelayState = !dRelayState;
        digitalWrite(relayGPIO[1], dRelayState ? HIGH : LOW);
        break;

      case 'M':
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
        ledcWrite(33, 255);
        break;

      case 'W':
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
        ledcWrite(33, 255);
        break;

      case 'O':
        digitalWrite(14, LOW);
        digitalWrite(12, HIGH);
        ledcWrite(13, 255);
        break;

      case 'X':
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        ledcWrite(33, 0);
        digitalWrite(14, LOW);
        digitalWrite(12, LOW);
        ledcWrite(13, 0);
        break;
    }
  }
}