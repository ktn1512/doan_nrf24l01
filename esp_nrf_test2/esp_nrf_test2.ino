// 1 -> GND
// 2 -> 3V3
// 3 -> 22(CE)
// 4 -> 21(CS)
// 5 -> 18(SCK)
// 6 -> 23(MOSI)
// 7 -> 19(MISO)

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 22
#define CSN_PIN 21

#define PIN_R 4
#define PIN_G 5
#define PIN_B 16

#define PWM_FREQ 5000
#define PWM_RES 8

RF24 radio(CE_PIN, CSN_PIN);

const byte tx_address[6] = { 0x30, 0x30, 0x30, 0x30, 0x31 };
const byte rx_address[6] = { 0x30, 0x30, 0x30, 0x30, 0x32 };

void setup() {
  Serial.begin(115200);
  ledcAttach(PIN_R, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_G, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_B, PWM_FREQ, PWM_RES);

  ledcWrite(PIN_B, 0);

  //4 chân điều khiển xe
  pinMode(12, INPUT_PULLDOWN);
  pinMode(14, INPUT_PULLDOWN);
  pinMode(27, INPUT_PULLDOWN);
  pinMode(26, INPUT_PULLDOWN);

  //bật flash
  pinMode(17, INPUT_PULLDOWN);  //bỏ

  //tăng tốc
  pinMode(2, INPUT_PULLDOWN);

  //giảm tốc
  pinMode(13, INPUT_PULLDOWN);

  if (!radio.begin()) {
    Serial.println("LOI!");
    while (1) {
    }
  } else {
    Serial.println("Success");
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.disableDynamicPayloads();
  radio.setPayloadSize(32);
  radio.openWritingPipe(tx_address);
  radio.openReadingPipe(1, rx_address);
  radio.setRetries(15, 15);
  radio.stopListening();
  Serial.println("ESP32 Ready to Send!");
}

void sendCommand(const char *cmd) {
  radio.stopListening();

  bool ket_noi_tot = radio.write(cmd, strlen(cmd) + 1);

  radio.startListening();

  if (ket_noi_tot) {
    ledcWrite(PIN_R, 0);
    ledcWrite(PIN_G, 0);
    ledcWrite(PIN_B, 0);
  } else {
    Serial.println("matketnoi");
  }
}
int voltToPercent(float volt) {
  if (volt >= 12.0f) return 100;
  if (volt <= 10.0f) return 10;

  return map((int)(volt * 100),
             1000, 1200,
             10, 100);
}

void setColorByPercent(int percent) {
  int r, g;

  if (percent > 100) percent = 100;
  if (percent < 10) percent = 10;

  if (percent < 55) {
    r = 255;
    g = map(percent, 10, 55, 0, 255);
  } else {
    g = 255;
    r = map(percent, 55, 100, 255, 0);
  }

  ledcWrite(PIN_R, r);
  ledcWrite(PIN_G, g);
}

void loop() {
  static float receive_volt = 0;
  static unsigned long lastRxTime = 0;

  if (digitalRead(12) == HIGH) {
    sendCommand("tien");
  } else if (digitalRead(14) == HIGH) {
    sendCommand("lui");
  } else if (digitalRead(26) == HIGH) {
    sendCommand("trai");
  } else if (digitalRead(27) == HIGH) {
    sendCommand("phai");
  } else if (digitalRead(17) == HIGH) {
    sendCommand("tang");
  } else if (digitalRead(13) == HIGH) {
    sendCommand("giam");
  } else if (digitalRead(2) == HIGH) {
    sendCommand("flash");
  } else {
    sendCommand("ping");
  }

  unsigned long start = millis();
  while (millis() - start < 50) {
    if (radio.available()) {
      char buffer[32];
      radio.read(&buffer, sizeof(buffer));
      memcpy(&receive_volt, buffer, sizeof(float));
      lastRxTime = millis();
      Serial.print("Pin: ");
      Serial.print(receive_volt);
      Serial.println("V");
      break;
    }
  }

  if (receive_volt > 5.0 && receive_volt < 20.0) {
    int percent = voltToPercent(receive_volt);
    setColorByPercent(percent);
  }

  if (millis() - lastRxTime > 3000 && lastRxTime != 0) {
    ledcWrite(PIN_R, 255);
    ledcWrite(PIN_G, 0);
    ledcWrite(PIN_B, 0);
  }

  delay(50);
}