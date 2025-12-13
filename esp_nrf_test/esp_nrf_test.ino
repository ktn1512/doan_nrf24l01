#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 22
#define CSN_PIN 21
#define LED_BAO_LOI 2 // Đèn báo trạng thái kết nối

RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  pinMode(12, INPUT_PULLDOWN);
  pinMode(14, INPUT_PULLDOWN);
  pinMode(26, INPUT_PULLDOWN);
  pinMode(27, INPUT_PULLDOWN);
  pinMode(LED_BAO_LOI, OUTPUT);
  digitalWrite(LED_BAO_LOI, HIGH); 

  if (!radio.begin()) { Serial.println("❌ NRF24 Loi!"); while (1); }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void sendCommand(const char *cmd) {
  bool ket_noi_tot = radio.write(cmd, strlen(cmd) + 1);

  if (ket_noi_tot) {
    digitalWrite(LED_BAO_LOI, LOW); 
  } else {
    digitalWrite(LED_BAO_LOI, HIGH);
  }
}

void loop() {
  if (digitalRead(12) == HIGH) sendCommand("tien");
  else if (digitalRead(14) == HIGH) sendCommand("lui");
  else if (digitalRead(26) == HIGH) sendCommand("trai");
  else if (digitalRead(27) == HIGH) sendCommand("phai");
  else {
    sendCommand("ping");
  }
  
  delay(50);
}
