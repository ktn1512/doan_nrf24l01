// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #define CE_PIN 22
// #define CSN_PIN 21
// #define SEND_PIN 4     // chân bạn muốn đọc (ví dụ GPIO4)
// /*
// ======NRF2401L=========
// 1 -> GND
// 2 -> 3V3
// 3 -> 22(CE)
// 4 -> 21(CS)
// 5 -> 18(SCK)
// 6 -> 23(MOSI)
// 5 -> 19(MISO)
// */
// RF24 radio(CE_PIN, CSN_PIN);
// const byte address[6] = "00001";

// void setup() {
//   Serial.println("✔ SPI OK, tìm thấy NRF24L01");
//   Serial.begin(9600);
//   Serial.println("ESP32 NRF24L01 Transmitter");
//   pinMode(16, INPUT_PULLDOWN);
//   pinMode(SEND_PIN, INPUT_PULLDOWN);  // mặc định = 0, khi nhấn/kéo lên 1 mới gửi
//   pinMode(12, INPUT_PULLDOWN);
//   pinMode(14, INPUT_PULLDOWN);
//   pinMode(26, INPUT_PULLDOWN);
//   pinMode(27, INPUT_PULLDOWN);
//   if (!radio.begin()) {
//     Serial.println("NRF24 init failed!");
//     while (1);
//   }

//   radio.openWritingPipe(address);
//   radio.openReadingPipe(1, address);
//   radio.setPALevel(RF24_PA_LOW);
//   radio.setDataRate(RF24_250KBPS);
//   radio.stopListening(); // chế độ phát
// }

// void loop() {
//   if(digitalRead(12)==1){
//     const char ch[] = "tien";
//     radio.write(&ch, sizeof(ch));
//     Serial.println("sent: tien");
//     delay(50);
//   }

//   if(digitalRead(14)==1){
//     const char ch[] = "lui";
//     radio.write(&ch, sizeof(ch));
//     Serial.println("sent: lui");
//     delay(50);
//   }

//   if(digitalRead(26)==1){
//     const char ch[] = "trai";
//     radio.write(&ch, sizeof(ch));
//     Serial.println("sent: trai");
//     delay(50);
//   }
  
//   if (digitalRead(27) == HIGH) {
//     const char text[] = "phai";
//     radio.write(&text, sizeof(text));
//     Serial.println("Sent: phai");
//     delay(50);  // tránh gửi liên tục quá nhanh
//   }
// }



// ====== TRANSMITTER (TAY CẦM - ESP32) ======
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
  digitalWrite(LED_BAO_LOI, HIGH); // Mới vào coi như mất kết nối

  if (!radio.begin()) { Serial.println("❌ NRF24 Loi!"); while (1); }
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void sendCommand(const char *cmd) {
  // Gửi lệnh đi
  bool ket_noi_tot = radio.write(cmd, strlen(cmd) + 1);
  
  // Dựa vào kết quả gửi để bật/tắt đèn báo trên tay cầm
  if (ket_noi_tot) {
    digitalWrite(LED_BAO_LOI, LOW); // Gửi được -> Tắt đèn lỗi
  } else {
    digitalWrite(LED_BAO_LOI, HIGH); // Gửi thất bại -> Bật đèn lỗi
  }
}

void loop() {
  // Nếu CÓ bấm nút -> Gửi lệnh điều khiển
  if (digitalRead(12) == HIGH) sendCommand("tien");
  else if (digitalRead(14) == HIGH) sendCommand("lui");
  else if (digitalRead(26) == HIGH) sendCommand("trai");
  else if (digitalRead(27) == HIGH) sendCommand("phai");
  
  // Nếu KHÔNG bấm nút -> Gửi "ping" để check sóng
  else {
    sendCommand("ping");
  }
  
  delay(50); // Delay vừa phải để không spam quá nhanh
}