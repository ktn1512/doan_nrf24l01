// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <Arduino.h>
// /*
// ======L293D/L298N======
// enA=3
// in1=4
// in2=6
// in3=7
// in4=8
// enB=5
// ======NRF2401L=========
// 1 -> GND
// 2 -> 3V3
// 3 -> 9(CE)
// 4 -> 10(CS)
// 5 -> 13(SCK)
// 6 -> 11(MOSI)
// 5 -> 12(MISO)
// */
// RF24 radio(9, 10);  // CE = D9, CSN = D10 (đúng dây bạn đang dùng)
// const byte address[6] = "00001";
// unsigned long last_rx = 0;
// const long timeout = 100;
// void setup() {
//    Serial.begin(9600);
//   if (!radio.begin()) {
//     Serial.println("❌ Không tìm thấy NRF24L01 (SPI FAIL?)");
//     while (1);
//   }
//   Serial.println("✔ SPI OK, tìm thấy NRF24L01");
//   Serial.begin(9600);
//   Serial.println("Arduino NRF24L01 Receiver");
//   pinMode(2, OUTPUT);
//   pinMode(4, OUTPUT);
//   pinMode(6, OUTPUT);
//   pinMode(7, OUTPUT);
//   pinMode(5, OUTPUT);
//   pinMode(8, OUTPUT);
//   digitalWrite(2, LOW);
//   digitalWrite(4, LOW);
//   digitalWrite(6, LOW);
//   digitalWrite(7, LOW);
//   digitalWrite(5, LOW);
//   digitalWrite(8, LOW);
//   pinMode(3, OUTPUT);
//   digitalWrite(3, LOW);

//   if (!radio.begin()) {
//     Serial.println("NRF24 init failed!");
//     while (1)
//       ;
//   }

//   radio.openWritingPipe(address);
//   radio.openReadingPipe(1, address);
//   radio.setPALevel(RF24_PA_LOW);
//   radio.setDataRate(RF24_250KBPS);
//   radio.startListening();
// }

// void loop() {
//   if (radio.available()) {
//     char msg[32] = "";
//     radio.read(&msg, sizeof(msg));
//     Serial.print("Received: ");
//     Serial.println(msg);
//     last_rx = millis();

//     //// Nếu nhận được "hello" thì nháy còi
//     // if (strcmp(msg, "hello") == 0) {
//     //   Serial.println("Trigger buzzer!");
//     //   for (int i = 0; i < 3; i++) {  // nháy 3 lần
//     //     digitalWrite(3, HIGH);
//     //     delay(200);
//     //     digitalWrite(3, LOW);
//     //     delay(200);
//     //   }
//     // }

//     if (strcmp(msg, "tien") == 0) {
//       tien();
//     }

//     if (strcmp(msg, "lui") == 0) {
//       lui();
//     }

//     if (strcmp(msg, "trai") == 0) {
//       trai();
//     }

//     if (strcmp(msg, "phai") == 0) {
//       phai();
//     }

//     // Gửi phản hồi "chao"
//     radio.stopListening();
//     const char reply[] = "chao";
//     radio.write(&reply, sizeof(reply));
//     Serial.println("Replied: chao");
//     radio.startListening();
//   }

//   if (millis() - last_rx > timeout) {
//     stop();
//    // last_rx = millis();
//   }
// }

// void stop() {
//   digitalWrite(3, 0);
//   digitalWrite(4, 0);
//   digitalWrite(6, 0);
//   analogWrite(5, 0);
//   digitalWrite(7, 0);
//   digitalWrite(8, 0);
// }

// void tien() {
//   digitalWrite(3, 1);
//   digitalWrite(4, 1);
//   digitalWrite(6, 0);
//   analogWrite(5, 255);
//   digitalWrite(7, 1);
//   digitalWrite(8, 0);
// }
// void lui() {
//   digitalWrite(3, 1);
//   digitalWrite(4, 0);
//   digitalWrite(6, 1);
//   analogWrite(5, 255);
//   digitalWrite(7, 0);
//   digitalWrite(8, 1);
// }
// void trai() {
//   digitalWrite(3, 1);
//   digitalWrite(4, 1);
//   digitalWrite(6, 0);
//   analogWrite(5, 255);
//   digitalWrite(7, 0);
//   digitalWrite(8, 1);
// }
// void phai() {
//   digitalWrite(3, 1);
//   digitalWrite(4, 0);
//   digitalWrite(6, 1);
//   analogWrite(5, 255);
//   digitalWrite(7, 1);
//   digitalWrite(8, 0);
// }



// ====== RECEIVER (XE - ARDUINO) ======
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);
const byte address[6] = "00001";

unsigned long last_rx = 0;
const long timeout = 500;

#define LED_MAT_SONG A0

void setup() {
  Serial.begin(9600);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(LED_MAT_SONG, OUTPUT);

  if (!radio.begin()) {
    while (1)
      ;
  }
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
}

void loop() {
  // 1. KHI NHẬN ĐƯỢC TÍN HIỆU (Bất kể là lệnh gì hay là ping)
  if (radio.available()) {
    char msg[32] = "";
    radio.read(&msg, sizeof(msg));

    last_rx = millis();               // Cập nhật thời gian -> XÁC NHẬN CÒN KẾT NỐI
    digitalWrite(LED_MAT_SONG, LOW);  // Tắt đèn báo lỗi

    // Xử lý lệnh
    if (strcmp(msg, "tien") == 0) tien();
    else if (strcmp(msg, "lui") == 0) lui();
    else if (strcmp(msg, "trai") == 0) trai();
    else if (strcmp(msg, "phai") == 0) phai();

    // NẾU LÀ "ping" (Tay cầm đang rảnh) -> DỪNG XE
    else {
      stop();
    }
  }

  // 2. KHI MẤT TÍN HIỆU QUÁ LÂU (Rút điện tay cầm / Đi quá xa)
  if (millis() - last_rx > timeout) {
    stop();                            // Dừng xe
    digitalWrite(LED_MAT_SONG, HIGH);  // Bật đèn báo lỗi
  }
}

// --- CÁC HÀM MOTOR (GIỮ NGUYÊN) ---
void stop() {
  digitalWrite(3, 0);
  digitalWrite(4, 0);
  digitalWrite(6, 0);
  analogWrite(5, 0);
  digitalWrite(7, 0);
  digitalWrite(8, 0);
}
void tien() {
  digitalWrite(3, 1);
  digitalWrite(4, 1);
  digitalWrite(6, 0);
  analogWrite(5, 255);
  digitalWrite(7, 1);
  digitalWrite(8, 0);
}
void lui() {
  digitalWrite(3, 1);
  digitalWrite(4, 0);
  digitalWrite(6, 1);
  analogWrite(5, 255);
  digitalWrite(7, 0);
  digitalWrite(8, 1);
}
void trai() {
  digitalWrite(3, 1);
  digitalWrite(4, 1);
  digitalWrite(6, 0);
  analogWrite(5, 255);
  digitalWrite(7, 0);
  digitalWrite(8, 1);
}
void phai() {
  digitalWrite(3, 1);
  digitalWrite(4, 0);
  digitalWrite(6, 1);
  analogWrite(5, 255);
  digitalWrite(7, 1);
  digitalWrite(8, 0);
}