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
  if (radio.available()) {
    char msg[32] = "";
    radio.read(&msg, sizeof(msg));

    last_rx = millis();              
    digitalWrite(LED_MAT_SONG, LOW);  
    
    if (strcmp(msg, "tien") == 0) tien();
    else if (strcmp(msg, "lui") == 0) lui();
    else if (strcmp(msg, "trai") == 0) trai();
    else if (strcmp(msg, "phai") == 0) phai();

    else {
      stop();
    }
  }

  if (millis() - last_rx > timeout) {
    stop();                            
    digitalWrite(LED_MAT_SONG, HIGH);  
  }
}
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
