#include <SPI.h>
#define ESPCS 5

uint32_t msg;
byte eightbitmsg[10] = {0xAA, 0xAA, 0x55, 0xFE, 0xFE, 0x66, 0x03, 0x09, 0xA2, 0x33};
byte eightbitsamemsg[10] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
SPISettings settingsA(200000, MSBFIRST, SPI_MODE0);
SPISettings orgSettings(24000000, MSBFIRST, SPI_MODE0);
SPISettings espSettings(4000000, LSBFIRST, SPI_MODE0);

void setup() {
  // initialize the digital pin as an output.
  pinMode(ESPCS, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  digitalWrite(ESPCS, HIGH);
  SPI.begin();
}

void loop() {
//  SPI.beginTransaction(espSettings);
//  digitalWrite(ESPCS, LOW);
  for(uint8_t i = 0; i < 8; i++){
    SPI.beginTransaction(espSettings);
    digitalWrite(ESPCS, LOW);
    Serial.print("Send: "); Serial.print(eightbitmsg[i], HEX); Serial.print(" | Received: ");
    Serial.println(SPI.transfer(eightbitmsg[i]), HEX);
    delayMicroseconds(1);
    digitalWrite(ESPCS, HIGH);
    SPI.endTransaction();
  }
  
//  delayMicroseconds(1);
//  digitalWrite(ESPCS, HIGH);
//  SPI.endTransaction();
  Serial.println("======MASTER=======");
  delay(500);
}
