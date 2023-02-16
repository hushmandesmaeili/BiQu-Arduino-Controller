#include <SPI.h>

uint8_t arr[8] = {1, 2, 3, 4, 32, 64, 128, 255};

void setup() {
  // initialize the digital pin as an output.
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  SPI.begin();
  digitalWrite(10, HIGH);
}

void loop() {
  SPI.beginTransaction(SPISettings(200000, MSBFIRST, SPI_MODE0));
  digitalWrite(10, LOW);
  for(uint8_t i = 0; i < 8; i++){
    //Serial.println(SPI.transfer(0x55));
    //SPI.transfer(0x55);
    SPI.transfer(arr[i]);
  }
  delayMicroseconds(1);
  digitalWrite(10, HIGH);
  SPI.endTransaction();
  Serial.println("=============");
  delay(100);
}
