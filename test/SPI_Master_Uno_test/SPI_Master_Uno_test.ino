#include <SPI.h>

void setup() {
  // initialize the digital pin as an output.
  pinMode(10, OUTPUT);
  Serial.begin(9600);
  while(!Serial);
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(SPISettings(4000000, LSBFIRST, SPI_MODE0));
  digitalWrite(10, LOW);
  for(uint8_t i = 0; i < 9; i++){
    Serial.println(SPI.transfer(3));
  }
  delayMicroseconds(1);
  digitalWrite(10, HIGH);
  SPI.endTransaction();
  Serial.println("=============");
  delay(500);
}
