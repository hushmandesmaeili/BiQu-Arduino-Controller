#include <SPI.h>

void setup() {
  // initialize the digital pin as an output.
  pinMode(5, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(SPISettings(2000000, LSBFIRST, SPI_MODE0));
  digitalWrite(5, LOW);
  for(uint8_t i = 0; i < 9; i++){
    Serial.println(SPI.transfer(3));
  }
  delayMicroseconds(1);
  digitalWrite(5, HIGH);
  SPI.endTransaction();
  Serial.println("=============");
  delay(500);
}
