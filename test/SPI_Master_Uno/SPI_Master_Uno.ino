#include <SPI.h>
uint32_t msg;
byte eightbitmsg[5] = {0xAA, 0xAA, 0x55, 0xFE, 0xFE};
SPISettings settingsA(200000, MSBFIRST, SPI_MODE0);
SPISettings orgSettings(24000000, MSBFIRST, SPI_MODE0);

void setup() {
  // initialize the digital pin as an output.
  pinMode(10, OUTPUT);
  Serial.begin(115200);
  while(!Serial);
  digitalWrite(10, HIGH);
  SPI.begin();
}

void loop() {
  SPI.beginTransaction(orgSettings);
  digitalWrite(10, LOW);
  //for(uint8_t i = 0; i < 4; i++){
    //msg = 0xEF000000;
    //Serial.print("Send: ");
    //Serial.print(eightbitmsg[i]);
    //Serial.print(" | Received: ");
    //Serial.println(SPI.transfer(eightbitmsg[i]));
    //SPI.transfer(msg, 4);
  //}
  Serial.println(SPI.transfer(0xEF));
  Serial.println(SPI.transfer(0xAA));
  Serial.println(SPI.transfer(0x55));
  Serial.println(SPI.transfer(0xAA));
//  Serial.println(SPI.transfer(msg));
  
  //delayMicroseconds(1);
  digitalWrite(10, HIGH);
  SPI.endTransaction();
  Serial.println("=============");
  delay(500);
}
