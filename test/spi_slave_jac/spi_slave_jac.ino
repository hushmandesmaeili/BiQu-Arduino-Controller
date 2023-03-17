#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_8_BITS);
uint8_t arr[10] = {9, 2, 8, 6, 10, 20, 44, 65, 0, 0};  //size = 10
byte actualdata[255];
byte rpitest[8] = {0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04};
byte rpitestff00[8] = {0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00};
//byte rpitestaa[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
byte rpitestaa[8] = {0x00, 0xD9, 0x00, 0xD9, 0x00, 0xD9, 0x00, 0xD9};

// D9 1101 1001
//    1001 1011
//    1101 1001


uint32_t bigbytes = 0xAAAA;

uint8_t onlyonce = 1;
uint8_t i = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
  actualdata[0] = 0x79;
  actualdata[254] = 0x44;
  
  mySPI.begin(LSBFIRST, SPI_MODE0);
  mySPI.onReceive(myFunc);

  
}

void loop() {
//  Serial.print("millis: "); Serial.println(millis());
  delay(100);
}

void myFunc() {
//  Serial.print("-----\n");
  
  while ( mySPI.available() ) {
    if(i >= sizeof(rpitest)){
      i = 0;

      // for debugging lower frequency
      //Serial.print("-------SLAVE------\n");

      // for debugging higher frequency
      Serial.println("-");
    }
      Serial.print(i); Serial.print("|");
      Serial.print(rpitest[i], HEX);
      mySPI.pushr(rpitest[i]);
      i++;
      uint8_t receivedVal = mySPI.popr();
      Serial.print("|"); Serial.print(receivedVal, HEX); Serial.println("");
  }

//  Serial.print("-------SLAVE------\n");
}
