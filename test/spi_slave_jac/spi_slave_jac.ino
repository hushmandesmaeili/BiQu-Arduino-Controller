#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_8_BITS);
uint8_t arr[10] = {9, 2, 8, 6, 10, 20, 44, 65, 86, 254};
uint32_t bigbytes = 0xAAAA;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
  
  mySPI.begin(LSBFIRST, SPI_MODE0);
  mySPI.onReceive(myFunc);
}

void loop() {
  Serial.print("millis: "); Serial.println(millis());
  Serial.print("-------SLAVE------\n");
  delay(500);
}

void myFunc() {
//  Serial.print("-----\n");
  while ( mySPI.available() ) {
//    for(int i = 0; i < 10; i++){ 
      //Serial.print("Send:"); Serial.print(arr[i]);
      Serial.print("Send: "); Serial.print(0x55, HEX);
      mySPI.pushr(0x55);
      uint8_t receivedVal = mySPI.popr();
      Serial.print(" | Received: "); Serial.print(receivedVal, HEX); Serial.println("");
//      
//    }
  }

//  Serial.print("-------SLAVE------\n");
}
