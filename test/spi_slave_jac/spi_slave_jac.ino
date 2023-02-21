#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_8_BITS);
uint8_t arr[10] = {9, 2, 8, 6, 10, 20, 44, 65, 86, 254};
uint32_t bigbytes = 0xAAAA;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
  
  mySPI.begin(LSBFIRST, SPI_MODE0);
  //mySPI.swapPins(true);
  mySPI.onReceive(myFunc);
}

void loop() {
  Serial.print("millis: "); Serial.println(millis());
  delay(1000);
}

void myFunc() {
  Serial.print("-----\n");
  while ( mySPI.available() ) {
//    mySPI.pushr(0xFF);/
    Serial.print("FIFO Status: ");
    for(int i = 0; i < 10; i++){  // Serial.print("FIFO Status:");
      mySPI.pushr(arr[i]);
//    Serial.println(" ");
      uint8_t receivedVal = mySPI.popr();
//   / Serial.print("received: "); Serial.print(receivedVal, HEX); Serial.println("");
      
    }
  }

  Serial.print("-----\n");
}
