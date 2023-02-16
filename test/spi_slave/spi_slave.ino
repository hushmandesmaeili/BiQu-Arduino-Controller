#include "SPISlave_T4.h"
SPISlave_T4<&SPI, SPI_8_BITS> mySPI;
byte returnVal = 0xEE;

void setup() {
  //Serial.begin(115200);
  //while (!Serial);
  //Serial.println("Starting");
  mySPI.onReceive(myFunc);
  mySPI.begin();
  //Serial.println("Begin");
  //Serial.println("onReceive setup");
}

void loop() {
  //Serial.print("millis: "); Serial.println(millis());
  delay(1000);
}

void myFunc() {
  //Serial.println("START: ");
  // uint8_t arr[] = { 3, 2, 8, 3, 10, 11, 33, 13, 14 };
  //uint8_t arr[] = {0x03, 0x02, 0x05, 0x06};
  uint8_t i = 0;
  while ( mySPI.active() ) {
    if (mySPI.available()) {
      mySPI.pushr(2);
      uint8_t receive = mySPI.popr();
      //Serial.println(receive, HEX);
      //if ( i++ > sizeof(arr) ) i = 0;
      // Serial.print("\t i = ");
      // Serial.print(i);
      //mySPI.pushr(receive);
      //Serial.print("\tVALUE: ");
      //Serial.print(mySPI.popr());
    }
  }
  //Serial.print("\tEND\n");
}
