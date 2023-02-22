#include <SPISlave_T4.h>

SPISlave_T4 mySPI(0, SPI_8_BITS);
uint8_t arr[4] = {5, 6, 7, 8};

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
  mySPI.begin(LSBFIRST, SPI_MODE0);
  //mySPI.onReceive(myFunc);
}

void loop() {
  Serial.print("millis: "); Serial.println(millis());
  delay(1000);
}

void myFunc() {
  Serial.print("[");
  while ( mySPI.available() ) {
    Serial.println(mySPI.available());
    //Serial.print((char)mySPI.popr());
    //mySPI.popr();
    //Serial.println(mySPI.popr());
    mySPI.pushr(arr[1]);
    mySPI.pushr(arr[1]);
    mySPI.pushr(arr[1]);
    uint8_t receive = mySPI.popr();
    Serial.println(receive);
    Serial.println(mySPI.available());
    //mySPI.pushr(receive);
  }
  Serial.println("]");
  
}
