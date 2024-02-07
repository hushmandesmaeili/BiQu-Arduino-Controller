#include "SPISlave_T4.h"

SPISlave_T4 mySPI(0, SPI_16_BITS);
//uint8_t arr[10] = {9, 2, 8, 6, 10, 20, 44, 65, 0, 0};  //size = 10
//uint8_t arr[10] = {1, 4, 2, 3, 4, 5, 6, 7, 8, 9};  //size = 10
uint16_t arr[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9,10};  //size = 10
byte actualdata[255];
byte rpitest[8] = {0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x04};
byte rpitestff00[8] = {0xFF, 0x00,0xFF, 0x00,0xFF, 0x00,0xFF, 0x00};
//byte rpitestaa[8] = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
byte rpitestaa[8] = {0x00, 0xD9, 0x00, 0xD9, 0x00, 0xD9, 0x00, 0xD9};

uint8_t i = 0;
uint8_t j = 0;
uint8_t buildArr[4];
uint16_t buildTwoArr[2];

// Good: using memcpy
float bytesToFloat(uint8_t (&bytes)[4]) {
    static_assert(sizeof(float) == 4, "float size expected to be 4 bytes");
    float result;
    memcpy(&result, bytes, 4);
    return result;
}

// Good: using memcpy
float TWOToFloat(uint16_t twobytes[2]) {
    float result;
    memcpy(&result, twobytes, 4);
    return result;
}


void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hello World!");
//  Serial.println("Iteration | Send | Receive");
//  Serial.println(sizeof(arr));
  mySPI.begin(LSBFIRST, SPI_MODE0);
  mySPI.onReceive(myFunc);
  
}

void loop() {
//  Serial.print("millis: "); Serial.println(millis());
  delay(100);
}

void myFunc() {
  Serial.print("-----\n");
  
  while ( mySPI.available() ) {
    if(i >= 2){
      // one float requires 4 transactions
      i = 0;
      float entry = TWOToFloat(buildTwoArr);
      Serial.print("read : "); Serial.print(entry); Serial.println("");
//      Serial.println("-");
    } 
    if(j > 9){
      j = 0;
    }
//      Serial.print(j); Serial.print("|");
//      Serial.print(arr[j], HEX);
//      Serial.println("");
      mySPI.pushr(arr[j]);
      
      uint16_t receivedVal = mySPI.popr();
      buildTwoArr[i] = receivedVal;
//      Serial.print("|"); Serial.print(receivedVal, BIN); Serial.println("");
      
      i++; j++;
  }

//  Serial.print("-------SLAVE------\n");
}
