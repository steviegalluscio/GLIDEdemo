#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

unsigned char buffer_RTT[4] = {0}; // ac
uint8_t CS; // ac

#define COM 0x55 // ac
#define SPIN 6 
File myFile;

SoftwareSerial mySerial(7, 8); // ac

void setup() {
  // acoustic sensor setup
  Serial.begin(115200);
  mySerial.begin(115200);

  // serial data logger setup
  myFile = SD.open("file.txt", FILE_WRITE);

}
void loop() { 
  // acoustic sensor readings
  mySerial.write(COM);
  delay(100);
  if(mySerial.available() > 0){
    delay(4);
    if(mySerial.read() == 0xff){    
      buffer_RTT[0] = 0xff;
      for (int k=1; k<4; k++){
        buffer_RTT[k] = mySerial.read();   
      }
      CS = buffer_RTT[0] + buffer_RTT[1]+ buffer_RTT[2];  
      if(buffer_RTT[3] == CS) {
        int Distance = (buffer_RTT[1] << 8) + buffer_RTT[2];
        //Serial.print("Distance:");
        Serial.println(Distance);
        //Serial.println("mm");
      }
    }
  }
}