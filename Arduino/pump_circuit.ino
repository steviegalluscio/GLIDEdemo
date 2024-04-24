#include <Servo.h>
Servo myservo;
//rudder related stuff
const int initPos = 84;
const int relativePos = 30;
const char stepSize = relativePos/6; //this must be a factor of relativePos
int currPos = initPos;
bool startup_complete = false;
const char rudderPin = 6;

//peristaltic pump stuff
const char forwardPin = 3;
const char reversePin = 5;
const char enablePin = 2;
const char buttonPin = 8;
const int delayTime = 120000;
char buttonState = 0;


void setup()
{
  Serial.begin(9600);
  pinMode(forwardPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  //myservo.attach(rudderPin);
  //myservo.write(initPos);
  digitalWrite(enablePin, HIGH);
  //delay(300000);
}

void delayRudder(int targetPos){
  for(int i = 0; i < relativePos/stepSize; i++){
    if(currPos > targetPos){
      myservo.write(currPos - stepSize);
      currPos -= stepSize;
      delay(400);
    }
    else {
      myservo.write(currPos + stepSize);
      currPos += stepSize;
      delay(400);
    }
  }
}

void depthController(){
	
  digitalWrite(forwardPin, LOW);
  digitalWrite(reversePin, HIGH);
  //delayRudder(initPos - relativePos);
  delay(delayTime);

  digitalWrite(forwardPin, LOW);
  digitalWrite(reversePin, LOW);
  //delayRudder(initPos);
  delay(delayTime/2);

  digitalWrite(forwardPin, HIGH);
  digitalWrite(reversePin, LOW);
  //delayRudder(initPos + relativePos);
  delay(delayTime);

  digitalWrite(forwardPin, LOW);
  digitalWrite(reversePin, LOW);
  //delayRudder(initPos);
  delay(delayTime/2);

}

void loop()
{
  // if(!startup_complete){
  //   digitalWrite(forwardPin, LOW);
  //   digitalWrite(reversePin, HIGH);
  //   delay(7500);
  //   startup_complete = true;
  // }
  // depthController();
  
  // if(digitalRead(buttonPin) == 1){
   
  // }
  Serial.println(digitalRead(buttonPin));
  switch(buttonState){
      case 0:
        digitalWrite(forwardPin, LOW);
        digitalWrite(reversePin, LOW);
        delay(20000);
        buttonState = 1;
        break;
      case 1:
      //sucks water out
        digitalWrite(forwardPin, LOW);
        digitalWrite(reversePin, HIGH);
        delay(120000);
        buttonState = 2;
        break;
      case 2:
        digitalWrite(forwardPin, LOW);
        digitalWrite(reversePin, LOW);
        delay(20000);
        buttonState = 3;
        break;
      case 3:
      //sucks water in
        digitalWrite(forwardPin, HIGH);
        digitalWrite(reversePin, LOW);
        delay(100000);
        buttonState = 0;
        break;
      default:
        buttonState = 0;
    }

  
}