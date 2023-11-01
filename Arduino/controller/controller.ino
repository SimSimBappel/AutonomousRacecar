#include <Servo.h>

#define motorPin 5
#define backwardDirection 10
#define forwardDirection 12
#define warningLED 13

Servo myservo;


unsigned long prevMillis = 0;
unsigned long prevmillis2 = 0;
int time = 0;
int timing = 10; //update period (ms)
char serialString[10] = "          "; // Empty serial string variable
bool stringFinished = false; // Flag to indicate reception of a string after terminator is reached
bool serialTail = false;
int value;
int forward = 0;


int pwm1 = 0;
int angle = 0;


void setup() {
  // Serial
  Serial.begin(115200);

  // led and motor pwm init
  pinMode(warningLED, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(backwardDirection, OUTPUT);
  pinMode(forwardDirection, OUTPUT);

  //neutral gear
  digitalWrite(backwardDirection, LOW);
  digitalWrite(forwardDirection, LOW);

  // servo init
  myservo.attach(3); 
  myservo.write(65);


  while(!Serial){
    digitalWrite(warningLED, HIGH);
  }
  digitalWrite(warningLED, LOW);


  Serial.println();
  Serial.println("I,ready");
 
}

void loop() {
  if(stringFinished){
    // Serial.print("I,");
    // Serial.println(serialString);
    int nonNullCount = 0;
    int secondValue = 0;
    pwm1 = 0;
    angle = 0;
    
    for (int i = 0; i < sizeof(serialString); i++) {
      if (serialString[i] != ' ') {
        nonNullCount++;
      }
    }

    for(int i = 1; i < nonNullCount; i++) {
      if(serialString[i] == ',') {
        secondValue++; // Set flag to true once we encounter the comma
      } else {
        if(secondValue == 0) {
          pwm1 = pwm1 * 10 + (serialString[i] - '0');
        } else if(secondValue == 1) {
          angle = angle * 10 + (serialString[i] - '0');
        }
        else if(secondValue == 2) {
          forward = int(serialString[i]);
        }
      }
    }


    if(serialString[0] == 'm'){
      //set direction
      if(forward == 48){ //backward
        digitalWrite(backwardDirection, HIGH);
        digitalWrite(forwardDirection, LOW);
      }
      else{ //forward
        digitalWrite(backwardDirection, LOW);
        digitalWrite(forwardDirection, HIGH);
      }
      // Serial.print("I,forward: ");
      // Serial.println(forward);
      analogWrite(motorPin, pwm1);

      //set servo pwm
      Serial.print("I,m:");
      Serial.print(pwm1);
      
      //set servo
      Serial.print("s:");
      Serial.println(angle);
      myservo.write(angle);
      time = micros() - prevmillis2;
      Serial.print("Time to set shit: ");
      Serial.println(time);
    }
  
    
    if(serialString[0] != 'm'){
      Serial.println("I,error msg[0] is unknown");
    }


    memset(serialString, ' ', sizeof(serialString));
    stringFinished = false;
  }



  // ensure the correct timing is upheld
  time = millis() - prevMillis;
  
  while (millis() - prevMillis < timing){/* DO NOTHING*/}

  if(time > timing){
    Serial.print("I,Arduino is behind by: ");
    Serial.println(time - timing);
  }

  // Serial.print("time: ");
  // Serial.println(time);
  prevMillis = millis();
}




void serialEvent()
{
  int idx = 0;
  prevmillis2 = micros();

  while (Serial.available())
  {
    char inChar = (char)Serial.read();

    if (inChar == '\n')    // The reading event stops at a new line character
    {
      serialTail = true;
      //serialString[idx] = inChar;
    }

    if(idx > 10){
      memset(serialString, 0, sizeof(serialString));
      idx = 0;
    }

    if (!serialTail)
    {
      serialString[idx] = inChar;
      idx++;
    }

    if (serialTail)
    {
      stringFinished = true;
      Serial.flush();
      serialTail = false;
    }
  }
}
