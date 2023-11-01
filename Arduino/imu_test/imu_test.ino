#include <Wire.h>
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
// #include <HMC5883L_Simple.h>
#include <Servo.h>

MPU6050 accelgyro;
Adafruit_BMP085 bmp;
// HMC5883L_Simple Compass;

#define motorPin 5
#define backwardDirection 10
#define forwardDirection 12
#define warningLED 13

Servo myservo;

int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long prevMillis = 0;
int time = 0;
char serialString[10] = "          "; // Empty serial string variable
bool stringFinished = false; // Flag to indicate reception of a string after terminator is reached
bool serialTail = false;
int value;
int forward = 0;


int pwm1 = 0;
int pwm2 = 0;


void imu_dump(){
  Serial.print("IMU,");
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.println(gz);
}


void setup() {
  // Serial and i²c init
  Serial.begin(115200);
  Wire.begin();

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

  // initialize mpu6050
  accelgyro.initialize();
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883


  // check if IMU is connected 
  if (!bmp.begin()) {
    Serial.println("I,IMU: check wiring!");
    digitalWrite(warningLED, HIGH);
    while (1) {}
  }

  Serial.println();
  Serial.println("I,ready");
 
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu_dump();
  

  if(stringFinished){
    // Serial.print("I,");
    // Serial.println(serialString);
    int nonNullCount = 0;
    int secondValue = 0;
    pwm1 = 0;
    pwm2 = 0;
    
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
          pwm2 = pwm2 * 10 + (serialString[i] - '0');
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
      Serial.println(pwm2);
      myservo.write(pwm2);
    }
  
    
    if(serialString[0] != 'm'){
      Serial.println("I,error msg[0]!=known input");
    }


    memset(serialString, ' ', sizeof(serialString));
    stringFinished = false;
  }


  while(!Serial){ //not working
    // stop the car!
    analogWrite(motorPin, 0);
    digitalWrite(forwardDirection, LOW);
    digitalWrite(backwardDirection, LOW);
    digitalWrite(warningLED, HIGH);
  }


  // ensure the correct timing is upheld
  while (millis() - prevMillis < 100){/* DO NOTHING*/}
  prevMillis = millis();
  // time = millis() - prevMillis;
  // Serial.print("time: ");
  // Serial.println(time);
}




void serialEvent()
{
  int idx = 0;

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
