/*
  Modified on Mar 16, 2021
  Modified by MehranMaleki from Arduino Examples
  https://electro
  peak.com/learn/
*/

// #include "I2Cdev.h"
#include <Wire.h>
#include "MPU6050.h"
#include <Adafruit_BMP085.h>
#include <HMC5883L_Simple.h>


MPU6050 accelgyro;
Adafruit_BMP085 bmp;
HMC5883L_Simple Compass;


int16_t ax, ay, az;
int16_t gx, gy, gz;

unsigned long prevMillis = 0;
int time = 0;
// char jar[20] = "";
// int incoming = 0;
char serialString[10] = "          "; // Empty serial string variable
bool stringFinished = false; // Flag to indicate reception of a string after terminator is reached
bool serialTail = false;
int value;


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
  Serial.begin(115200);
  Wire.begin();
  pinMode(13, OUTPUT);
  // initialize devices
  // Serial.println("Initializing I2C devices...");

  // initialize bmp085
  if (!bmp.begin()) {
    // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    digitalWrite(13, HIGH);
    while (1) {}
  }

  // initialize mpu6050
  accelgyro.initialize();
  // Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
  
  
  // initialize hmc5883l
  // Compass.SetDeclination(23, 35, 'E');
  // Compass.SetSamplingMode(COMPASS_SINGLE);
  // Compass.SetScale(COMPASS_SCALE_130);
  // Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
  while(!Serial){}
  Serial.println();
  Serial.println("heard,ready");

 
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu_dump();
  

  if(stringFinished){
    // Serial.print("heard,");
    // Serial.println(serialString);
    int nonNullCount = 0;
    bool secondValue = false;
    pwm1 = 0;
    pwm2 = 0;
    
    for (int i = 0; i < sizeof(serialString); i++) {
      if (serialString[i] != ' ') {
        nonNullCount++;
      }
    }

    for(int i = 1; i < nonNullCount; i++) {
      if(serialString[i] == ',') {
        secondValue = true; // Set flag to true once we encounter the comma
      } else {
        if(!secondValue) {
          pwm1 = pwm1 * 10 + (serialString[i] - '0');
        } else {
          pwm2 = pwm2 * 10 + (serialString[i] - '0');
        }
      }
    }


    if(serialString[0] == 'm'){
      //set servo pwm
      Serial.print("heard,m:");
      Serial.print(pwm1);
      Serial.print("s:");
      Serial.println(pwm2);
    }
  
    
    if(serialString[0] != 'm' && serialString[0] != 's'){
      Serial.println("heard,error msg[0]!=known input");
    }


    memset(serialString, ' ', sizeof(serialString));
    stringFinished = false;
  }


  while(!Serial){
    // stop the car!
    digitalWrite(13,HIGH);
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
