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

  // initialize devices
  // Serial.println("Initializing I2C devices...");

  // initialize bmp085
  if (!bmp.begin()) {
    // Serial.println("Could not find a valid BMP085 sensor, check wiring!");
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


 
}

void loop() {
  // Serial.print("Temperature = ");
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");

  // Serial.print("Pressure = ");
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  // Serial.print("Altitude = ");
  // Serial.print(bmp.readAltitude());
  // Serial.println(" meters");
  // Serial.print("Pressure at sealevel (calculated) = ");
  // Serial.print(bmp.readSealevelPressure());
  // Serial.println(" Pa");
  // Serial.print("Real altitude = ");
  // Serial.print(bmp.readAltitude(101500));
  // Serial.println(" meters");

  
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // display tab-separated accel/gyro x/y/z values



  // float heading = Compass.GetHeadingDegrees();
  // Serial.print("Heading: \t");
  // Serial.println( heading );



  imu_dump();


  // ensure the correct timing is upheld
  while (millis() - prevMillis < 100){/* DO NOTHING*/}
  time = millis() - prevMillis;
  prevMillis = millis();
  // Serial.print("time: ");
  // Serial.println(time);
}


