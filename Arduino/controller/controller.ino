#include <math.h>
#include <Servo.h>

const unsigned int intR_pin = 2;
const unsigned int intL_pin = 3;

const unsigned int encR_pin = 4;
const unsigned int encL_pin = 5;

const unsigned int dirPinA = 10;
const unsigned int dirPinB = 12;

const unsigned int pwm_pin = 6;

volatile unsigned long last_msg = 0;

unsigned long last_run = 0;
int period = 100;

volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;


float cmd_linear = 0;
int cmd_angular = 60;
unsigned long cmd_time = 0;

//read enc
unsigned long prevmicros = 0;
char res;
long position_current = 0;
float position_rad = 0.0;
float speed_rad = 0.0;
int speed_ = 0;
long now = 0;
long lastTime_ = 0; //used to be volatile
float speed_rad_old = 0;

const int ppr = 32; // pulses per revolution
const float wheelRadius = 58;  // in mm
const float wheelBase = 260;   // distance between the wheels in mm

float distanceL = 0.0;
float distanceR = 0.0;

// float speed_arr[ARRAY_SIZE] = {0.0};
int currentIndex = 0;

//PWM
double previous_error = 0;
double error_sum = 0;
double error = 0;
double derror = 0;

int motor_speed = 0;
// float speed_rad = 0;
// float speed_ = 0;
unsigned long currentTime = 0;
unsigned long previousTime = 0;
double elapsedTime = 0;


//old
// const float P = 25.0; 
// const float I = 0.01;
// const float D = 0.01;
//new
const float P = 50.0; 
const float I = 0.05;
const float D = 0.01;

const int maxpwm = 50;

char res1, res2, res3;
int lost1, lost2, lost3;

Servo steering;







boolean newData = false;
char receivedChars[20]; // Adjust the size based on your needs
int index = 0;


// function for quickly set pin from PORTB on the Arduino
void setPin(int digPinNo){

  PORTB = PORTB|1<<(digPinNo-8); 
}

// function for quickly reset pin from PORTB on the Arduino
void resetPin(int digPinNo){
  PORTB = PORTB&~(1<<(digPinNo-8)); 
}


// Interrupt function left wheel
void encoderLInterrupt() {
  noInterrupts();
  if (digitalRead(encL_pin) == digitalRead(intL_pin)){
    encoderLPosition--;
  }
  else{
    encoderLPosition++;
  }
  interrupts();
  
}

// interrupt function right wheel
void encoderRInterrupt() {
  noInterrupts();
  if (digitalRead(encR_pin) == digitalRead(intR_pin)){
    encoderRPosition++;
  }
  else{
    encoderRPosition--;
  }
  interrupts();
}


void ReadEncoders();

void PID();


void setup(){
  while(!Serial){}
  Serial.begin(115200);
  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  resetPin(dirPinA);
  resetPin(dirPinB);

  attachInterrupt(digitalPinToInterrupt(intL_pin),encoderLInterrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(intR_pin),encoderRInterrupt,CHANGE);

  steering.attach(9);
  steering.write(65);
}


void loop(){

  steering.write(cmd_angular);
  analogWrite(pwm_pin, motor_speed);

  if(last_run + period < millis()){
    last_run = millis();
    ReadEncoders();
    PID();
  }
  if (newData){
    // Process the received data
    processCommand();
    
    // Reset the flag and buffer for the next data
    newData = false;
    index = 0;
    memset(receivedChars, 0, sizeof(receivedChars));
  }
  delay(1);
}

void ReadEncoders(){
  distanceL = (2 * PI * wheelRadius) * encoderLPosition / ppr;
  distanceR = (2 * PI * wheelRadius) * encoderRPosition / ppr;

  float displacement = (distanceL + distanceR) / 2.0;
  float heading = (distanceR - distanceL) / wheelBase;

  encoderLPosition = 0;
  encoderRPosition = 0;

  Serial.print(displacement);
  Serial.print(" ");
  Serial.println(heading); 

  now = micros();
  speed_rad = (float)(((displacement)*1000000)/(now-lastTime_));

  // speed_arr[currentIndex] = speed_rad;

  // // Increment the index and wrap around if necessary
  // currentIndex = (currentIndex + 1) % ARRAY_SIZE;

  // Calculate the average
  // float sum = 0.0;
  // for (int i = 0; i < ARRAY_SIZE; ++i) {
  //   sum += speed_arr[i];
  // }
  // float average = sum / ARRAY_SIZE;


  // speed_rad_old = speed_rad;
  lastTime_ = now;
}


void PID(){             
  elapsedTime = (double)(millis() - previousTime);  

  // speed_rad = speed_rad/1000; //to m/s
  error = cmd_linear - speed_rad/1000;
  // error_sum += error * elapsedTime;  
  derror = (error - previous_error)/elapsedTime;

  // Calculate the motor speed using cmd equation
  // speed_ = (P * error) + (I * error_sum) + (D * derror);

  //anti windup
  if(motor_speed < maxpwm){
    error_sum += error * elapsedTime; 
  }

  speed_ = (P * error) + (I * error_sum) + (D * derror);

  motor_speed = (int)(speed_);

  // noInterrupts();
  // Serial.print(cmd.linear);
  // Serial.print(" ");
  // Serial.print(speed_rad);
  // Serial.print(" ");
  // Serial.println(motor_speed/10);
  // interrupts();

  if(motor_speed < 0){ //backward
    resetPin(dirPinA);
    setPin(dirPinB);
  }
  else if(motor_speed > 0){ //forward
    setPin(dirPinA);
    resetPin(dirPinB);
  }
  else{
    resetPin(dirPinA);
    resetPin(dirPinB);
  }

  if(motor_speed < 0){
    motor_speed = 0 - motor_speed;
  }

  if(motor_speed > maxpwm){
    motor_speed = maxpwm;
  }

  if(cmd_linear == 0.0 || cmd_time + 200 < millis()){
    analogWrite(pwm_pin, 0);
    resetPin(dirPinA);
    resetPin(dirPinB);
  }
  else{
    analogWrite(pwm_pin, motor_speed);
  }
  // steering.write(cmd_angular);

  // Serial.print("Received motor_speed: ");
  // Serial.print(motor_speed);
  // Serial.print(", Received cmd_angular: ");
  // Serial.println(cmd_angular);

  previous_error = error;
  previousTime = currentTime; 
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == ';') {
      // End of command, set flag to process the data
      newData = true;
    } else {
      // Add the character to the buffer
      receivedChars[index] = inChar;
      index++;
    }
  }
}

void processCommand() {
  // Use strtok to split the received data by ","
  char *token = strtok(receivedChars, ",");
  
  // Check if token is not null
  if (token != NULL) {
    // Convert the first part to a float
    cmd_linear = atof(token);
    
    // Move to the next token
    token = strtok(NULL, ",");
    
    // Check if the next token is not null
    if (token != NULL) {
      // Convert the second part to an int
      cmd_angular = atoi(token);
      cmd_time = millis();
      
    }
  }
}