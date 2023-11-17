#include <krnl.h>
#include <math.h>
#include <Servo.h>

#define STK 200

const unsigned int intR_pin = 2;
const unsigned int intL_pin = 3;

const unsigned int encR_pin = 4;
const unsigned int encL_pin = 5;

const unsigned int dirPinA = 10;
const unsigned int dirPinB = 12;

const unsigned int pwm_pin = 6;

unsigned long timer_prev = 0;




struct k_t *p1, *p2, *p4, *semmutex;

struct k_msg_t *cmdMsgQ, *speedMsgQ;



char st1[STK], st2[STK], st4[STK];

volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;
int err;

struct CMD{
  uint8_t servo_pwm;
  int motor_pwm; //needs to be changed to understand m/s aka float or double
  bool dir = 0; //remove
};




CMD dataBufForCMDMsgQ[5];
float dataBufForCurrentSpeedMsgQ[5];


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

// read encoder task, it takes the position and calculated the current speed of the motor, this is then send by the curret speed queue
void ReadEncoders()
{
  unsigned long prevmicros = 0;
  char res;
  long position_current = 0;
  float position_rad = 0.0;
  float position_old_rad = 0.0;
  float speed_rad = 0.0;
  int position_old = 0;
  int speed_ = 0;
  long now = 0;
  long lastTime_ = 0; //used to be volatile
  int ppr = 32;//2048; // pulses per revolution
  float speed_rad_old = 0;

  const float wheelRadius = 11.5;  // in centimeters
  const float wheelBase = 13.0;   // distance between the wheels in centimeters

  float distanceL = 0.0;
  float distanceR = 0.0;

  // float total_dis = 0.0;
  // float total_head = 0.0;

  // k_set_sem_timer(sem1,100);
  while(1)
  {
    distanceL = (2 * PI * wheelRadius * encoderLPosition) / 360.0;
    distanceR = (2 * PI * wheelRadius * encoderRPosition) / 360.0;

    float displacement = (distanceL + distanceR) / 2.0;
    float heading = (distanceR - distanceL) / wheelBase;

    encoderLPosition = 0;
    encoderRPosition = 0;

    // total_dis+=displacement;
    // total_head+=heading;
    

    // Serial.print("Displacement: ");
    Serial.print(displacement);
    Serial.print(" ");
    Serial.println(heading);

    position_current = (encoderLPosition + encoderRPosition) / 2;

    // Position in radians
    position_rad = (float)((M_PI*2*position_current/ppr));

    now = micros();

    speed_ = (position_current-position_old); // tick per second
    
    speed_rad = (float)(((position_rad-position_old_rad)*1000000)/(now-lastTime_));
    speed_rad_old = speed_rad;

    res = k_send(speedMsgQ, &speed_rad);
    
    position_old = position_current;
    position_old_rad = position_rad;
    lastTime_ = now;

    k_signal(semmutex); //Husk at mutexen kun skal dække operationer med delte variable, så tidsrummet i mutex bliver så lille som muligt
    k_sleep(10);
  }
}




// The PWM task the PID value, target speed and currernt speed from the queue, calculate the pwm signal and sitting the motor dir pin
void PWM()
{ 
  double previous_error = 0;
  double error_sum = 0;
  double error = 0;
  double derror = 0;

  int motor_speed = 0;
  float speed_rad = 0;
  float speed_ = 0;
  unsigned long currentTime = 0;
  unsigned long previousTime = 0;
  double elapsedTime = 0;

  
  CMD cmd;
  cmd.servo_pwm = 60;
  cmd.motor_pwm = 0;
  float P = 1.0; 
  float I = 0.001;
  float D = 0.01;

  char res1, res2, res3;
  int lost1, lost2, lost3;

  Servo steering;
  steering.attach(9);
  
  
  while(1)
  {
    k_wait(semmutex, 0);
    currentTime = millis();                
    elapsedTime = (double)(currentTime - previousTime);  

    res1 = k_receive(cmdMsgQ, &cmd, 1, &lost1);

    res3 = k_receive(speedMsgQ, &speed_rad, 1, &lost3);
    
    error = cmd.motor_pwm - speed_rad;
    error_sum += error * elapsedTime;  
    derror = (error - previous_error)/elapsedTime;

    // Calculate the motor speed using cmd equation
    // speed_ = (P * error) + (I * error_sum) + (D * derror);
    float speedP = P * error;
    float speedI = I * error_sum;
    float speedD = D * derror;

    speed_ = speedP + speedI; //+ speedD;
    motor_speed = (int)(speed_);

    if(speed_ < 0){ //backward
      resetPin(dirPinA);
      setPin(dirPinB);
    }
    else if(speed_ > 0){ //forward
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

    if(motor_speed > 50){
      motor_speed = 50;
    }

    if(cmd.motor_pwm == 0){
      analogWrite(pwm_pin, 0);
    }
    else{
      analogWrite(pwm_pin, motor_speed);
    }
  
    
    steering.write(cmd.servo_pwm);

    // Serial.print(" ");
    // Serial.print(speedP);
    // Serial.print(" ");
    // Serial.print(speedI);
    // Serial.print(" ");
    // Serial.print(speedD);
    // Serial.print(" ");
    // Serial.print(speed_rad);
    // Serial.print(" ");
    // if(speed_ < 0){ //backward
    //   Serial.println(-cmd.motor_pwm);
    // }
    // else{ //forward
    //   Serial.println(cmd.motor_pwm);
    // }
    // // Serial.println(motor_speed);
    
    previous_error = error;
    previousTime = currentTime; 
    
    k_sleep(1);
  }
}

// the read cmd serial check if there are something in the serial buffer to be read, if there is the cmd values are then put in the queue
void ReadSerial()
{
  bool serialTail = false;
  bool stringFinished = false; // Flag to indicate reception of a string after terminator is reached
  char serialString[10] = "          "; // Empty serial string variable
  int pwm1 = 0;
  int angle = 90;
  int forward = 0;
  CMD cmd;

  // Takes 256 microseconds worst case
  while(1)
  {

    int idx = 0;
    
    while (Serial.available())
    {
      char inChar = (char)Serial.read();

      if (inChar == '\n')    // The reading event stops at a new line character
      {
        serialTail = true;
        int nonNullCount = 0;
        int secondValue = 0;
        pwm1 = 0;
        angle = 0;
        
        for (int i = 0; i < sizeof(serialString); i++) {
          if (serialString[i] != ' ') {
            nonNullCount++;
          }
        }
        // Serial.println(serialString);
        for(int i = 0; i < nonNullCount; i++) {
          if(serialString[i] == ',') {
            secondValue++; // Set flag to true once we encounter the comma
          } else {
            if(secondValue == 0) {
              pwm1 = pwm1 * 10 + (serialString[i] - '0');
            } else if(secondValue == 1) {
              angle = angle * 10 + (serialString[i] - '0');
            }
            else if(secondValue == 2) { //remove
              forward = int(serialString[i]); //remove
            }//remove
          }
        }

        if(forward == 48){ //remove
          cmd.dir = false; //reverse //remove
          pwm1 = -pwm1; //remove
        }//remove

        cmd.motor_pwm = pwm1;
        cmd.servo_pwm = angle;

        // Serial.print(cmd.motor_pwm);
        // Serial.print(",");
        // Serial.print(cmd.servo_pwm);
        // Serial.print(",");//remove
        // Serial.println(cmd.dir);//remove

        k_send(cmdMsgQ, &cmd);

        memset(serialString, ' ', sizeof(serialString));
        stringFinished = false;
      }


      if(idx > 10){
        memset(serialString, ' ', sizeof(serialString));
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

    
    k_sleep(50);
    
  }
}



// the setup function set up the three tasks, the one timer semerphore and one mutex and two queue up
void setup() {
  Serial.begin(115200);
  while(!Serial)
  {
    
  }
  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  resetPin(dirPinA);
  resetPin(dirPinB);

  attachInterrupt(digitalPinToInterrupt(intL_pin),encoderLInterrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(intR_pin),encoderRInterrupt,CHANGE);


  k_init(3, 1, 2); // init with space for 3 tasks, 1 semaphore, 2 message queue

  cmdMsgQ = k_crt_send_Q(5, sizeof(CMD), dataBufForCMDMsgQ); // size, size of data type, buffer array
  speedMsgQ = k_crt_send_Q(5, sizeof(float), dataBufForCurrentSpeedMsgQ);

  // Task 1 and 2 take 1180µs when running at 100 speed_ and 9120µs between them.
  p1 = k_crt_task(ReadEncoders, 10, st1, STK); // t1 as task, priority 10, 100 B stak
  p2 = k_crt_task(PWM, 10 , st2, STK); // t1 as task, priority 10, 100 B stak
  // task 4 takes 256µs worstcase and 4 when not recieving
  p4 = k_crt_task(ReadSerial, 11 , st4, STK);

  semmutex = k_crt_sem(1, 10);


  // Serial.println("starting");
  err = k_start(); // 1 milli sec tick speed
  Serial.print("start error: ");
  Serial.print(err);
}

void loop() {
  // Not used
}