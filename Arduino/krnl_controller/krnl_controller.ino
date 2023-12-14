#include <krnl.h>
#include <math.h>
#include <Servo.h>

#define STK 300

#define ARRAY_SIZE 5

const unsigned int intR_pin = 2;
const unsigned int intL_pin = 3;

const unsigned int encR_pin = 4;
const unsigned int encL_pin = 5;

const unsigned int dirPinA = 10;
const unsigned int dirPinB = 12;

const unsigned int pwm_pin = 6;

volatile unsigned long last_msg = 0;






struct k_t *p1, *p2, *p4, *semmutex;

struct k_msg_t *cmdMsgQ, *speedMsgQ;



char st1[STK], st2[STK], st4[STK];

volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;
int err;

struct CMD{
  float linear = 0;
  int angular = 0;
  unsigned long time = 0;
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
  float speed_rad_old = 0;

  const int ppr = 32; // pulses per revolution
  const float wheelRadius = 58;  // in mm
  const float wheelBase = 260;   // distance between the wheels in mm

  float distanceL = 0.0;
  float distanceR = 0.0;

  float speed_arr[ARRAY_SIZE] = {0.0};
  int currentIndex = 0;

  // float total_dis = 0.0;
  // float total_head = 0.0;

  // k_set_sem_timer(sem1,100);
  while(1)
  {

    distanceL = (2 * PI * wheelRadius) * encoderLPosition / ppr;
    distanceR = (2 * PI * wheelRadius) * encoderRPosition / ppr;

    float displacement = (distanceL + distanceR) / 2.0;
    float heading = (distanceR - distanceL) / wheelBase;

    encoderLPosition = 0;
    encoderRPosition = 0;

    // total_dis+=displacement;
    // total_head+=heading;
    
    // Serial.print(total_dis);
    // Serial.print(" ");
    // Serial.println(total_head);



    Serial.print(displacement);
    Serial.print(" ");
    Serial.println(heading); 



    // position_current = (encoderLPosition + encoderRPosition) / 2;

    // Position in radians
    // position_rad = (float)((M_PI*2*position_current/ppr));



    now = micros();

    // speed_ = (position_current-position_old); // tick per second
    
    speed_rad = (float)(((displacement)*1000000)/(now-lastTime_));

    speed_arr[currentIndex] = speed_rad;

    // Increment the index and wrap around if necessary
    currentIndex = (currentIndex + 1) % ARRAY_SIZE;

    // Calculate the average
    float sum = 0.0;
    for (int i = 0; i < ARRAY_SIZE; ++i) {
      sum += speed_arr[i];
    }
    float average = sum / ARRAY_SIZE;


    // speed_rad_old = speed_rad;

    res = k_send(speedMsgQ, &average);
    
    // position_old = position_current;
    // displacement_old = displacement;
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
  cmd.linear = 0;
  cmd.angular = 60;

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
  steering.attach(9);
  
  
  while(1)
  {
    k_wait(semmutex, 0);
    currentTime = millis();                
    elapsedTime = (double)(currentTime - previousTime);  

    res1 = k_receive(cmdMsgQ, &cmd, 1, &lost1);
    res3 = k_receive(speedMsgQ, &speed_rad, 1, &lost3);

    speed_rad = speed_rad/1000; //to m/s
    error = cmd.linear - speed_rad;
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

    if(motor_speed > 50){
      motor_speed = 50;
    }

    if(cmd.linear == 0.0 || cmd.time + 200 < millis()){
      analogWrite(pwm_pin, 0);
      resetPin(dirPinA);
      resetPin(dirPinB);
    }
    else{
      analogWrite(pwm_pin, motor_speed);
    }
  
    
    steering.write(cmd.angular);

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
  char serialString[30] = "";
  float speed = 0;
  float angle = 60;
  CMD cmd;

  // Takes 256 microseconds worst case
  while(1)
  {
    int idx = 0;
    
    while (Serial.available())
    {
      // char inChar = (char)Serial.read();
      char inChar = (char)Serial.read();

      if (inChar == '\n')    // The reading event stops at a new line character
      {
        serialTail = true;
        int secondValue = 0;
        speed = 0.0;
        angle = 0.0;
        
        for (int i = 0; i < sizeof(serialString); i++) {
          if (serialString[i] == ',') {
            char tempchar[10] = "";
            for(int x = 0; x < i; x++){
              tempchar[x] = serialString[x];
            }
            speed = atof(tempchar);
            secondValue = i + 1;
          }
          if (serialString[i] == ';') {
            char tempchar[10] = "";
            for(int y = 0; y < i - secondValue; y++){
              tempchar[y] = serialString[y + secondValue];
            }
            // angle = atof(tempchar);
            angle = atoi(tempchar);
          }
        }


        cmd.linear = speed;
        cmd.angular = angle;
        cmd.time = millis();

        // last_msg = millis();

        k_send(cmdMsgQ, &cmd);

        memset(serialString, ' ', sizeof(serialString));
        stringFinished = false;
      }


      if(idx > 30){
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
  // Serial.println("im up!");
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