#include <krnl.h>
#include <math.h>
#include <Servo.h>

#define STK 110

const unsigned int intR_pin = 2;
const unsigned int intL_pin = 3;

const unsigned int encR_pin = 4;
const unsigned int encL_pin = 5;

const unsigned int dirPinA = 10;
const unsigned int dirPinB = 12;

const unsigned int pwm_pin = 6;


struct k_t *p1, *p2, *p3, *p4, *sem1, *sem2, *sem4, *semmutex;

struct k_msg_t *cmdMsgQ, *potMsgQ, *speedMsgQ;



char st1[STK], st2[STK], st3[STK], st4[STK];

volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;
int position_increment = 0;
int err;

struct CMD{
  uint8_t servo_pwm;
  uint8_t motor_pwm;
  bool dir; 
};

struct debug_pack{
  float sp;
  float tg;
  float pwm_sp;
  int mo_sp;
};

CMD dataBufForCMDMsgQ[5];
float dataBufForTargetSpeedMsgQ[5];
float dataBufForCurrentSpeedMsgQ[5];
debug_pack dataBufferForDebugMsgQ[5];


//HS -- fin forståelse for bitmanipulation
// function for quickly set pin from PORTB on the Arduino UNO
void setPin(int digPinNo){

  PORTB = PORTB|1<<(digPinNo-8); // her trækker så vi får pin nummerne til at passe, sætte vi in en pin høj med en 'or' operation hvor vi sætter den rigtige bit høj
}

// function for quickly reset pin from PORTB on the Arduino UNO
void resetPin(int digPinNo){
  PORTB = PORTB&~(1<<(digPinNo-8)); // her gør vi de samme dog bare med resette med en 'and'
}



float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Interrupt function
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

// interrupt 2 function
void encoderRInterrupt() {
  noInterrupts();
  if (digitalRead(encR_pin) == digitalRead(intR_pin)){
    encoderRPosition--;
  }
  else{
    encoderRPosition++;
  }
  interrupts();
  
  // if (digitalRead(intB_pin) == digitalRead(intA_pin)) {
  //   encoderPosition++;
  // } else {
  //   encoderPosition--;
  // }
}
//HS--hvorfor er der to stk ISR

// read encoder task, it takes the position and calculated the current speed of the motor, this is then send by the curret speed queue
void ReadEncoders()
{
  unsigned long long prevmicros = 0;
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
  

  k_set_sem_timer(sem1,100);
  while(1)
  {
    k_wait(sem1, 0);
    k_wait(semmutex, 0);
    prevmicros = micros();
    // if(pos_sent){ // move to where it is sent
    //   pos_sent = false;
    //   position_increment = 0;
    //   encoderLPosition = 0;
    //   encoderRPosition = 0;
    // }
    // analogWrite(pwm_pin, 20);
    // Serial.println(encoderLPosition);
    // position_current = (encoderLPosition + encoderRPosition) / 2;

    position_current = encoderLPosition;
    // Position in radians
    position_rad = (float)((M_PI*2*position_current/ppr));

    now = micros();

    //Serial.printf("sp: %f\n",position_rad);
    speed_ = (position_current-position_old); // tick per second
    
    speed_rad = (float)(((position_rad-position_old_rad)*1000000)/(now-lastTime_));
    //Serial.printf("s: %f\n",speed_rad);
    // Serial.println(speed_rad);
    res = k_send(speedMsgQ, &speed_rad);
    
    // Serial.println(position_rad);
    position_old = position_current;
    position_old_rad = position_rad;
    lastTime_ = now;
    int time = micros() - prevmicros;
    Serial.println(time);
    k_signal(semmutex); //Husk at mutexen kun skal dække operationer med delte variable, så tidsrummet i mutex bliver så lille som muligt
    k_sleep(90);
  }
}

// The PWM task the PID value, target speed and currernt speed from the queue, calculate the pwm signal and sitting the motor dir pin
void PWM()
{ 
  
  // pinMode(pwm_pin,OUTPUT);
  // pinMode(motor_dir, OUTPUT);
  double previous_error = 0;
  double error_sum = 0;
  double error = 0;
  double derror = 0;

  float goal_speed = 0;
  int motor_speed = 0;
  float speed_rad = 0;
  float speed_ = 0;
  long int currentTime = 0;
  long int previousTime = 0;
  double elapsedTime = 0;
  
  CMD cmd;
  cmd.servo_pwm = 60;
  cmd.motor_pwm = 0;
  float P = 2; 
  float I = 0;
  float D = 0.5;

  char res1, res2, res3;
  int lost1, lost2, lost3;

  Servo steering;
  steering.attach(9);
  
  k_set_sem_timer(sem2,100);
  while(1)
  {
    k_wait(sem2, 0);
    k_wait(semmutex, 0);
    
    currentTime = millis();                //get current time
    elapsedTime = (double)(currentTime - previousTime);  
  
    // goal speed, servo
    res1 = k_receive(cmdMsgQ, &cmd, 1, &lost1);
    // goal speed
    // res2 = k_receive(potMsgQ, &goal_speed, 1 ,&lost2);
    // current speed
    res3 = k_receive(speedMsgQ, &speed_rad, 1, &lost3);
    //Serial.print("r");
    //Serial.println(speed_rad);
    //Serial.print("pot");
    //Serial.println(goal_speed);
    error = goal_speed - speed_rad;
    error_sum += error * elapsedTime;  
    derror = (error - previous_error)/elapsedTime;

    // Calculate the motor speed using cmd equation
    speed_ = (P * error) + (I * error_sum) + (D * derror);
   
    //Serial.print("s");
    //Serial.println(speed_);
    //motor_speed = (int)(mapf(abs(speed_),0,62.8318530718,0,255));
    motor_speed = (int)(speed_);
    //Serial.print("d");
    //Serial.println(motor_speed);

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

    // if(motor_speed > 255){
    //   motor_speed = 0;
    // }
    // else if(motor_speed < -255){
    //   motor_speed = 0;
    // }

    if(motor_speed < 0){
      motor_speed = 0 - motor_speed;
    
    }

    if(motor_speed > 50){
      motor_speed = 50;
    }
  
    // analogWrite(pwm_pin,motor_speed);
    // Serial.println(motor_speed);
    steering.write(cmd.servo_pwm);
    
    previous_error = error;
    previousTime = currentTime; 
    //k_eat_msec(100);
    
    
    k_signal(semmutex);
    k_sleep(90);
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
  k_set_sem_timer(sem4,1000);
  while(1)
  {
    k_wait(sem4, 0);

    
    int idx = 0;
    // prevmillis2 = micros();

    while (Serial.available())
    {
      char inChar = (char)Serial.read();

      if (inChar == '\n')    // The reading event stops at a new line character
      {
        serialTail = true;
        int nonNullCount = 0;
        int secondValue = 0;
        // pwm1 = 0;
        // angle = 0;
        
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

        if(forward == 48){
          cmd.dir = true;
        }
        else{
          cmd.dir = false;
        }
        cmd.motor_pwm = pwm1;
        cmd.servo_pwm = angle;


        k_send(cmdMsgQ, &cmd);

        // if(serialString[0] == 'm'){
          
        // }
        
        // if(serialString[0] != 'm'){
        //   Serial.println("I,error msg[0] is unknown");
        // }


        memset(serialString, ' ', sizeof(serialString));
        stringFinished = false;
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

    //resetPin(11);
    k_sleep(990);
    
  }
}



// the setup function set  up the four task, the four timer semerphores and one mutex and threes queue up
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial)
  {
    
  }
  // for (int x=8; x <=13; x++) pinMode(x,OUTPUT);
  // pinMode(intA_pin,INPUT_PULLUP);
  // pinMode(6,INPUT_PULLUP);
  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  digitalWrite(dirPinA, LOW);
  digitalWrite(dirPinB, LOW);

  attachInterrupt(digitalPinToInterrupt(intL_pin),encoderLInterrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(intR_pin),encoderRInterrupt,CHANGE);

  k_init(3, 4, 3); // init with space for three tasks

  cmdMsgQ = k_crt_send_Q(5, sizeof(CMD), dataBufForCMDMsgQ); // size, size of data type, buffer array
  potMsgQ = k_crt_send_Q(5, sizeof(float), dataBufForTargetSpeedMsgQ);
  speedMsgQ = k_crt_send_Q(5, sizeof(float), dataBufForCurrentSpeedMsgQ);
  // priority low number higher priority than higher number
  //Task 1
  p1 = k_crt_task(ReadEncoders, 10, st1, STK); // t1 as task, priority 10, 100 B stak
  //Task 2
  p2 = k_crt_task(PWM, 10 , st2, STK); // t1 as task, priority 10, 100 B stak

  // p3 = k_crt_task(ReadPotentiometer, 10 , st3, STK);

  p4 = k_crt_task(ReadSerial, 11 , st4, STK);

  sem1 = k_crt_sem(0,1);  

  sem2 = k_crt_sem(0,1);

  sem4 = k_crt_sem(0,1);  

  semmutex = k_crt_sem(1, 10);

  
  //sem1 = k_crt_sem(0,5);
  Serial.println("starting");
  err = k_start(); // 1 milli sec tick speed
  Serial.print("start error: ");
  Serial.print(err);

}

void loop() {
  // put your main code here, to run repeatedly:

}