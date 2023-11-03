#include <krnl.h>
#include <math.h>

#define STK 110

uint8_t intL_pin = 2;
uint8_t intR_pin = 3;

uint8_t encL_pin = 6;
uint8_t encR_pin = 5;

uint8_t dirPinA = 10;
uint8_t dirPinB = 12;

uint8_t pwm_pin = 11;

// uint8_t analog_pin = A0;

struct k_t *p1, *p2, *p3, *p4, *sem1, *sem2, *sem3, *sem4, *semmutex;

struct k_msg_t *cmdMsgQ, *potMsgQ, *speedMsgQ;



char st1[STK], st2[STK], st3[STK], st4[STK];

volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;
int err;

struct CMD{
  float P;
  float I;
  float D;
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
  if (digitalRead(encL_pin)){
    encoderLPosition++;
  }
  else{
    encoderLPosition--;
  }
  interrupts();
  
  // if (digitalRead(intA_pin) == digitalRead(6/*intB_pin*/)) { //Skal du ikke bare læse om intA_pin er høj eller lav
  //   encoderPosition--;
  // } else {
  //   encoderPosition++;
  // }
  
}

// interrupt 2 function
void encoderRInterrupt() {
  noInterrupts();
  if (digitalRead(encR_pin)){
    encoderRPosition++;
  }
  else{
    encoderRPosition--;
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
void ReadEncoder()
{
  char res;
  int position_current = 0;
  float position_rad = 0.0;
  float position_old_rad = 0.0;
  float speed_rad = 0.0;
  int position_old = 0;
  int speed_ = 0;
  long now = 0;
  volatile long lastTime_ = 0;
  int ppr = 32;//2048; // pulses per revolution
  k_set_sem_timer(sem1,100);
  while(1)
  {
    k_wait(sem1, 0);
    k_wait(semmutex, 0);
    // setPin(8);
    
    position_current = encoderLPosition;
    Serial.println(position_current);

    position_rad = (float)((M_PI*2*position_current/ppr));

    now = micros();

    //Serial.printf("sp: %f\n",position_rad);
    speed_ = (position_current-position_old); // tick per second
    
    speed_rad =  (float)(((position_rad-position_old_rad)*1000000)/(now-lastTime_));
    //Serial.printf("s: %f\n",speed_rad);
    
    res = k_send(speedMsgQ, &speed_rad);
    
    // Serial.println(position_rad);
    position_old = position_current;
    position_old_rad = position_rad;
    lastTime_ = now;

    // resetPin(8);
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
  cmd.P = 0.01;
  cmd.I = 0.005;
  cmd.D = 0.0005;

  char res1, res2, res3;
  int lost1, lost2, lost3;
  
  k_set_sem_timer(sem2,100);
  while(1)
  {
    k_wait(sem2, 0);
    k_wait(semmutex, 0);


    currentTime = millis();                //get current time
    elapsedTime = (double)(currentTime - previousTime);  
  
    res1 = k_receive(cmdMsgQ, &cmd, 1, &lost1);
    res2 = k_receive(potMsgQ, &goal_speed, 1 ,&lost2);
    res3 = k_receive(speedMsgQ, &speed_rad, 1, &lost3);
    //Serial.print("r");
    //Serial.println(speed_rad);
    //Serial.print("pot");
    //Serial.println(goal_speed);
    error = goal_speed - speed_rad;
    error_sum += error * elapsedTime;  
    derror = (error - previous_error)/elapsedTime;

  // Calculate the motor speed using cmd equation
  //   speed_ = (cmd.P * error) + (cmd.I * error_sum) + (cmd.D * derror);
   
    //Serial.print("s");
    //Serial.println(speed_);
    //motor_speed = (int)(mapf(abs(speed_),0,62.8318530718,0,255));
    motor_speed = (int)(speed_);
    //Serial.print("d");
    //Serial.println(motor_speed);

    if(speed_ < 0){ //backward
      // digitalWrite(backwardDirection, HIGH);
      // digitalWrite(forwardDirection, LOW);
      resetPin(dirPinA);
      setPin(dirPinB);
    }
    else if(speed_ > 0){ //forward
      // digitalWrite(backwardDirection, LOW);
      // digitalWrite(forwardDirection, HIGH);
      setPin(dirPinA);
      resetPin(dirPinB);
    }
    else{
      resetPin(dirPinA);
      resetPin(dirPinB);
    }



  //  if(speed_ <0 ){
  //   digitalWrite(motor_dir,LOW);
    
  //  }
  //  else if(speed_ >0){
  //   digitalWrite(motor_dir,HIGH);
  //  }
   if(motor_speed > 255){
    motor_speed = 255;
   }
   else if(motor_speed < -255){
    motor_speed = 255;
   }
  
  analogWrite(pwm_pin,motor_speed);
  //HS -- fin cmd regulator
  
  previous_error = error;
  previousTime = currentTime; 
  //k_eat_msec(100);

  k_signal(semmutex);
  k_sleep(90);
    
    
  }
}

// The read potentiometer task read the potentiometer and calculate the target speed and sending an putting it in the queue
// void ReadPotentiometer()
// {
//   pinMode(analog_pin,INPUT_PULLUP);
//   int goal_speed = 0;
//   float speed_ = 0;
//   int max_speed = 0;
//   char res;
//   k_set_sem_timer(sem3,100);
//   while(1)
//   {
//     k_wait(sem3, 0);

//     k_wait(semmutex, 0);
    
//     setPin(10);
    
//     goal_speed = map(analogRead(analog_pin),0, 1023, -1000, 1000);
//     speed_ = (float)((goal_speed/100)*2*M_PI);
//     res = k_send(potMsgQ, &speed_);

//     resetPin(10);
//     k_signal(semmutex);
//     k_sleep(90);
//   }
// }

// the read cmd serial check if there are something in the serial buffer to be read, if there is the cmd values are then put in the queue
void ReadSerial()
{
  bool serialTail = false;
  bool stringFinished = false; // Flag to indicate reception of a string after terminator is reached
  char serialString[10] = "          "; // Empty serial string variable

  k_set_sem_timer(sem4,1000);
  while(1)
  {
    k_wait(sem4, 0);

    //setPin(11);
    
    int idx = 0;
    // prevmillis2 = micros();

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

  attachInterrupt(digitalPinToInterrupt(intL_pin),encoderLInterrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(intR_pin),encoderRInterrupt,CHANGE);

  k_init(4, 5, 3); // init with space for three tasks

  cmdMsgQ = k_crt_send_Q(5, sizeof(CMD), dataBufForCMDMsgQ); // size, size of data type, buffer array
  potMsgQ = k_crt_send_Q(5, sizeof(float), dataBufForTargetSpeedMsgQ);
  speedMsgQ = k_crt_send_Q(5, sizeof(float), dataBufForCurrentSpeedMsgQ);
  // priority low number higher priority than higher number
  //Task 1
  p1 = k_crt_task(ReadEncoder, 10, st1, STK); // t1 as task, priority 10, 100 B stak
  //Task 2
  p2 = k_crt_task(PWM, 10 , st2, STK); // t1 as task, priority 10, 100 B stak

  // p3 = k_crt_task(ReadPotentiometer, 10 , st3, STK);

  p4 = k_crt_task(ReadSerial, 11 , st4, STK);

  sem1 = k_crt_sem(0,1);  

  sem2 = k_crt_sem(0,1);  

  sem3 = k_crt_sem(0,1);  

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