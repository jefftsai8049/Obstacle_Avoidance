/*
 * NTU BIME Principles and Applications of Microcontrollers-Mechatronics (1) Yan-Fu Kuo
 * 
 * Final Project : Obstacle Avoidance
 * 
 * Created 2016-02-04 by Ching-Wei Tsai
 * Modified 2016-03-22 by Ching-Wei Tsai
 * Modified 2017-04-10 by Ching-Wei Tsai
 * 
 * Design and build a line following robot.
 * The robot needs to have a function that detect and bypass obstacles on its path automatically.
 * The robot returns to its path of the black line after bypassing the obstacles.
 * 
 * Required Material:
 * Wheel robot set x1
 * H-bridge board x1
 * IR Module x3
 * DMS Module x1
 * 
 */

/* Header files include */
#include <Timer.h>
#include <SoftwareSerial.h>
#define IRSensorPinR 16 //for A2
#define IRSensorPinL 15 //for A1
#define DMSSensorPin 14 //for A0

/* Default value setting */
#define CVAL 5
#define SCAN_RANGE 8
#define MOTOR_OUTPUT_SCALE 2.55
#define MAX_FORWARD_SPEED 100
#define MAX_BACKWARD_SPEED -100

/* Debug message is output or not */
#define DEBUG_MODE 0

/* Bluetooth device exist or not */
#define BLUE_TOOTH 0

/* For debug message output */
SoftwareSerial BT(3, 2);

/* Pin map setting*/
char motorIn[4] = {5,6,10,11};
float IRVal[2];
int DMSVal[1];

/* Timer for update the sensor value */
Timer sensorClock;

/*
Purpose : Update all sensors value
Input   : N/A
Ouput   : N/A
Return  : N/A
*/
void updateSensor() {
   getIRStatus(IRVal); 
   getDMSStatus(DMSVal);
}


/*
Purpose : Drive all moters
Input   : speedL - Left wheel speed (-100~100)
          speedR - Right wheel speed (-100~100)
Ouput   : N/A
Return  : N/A
*/
void moveMotor(int speedL,int speedR) {
  /* Speed range -100~+100 */
  int motorSpeed[4];

  /* Left wheel speed asign */
  speedL = (speedL > MAX_FORWARD_SPEED)? MAX_FORWARD_SPEED : speedL;
  speedL = (speedL < MAX_BACKWARD_SPEED)? MAX_BACKWARD_SPEED : speedL;
  motorSpeed[0] = (speedL >= 0)? 0 : MOTOR_OUTPUT_SCALE*(-speedL);
  motorSpeed[1] = (speedL >= 0)? MOTOR_OUTPUT_SCALE*speedL : 0;

  /* Right wheel speed asign */
  speedR = (speedR > MAX_FORWARD_SPEED)? MAX_FORWARD_SPEED : speedR;
  speedR = (speedR < MAX_BACKWARD_SPEED)? MAX_BACKWARD_SPEED : speedR;
  motorSpeed[2] = (speedR >= 0) ? 0 : MOTOR_OUTPUT_SCALE*(-speedR);
  motorSpeed[3] = (speedR >= 0) ? MOTOR_OUTPUT_SCALE*speedR : 0;

  /* Output to motor */
  for(int i = 0; i < 4; i++) {
    analogWrite(motorIn[i],motorSpeed[i]);
  }
}

/*
Purpose : Read IR sensor value
Input   : N/A
Ouput   : val - Pointer for IR sensor values
Return  : N/A
*/
void getIRStatus(float* val) {
  val[0] = analogRead(IRSensorPinL);
  val[1] = analogRead(IRSensorPinR);
}

/*
Purpose : Read DMS sensor value
Input   : N/A
Ouput   : val - Pointer for DMS sensor values
Return  : N/A
*/
void getDMSStatus(int* val) {
  float DMSVal= analogRead(DMSSensorPin);
  val[0] = DMSVal;
}

void setup() {
  /* Pin mode setup */
  pinMode(IRSensorPinL,INPUT);
  pinMode(IRSensorPinR,INPUT);
  pinMode(DMSSensorPin,INPUT);
  for(int i = 0; i < 4 ;i++) {
    pinMode(motorIn[i],OUTPUT);
  }
  
  /* Set sensor update duration */
  sensorClock.every(1,updateSensor);

#ifdef DEBUG_MODE
  /* For output debug */
  Serial.begin(9600);
  Serial.println("Start");

#ifdef BLUE_TOOTH
  BT.begin(9600);
  BT.println("Start");
#endif /* BLUE_TOOTH */
#endif /* DEBUG_MODE */

  /* Update timer clock */
  sensorClock.update();

  /* Delay for system warm up */
  delay(200);
}

void turnLeft() {
    moveMotor(20+CVAL,-20);
    delay(300);
}

void loop() {
  /* update clock */
  sensorClock.update();
  static boolean isWall = 0; 

  float Kpd = 0.05;
  float Kpa = 0.004;
  float delta = IRVal[0]-IRVal[1];
  float avg = (IRVal[0]+IRVal[1])/2;
  
  if (DMSVal[0] > 620 || isWall) {
    isWall = 1;
    moveMotor(20+CVAL,-20);
    delay(10);
  }
  else {
      moveMotor(20-delta*Kpd-avg*Kpa+CVAL,20+delta*Kpd+avg*Kpa);
  }
  if(DMSVal[0]  < 300)
      isWall = 0;

#ifdef DEBUG_MODE
  Serial.print("IR Value L:");
  Serial.print(IRVal[0]);
  Serial.print("  R:");
  Serial.print(IRVal[1]);
  Serial.print("   Distance:");
  Serial.println(DMSVal[0]);
#endif
 
}



  
