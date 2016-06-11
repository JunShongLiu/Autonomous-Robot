/*
 * Multi-function Robot Project
 * ELEC291
 *
 * Take note that speed is controlled by setting the
 * PWM signal on E1 and E2 (range is 0-255)

 *
 * authors:
 *    Omar Dzick
 *    Neema Boutorabi
 *    Logan Ennis
 *    Shavon Zhang
 *    Justin Liu
 *    Mav Cuyugan
 */

#include "pins_and_constants.h"
#include "ultrasonic.c"
#include "robot_motion.c"
#include "line_follower.c"
#include "IRremote.h"
#include <Servo.h>
#include "infaredsensor.c"

// Global Variables
float right_wheel_speed;
float left_wheel_speed;
int dir = 0;
int left=0;
int right=0;
int left_previous = 0;
int right_previous = 0;
int IR_sensors[2];  // IR_sensors[0] = left sensor
                    // IR_sensors[1] = right sensor
int robot_speed;
int pos = 0;                        // variable to store the servo position
double distance;
//hall effect loop variables
volatile unsigned long rev_r;
volatile unsigned long rev_l;
float right_rps;
float left_rps;
unsigned long time_old_r;
unsigned long time_old_l;
unsigned long left_isr_time;
unsigned long right_isr_time;
int ignore_hall = 0;
int state = 0;
IRrecv irrecv(IR);
decode_results results;
Servo myservo ; //create servo object

void setup() {
 
  // initializing pins for motor shield
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  //initialize transistor power control pins
  pinMode(OPTICAL_SENSORS, OUTPUT);
  pinMode(SERVO_AND_ULTRASONIC,OUTPUT);

  //initializing range sensor pins
  pinMode(TRIGGER, OUTPUT);

  //pins for LEDs are outputs
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT) ;


  // Setup the LCD receiver
  irrecv.enableIRIn(); 

  //initialize Servo
  myservo.attach(SERVO) ;
  myservo.write(FORWARD);         //turn the servo to face in the forward direction

  //constants for hall effect sensor 
  rev_r = 0;
  rev_l = 0;
  time_old_r = 0;
  time_old_l = 0;
  left_isr_time = 0;
  right_isr_time = 0;

  //set hall effect pins to a falling edge interrupt
  attachInterrupt(digitalPinToInterrupt(HR), right_hall_effect_update, FALLING);
  attachInterrupt(digitalPinToInterrupt(HL), left_hall_effect_update, FALLING);

  //initial robot speed is maximal
  right_wheel_speed = 255;
  left_wheel_speed = 255;

  ignore_hall = 0;

  //Serial.begin(9600);
}

void loop() {

      // Block to set what mode we are in.
      if (irrecv.decode(&results)){
        irrecv.resume();
        // Switch to receive which key is being pressed on the remote.
        // and set the other variables accordingly
        switch(results.value){
        case UPARROW:
          dir = 1;
          break;
        case DOWNARROW:
          dir = 2;
          break;
        case LEFTARROW:
          dir = 3;
          break;
        case RIGHTARROW:
          dir = 4;
          break;
        case POWERKEY:
          state = 0;
          dir = 0;
          break;
        case STATE1:
          state = 1;
          dir = 0;
          break;
        case STATE2:
          state = 2;
          dir = 0;
          break;
        case STATE3:
          state = 3;
          dir = 0;
          break;
      }
    }

    // If the remote is not being pressed the robot does not move (in functionality 3)
    else{
      dir = 0;
    }


   // Functionality 1 (Movement with range sensor)
   if (state == 1){

    //turn on power to appropriate devices
    digitalWrite(SERVO_AND_ULTRASONIC, HIGH);
    digitalWrite(OPTICAL_SENSORS, LOW);
 

    digitalWrite(RED, LOW) ;
    digitalWrite(GREEN, HIGH) ;
    digitalWrite(BLUE, HIGH) ;


   distance = closestObject();      // Continuously scan for obstructions
   //Serial.println(distance);

  // MOTION TESTING
  //robot_forward(robot_speed);
  //delay(1000);
  //turn_right(0);
  //delay(1000);
  //turn_left(0);
  //delay(1000);
  //robot_forward(robot_speed);
  //delay(1000);
  //robot_break();
  //delay(1000);
  
  /*----DEBUG OPTIONS----*/
  //Serial.println(distance, 3);
 // delay(1000);

  //debug_turning_test();

  //ReadSensor();
  /*----END OF DEBUG OPTIONS----*/

  //robot_forward(robot_speed);
  //memcpy(IR_sensors, readSensor(), 2*sizeof(int));

 
  // TURN ANGLE TESTING
  // turn_right(180);
  // delay(1000);
  // turn_left(180);
  // delay(1000);

   ignore_hall = 0;             //flag to tell interrupts to count    
                                                                                    
   update_rps();               //updates current wheel rpm, and adjusts speed accordingly
   robot_forward(left_wheel_speed, right_wheel_speed);         // Move forward

  if(distance <= WARNING_DISTANCE) {                    //if distance is under threshold, decrease speed
    float right_distance;
    float left_distance;

    ignore_hall = 1;          //since we are slowing down and turning we don't want the revolutions in this loop to count towards a wheel rps

    //make both wheel speed the same as the lowest current wheel speed.
    if(left_wheel_speed < right_wheel_speed){
      left_wheel_speed = right_wheel_speed;
    }

    if(right_wheel_speed < left_wheel_speed){
      right_wheel_speed = left_wheel_speed;
    }

    // decelerate at steady and observable pace
    while(left_wheel_speed > 0 || right_wheel_speed > 0){
      distance = closestObject();
      left_wheel_speed -= 45;
      right_wheel_speed -= 45;
      
      if(left_wheel_speed < 0){
        left_wheel_speed = 0;
      }
      if(right_wheel_speed < 0){
        right_wheel_speed = 0;
      }

      robot_forward(left_wheel_speed, right_wheel_speed);         // Move forward
      // if distance to object is less than minimum threshold, break and set speed to 0
      if ((distance) < STOP_DISTANCE)
        robot_break();
    }
    
    turn() ;
    delay(100);

    left_wheel_speed = 248;
    right_wheel_speed = 247;
  }
  }

   // Functionality 2 (Line follower)  
   else if(state == 2){

    ignore_hall = 1;             //flag to tell interrupts not to count

    digitalWrite(RED, HIGH) ;
    digitalWrite(GREEN, LOW) ;
    digitalWrite(BLUE, HIGH) ;


    //turn on power to appropriate devices
    digitalWrite(SERVO_AND_ULTRASONIC, LOW);
    digitalWrite(OPTICAL_SENSORS, HIGH);

     int left_read = readLeftSensor();

     //Serial.println("LEFT: ");
     //Serial.println(left_read);
     //delay(10);
     int right_read = readRightSensor();
     //Serial.println("RIGHT: ");
     //Serial.println(right_read);
     //delay(10);
    
     if (left_read >= BLACK_SENSOR_READING){
       left = 1;  
     }
     else {
       left = 0;
     }
     if(right_read >= BLACK_SENSOR_READING) {
      right = 1;
     }
     else {
      right = 0;
     }
     Serial.println(followBlackLine(left, right));
     Serial.print("left ");
     Serial.println(left);
     Serial.print("right ");
     Serial.println(right);
  }

   // Functionality 3 (Movement control by IR remote)
   else if (state == 3){

    digitalWrite(RED, HIGH) ;
    digitalWrite(GREEN, HIGH) ;
    digitalWrite(BLUE, LOW) ;

    //turn on power to appropriate devices
    digitalWrite(SERVO_AND_ULTRASONIC, LOW);
    digitalWrite(OPTICAL_SENSORS, LOW);

    // Use the hall effect sensor when the robot is moving forwards
    if(dir == 1){
      update_rps();
    }
    
    // Move according to the current button being pressed
    moveInDirection(dir, right_wheel_speed, left_wheel_speed);
    // Time delay for accuracy of movement
    delay(250);
   }

   // State to do nothing (If the power key on the remote was pressed)
   else if (state == 0){
    robot_break() ;
    

    digitalWrite(RED, HIGH) ;
    digitalWrite(GREEN, HIGH) ;
    digitalWrite(BLUE, HIGH) ;

    //turn on power to appropriate devices
    digitalWrite(SERVO_AND_ULTRASONIC, LOW);
    digitalWrite(OPTICAL_SENSORS, LOW);


    digitalWrite(RED, HIGH) ;
    digitalWrite(GREEN, HIGH) ;
    digitalWrite(BLUE, HIGH) ;

    }
}


/**************************************************************************************
 * This function detects whether there is an obstruction in the direction that the
 * range sensor is facing
 *************************************************************************************/

double closestObject (){
  
  triggerPulse();
  double echo_duration = pulseIn(ECHO, HIGH);
  delay(100);
  //speed_with_temp is simply a constant since we removed the anolog temperature sensor
  double speed_with_temp = sound_speed_with_temp();
  double distance = calculate_distance(echo_duration, speed_with_temp);
  //Serial.println(distance, 2);
  return distance;

}

/*************************************************************************************
 * This function turns the Servo turns from left to right, returning the direction that
 * has the furthest obstruction.
 *************************************************************************************/
int turnServo(){
  double distance ;
  double maxDistance = 0 ;
  int position_farthest = -1 ;

  myservo.write(LEFT) ; //turn servo to the left and scan for the closest obstruction
  delay(1000);
  //Serial.println("Servo is turning to the right") ;
  distance = closestObject() ;      //record left if the maxDistance is larger than the current maxDistance
  if(distance > maxDistance && distance > WARNING_DISTANCE){     
    maxDistance = distance ;        
    position_farthest = LEFT ;
   }
  delay(1000);
  myservo.write(RIGHT) ;
  delay(1000);
   //Serial.println("Servo is turning to the left") ;
  distance = closestObject() ;      
  if(distance > maxDistance && distance > WARNING_DISTANCE){
    maxDistance = distance ;
    position_farthest = RIGHT ;
  }
  delay(1000);
  myservo.write(FORWARD);           //return the servo to face in the forward direction
  delay(1000);
  return position_farthest ;
}

/**************************************************************************************
 * This function turns the robot in the direction with the furthest obstruction
 *************************************************************************************/
void turn(){
  int position_farthest = 0 ;
  position_farthest = turnServo() ;

  if(position_farthest == RIGHT){
    turn_left(90) ;
  }
  else if (position_farthest == LEFT){
    turn_right(90) ;
  }
  else{
    turn_left(180) ;        //if there are obstructions in both the left and right, turn around
    }

}

/**************************
 * DEBUG FUNCTIONS
 **************************/

void debug_motion_test() {
  robot_forward(left_wheel_speed, right_wheel_speed);
  delay(1000);
  turn_right(0);
  delay(1000);
  turn_left(0);
  delay(1000);
  robot_forward(left_wheel_speed, right_wheel_speed);
  delay(1000);
  robot_break();
  delay(1000);
}

void debug_turning_test() {
  turn_right(180);
  delay(1000);
  turn_left(180);
  delay(1000);

}
/**************************
 * HALL EFFECT FUNCTIONS
 **************************/

/*********************************************************************
 * updates the rotations per second value for both wheels by comparing the current time
 * and the time we took the last reading and multiplying that value by the number
 * of revolutions that have occured for both wheels during that time
 **********************************************************************/
 void update_rps(){

      //onlly update after 4 revolutions to get an accurate reading
      if (rev_r >= 2 && rev_l >= 2) { 

     //not really revolutions per second, but we only use this value 
      //in a ratio later
      right_rps = (float)(millis() - time_old_r)*rev_r;
      time_old_r = millis();
       
      //Serial.println(time_old_r);
      //Serial.println(right_rps);

      //not really revolutions per second, but we only use this value 
      //in a ratio later
      left_rps = (float)(millis() - time_old_l)*rev_l;
      time_old_l = millis();

      //Serial.println(time_old_l);
      //Serial.println(left_rps);

      //adjust speed given new rps values
      update_speed();
      //reset revolution counter
      rev_r = 0;
      rev_l = 0;
      }
     
}

/*********************************************************************
 * This function updates the amount of power that is supplied to
 * both motor servos according to how different the rps values are 
 * for the left and right wheels
 **********************************************************************/
 
void update_speed(){

    //if the ratio of rps is under a threshold, we know something went wrong in
    //the interrupt counter, so we ignore it and set the pwm to a known value
    //that moves relativley straight, and then adjusts from there.
    if((right_rps/left_rps) < 0.8 || (left_rps/right_rps) < 0.8 )
    {
      right_wheel_speed = 247;
      left_wheel_speed = 248;
    }
    else{
    //adjust the speed of the wheels relative to wheel rps
    if(right_rps < left_rps){  
     right_wheel_speed = (right_rps/left_rps) * 255;
    }
    if(left_rps < right_rps){
     left_wheel_speed = (left_rps/right_rps) * 255;
    }
    }
    
}
/**************************
 * ISR FUNCTIONS
 **************************/


/*********************************************************************
 * This function updates the number of revolutions for the right wheel
 * whenver the magnet on the right wheel passes the hall effect sensor
 **********************************************************************/
void left_hall_effect_update(){
  //if the time since the last interrupt is not too small, and the robot is
  //not turning, count this revolution to the overall revolutions.
  if((millis() - left_isr_time) > 50 && !ignore_hall){
  rev_l++;  
//  Serial.println("left revs: ");
//    Serial.println(rev_l);
  }
  left_isr_time = millis();
}

/*********************************************************************
 * This function updates the number of revolutions for the right wheel whenever 
 * the magnet on the right wheel passes the hall effect sensor
 **********************************************************************/
void right_hall_effect_update(){
  //if the time since the last interrupt is not too small, and the robot is
  //not turning, count this revolution to the overall revolutions.
  if((millis() - right_isr_time) > 50 && !ignore_hall){
  rev_r++;  
//  Serial.println("right revs: ");
  //  Serial.println(rev_r);
  }
  right_isr_time = millis();
}
