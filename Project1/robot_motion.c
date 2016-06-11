#include "pins_and_constants.h"

/*********************************************************************
 * Moves the robot forward by setting the speeds of the left wheel and the right wheel
 **********************************************************************/
void robot_forward(int speedL, int speedR) {
  digitalWrite(M1,LOW);   
  digitalWrite(M2, HIGH);       
  analogWrite(E1, speedL);   //PWM Speed Control left
  analogWrite(E2, speedR);   //PWM Speed Control right
}
/*********************************************************************
 * Stops the robot 
 **********************************************************************/
void robot_break() {
  analogWrite(E1, 0);   //PWM Speed Control
  analogWrite(E2, 0);   //PWM Speed Control
}
/*********************************************************************
 * rotates the Robot frame to the right by 'degrees' amount of degrees
 **********************************************************************/
void turn_right(int degrees) {
  digitalWrite(M1,LOW);
  digitalWrite(M2, LOW);
  analogWrite(E1, 255);
  analogWrite(E2, 255);

  double time_delay = degrees / 90;

  //the time delay was determined experimentally, if time delay = 1 then the robot should rotate 90 degrees
  
  delay(time_delay * 220 );
  robot_break();
}
/*********************************************************************
 * rotates the Robot frame to the left by 'degrees' amount of degrees
 **********************************************************************/
void turn_left(int degrees){
  digitalWrite(M1,HIGH);
  digitalWrite(M2, HIGH);
  analogWrite(E1, 255);
  analogWrite(E2, 255);

  double time_delay = degrees / 90;

  delay(time_delay * 220);
  robot_break();

}

