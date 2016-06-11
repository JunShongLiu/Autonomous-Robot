#include "pins_and_constants.h"

//Speed the robot will go at

#define SPEED 95

int readLeftSensor(void);
int readRightSensor(void);
int followBlackLine(int, int);
void pivot_left();
void pivot_right();

/**************************************
 * Reads left optical reflective sensor
 *************************************/
int readLeftSensor(){
  return analogRead(LEFT_SENSOR);
}

/***************************************
 * Reads right optical reflective sensor
 ***************************************/
int readRightSensor(){
  return analogRead(RIGHT_SENSOR);
}

/**********************************************************************
 * Adjust wheel's speeds depending on whether on what the sensors read
 **********************************************************************/
int followBlackLine(int leftSensor, int rightSensor){

  //If both sensors see black then have robot pivot to the left until both sees white
  if (leftSensor==1 && rightSensor==1){
    pivot_left();
    return 0;
  }
  //If the left sensor sees black then pivot to the left
  else if (leftSensor ==1 && rightSensor==0){
    pivot_left();
    return 1;
  }
  //If the right sensor sees black then pivot to the right
  else if(rightSensor == 1 && leftSensor==0){
    pivot_right();
    return 0;
  }
  //If both sensors see white, drive forward
  else {
    robot_forward(SPEED, SPEED);
    return 0;
  }
}

/*****************
 * Turn left
 *****************/
void pivot_left(){
  analogWrite(E1, 10);
  analogWrite(E2, SPEED + 20);
}

/*****************
 * Turn right
 *****************/
void pivot_right(){
  analogWrite(E1, SPEED + 5);
  analogWrite(E2, 10);
}


