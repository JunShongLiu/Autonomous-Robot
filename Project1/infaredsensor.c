#include "pins_and_constants.h"
// Moves robot in a specific direction based on dir parameter.
void moveInDirection(int dir, float right_wheel_speed, float left_wheel_speed){
  // Stop
  if (dir == 0){
    robot_break();
  }
  // Forward
  else if (dir == 1){
    robot_forward(248, 245);
  }
  // Backwards
  else if (dir == 2){
    digitalWrite(M1,HIGH);
    digitalWrite(M2, LOW);
    analogWrite(E1, 150);   //PWM Speed Control
    analogWrite(E2, 150);   //PWM Speed Control
  }
  // Rotate left
  else if (dir == 3){
    digitalWrite(M1,HIGH);
    digitalWrite(M2, HIGH);
    analogWrite(E1, 75);
    analogWrite(E2, 75);
  }
  // Rotate right
  else if (dir == 4){
    digitalWrite(M1, LOW);
    digitalWrite(M2, LOW);
    analogWrite(E1, 75);
    analogWrite(E2, 75);
  }
}

