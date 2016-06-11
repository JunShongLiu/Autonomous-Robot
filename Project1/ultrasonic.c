#include "pins_and_constants.h"

/**********************************************************************
 * sets the rangefinder trigger high
 **********************************************************************/
void triggerPulse(){
  digitalWrite(TRIGGER, HIGH);
  delay(0.01);
  digitalWrite(TRIGGER, LOW);
}

/**********************************************************************
 * calculates the speed of sound, accounting for temperature read from sensor
 **********************************************************************/
float sound_speed_with_temp(){
  //float temperature = read_temp();

  return 10000.0 / (SPEED_OF_SOUND + (0.6 * 22.0));
}

/**********************************************************************
 * measures distance using the time taken for the trigger signal to reach the closest
 * object and return, and the speed of sound
 **********************************************************************/
float calculate_distance(double echo_duration, double speed_of_sound){
   return  (echo_duration / (speed_of_sound * 2.0) ) / 100.0;

}

