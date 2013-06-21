#ifndef CONTROLLER_H
#define CONTROLLER_H

// Constants
#define LED_PIN        13
#define FORWARD_PIN    0
#define BACKWARD_PIN   1
#define RIGHT_PIN      2
#define LEFT_PIN       3
 
#define SERIAL_BAUD 9600

#define MOTOR_DELAY 3000 // 3 sec
 
#define FORWARD   1
#define BACKWARD  2
#define RIGHT     3
#define LEFT      4

/*
enum {
  FORWARD  = 0,
  BACKWARD = 1,
  RIGHT    = 2,
  LEFT     = 3  
};
*/


void toggleLED( void );
int moveMotor( int movement, int time );


#endif
