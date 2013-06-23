#ifndef CONTROLLER_H
#define CONTROLLER_H

// Constants
#define LED_PIN          13
#define MOTOR_ENABLE_PIN 7
#define FORWARD_PWM_PIN  5
#define BACKWARD_PWM_PIN 3

// Motor analog default values

#define MOTOR_100  255
#define MOTOR_50   127
#define MOTOR_0    0

 
#define SERIAL_BAUD 9600

#define MOTOR_DELAY 3000 // 3 sec
#define INIT_DELAY  3000 // 3 sec
 
#define FORWARD   1
#define BACKWARD  2
#define RIGHT     3
#define LEFT      4




void toggleLED( void );
int moveMotor( int movement, int time );


#endif
