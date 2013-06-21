/*
  Spencer Barton
  Mobot Tutorial
  
  Control RC car via the remote control. There are four buttons: forward, backward, right and left. 
  Each is wired to the arduino and is a pull-down pin. Serial is also used for debugging.
 
  Pins, Command Char:
  forward  - 0, f/F
  backward - 1, b/B
  right    - 2, r/R
  left     - 3, l/L
  
  Commands are sent to arduino via serial as command characters. 
  The LED can also be turned on/off with the character 'a'/'A'.
 
 */
 
#include "controller.h"
 
// Globals
boolean  led_on = true;
 
void setup() {
  
  // Init serial
  Serial.begin(SERIAL_BAUD); 
  
  // Init btns: all begin output high
  pinMode(FORWARD_PIN, OUTPUT);
  pinMode(BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(LEFT_PIN, OUTPUT);
  
  digitalWrite(FORWARD_PIN, HIGH);
  digitalWrite(BACKWARD_PIN, HIGH);
  digitalWrite(RIGHT_PIN, HIGH);
  digitalWrite(LEFT_PIN, HIGH);
  
  // init LED, set ON to signal init all done
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("Mobot online"); 

} 


/* main loop
 *   listen on serial and interpret commands
 *   accepts these byte commands:
 *    a/A, f/F, b/B, r/R, l/L
 */
void loop() { 
  char command = '\0';
 
  // check if command has been sent
  if ( Serial.available() ) {
    // read the most recent byte
    command = Serial.read();
    
    // interpret the command
    switch (command) {
      case 'a':
      case 'A':
        toggleLED();
        break;
      case 'f':
      case 'F':
        moveMotor( FORWARD, MOTOR_DELAY );
        break;
      case 'b':
      case 'B':
         moveMotor( BACKWARD, MOTOR_DELAY );
         break;
      case 'r':
      case 'R':
         moveMotor( RIGHT, MOTOR_DELAY );
         break;
      case 'l':
      case 'L':
         moveMotor( LEFT, MOTOR_DELAY );
         break;
    }
  }
} 

/* toggleLED: using led_on var decide to turn LED on/off */
void toggleLED() {
  
  if (led_on) {
    digitalWrite(LED_PIN, LOW);
    led_on = false;
  } else {
    digitalWrite(LED_PIN, HIGH);
    led_on = true;
  }  
  
}

/* moveMotor( movement, time )
 *   move motor in one of four motions for given time (milliseconds)
 *   motions are FORWARD, BACKWARD, RIGHT, LEFT
 *   Returns 0 on success and -1 on error
 *   Motion is toggled by setting the correct pin LOW. 
 *   This sends a command via the controller to the car.
 */
int moveMotor( int movement, int time ) {
  
  switch (movement) {
    case (FORWARD):
      digitalWrite(FORWARD_PIN, LOW);
      delay(time);
      digitalWrite(FORWARD_PIN, HIGH);
      break;
    case (BACKWARD):
      digitalWrite(BACKWARD_PIN, LOW);
      delay(time);
      digitalWrite(BACKWARD_PIN, HIGH);
      break;
    case (RIGHT):
      digitalWrite(RIGHT_PIN, LOW);
      delay(time);
      digitalWrite(RIGHT_PIN, HIGH);
      break;
    case (LEFT):
      digitalWrite(LEFT_PIN, LOW);
      delay(time);
      digitalWrite(LEFT_PIN, HIGH);
      break;
    default:
      return -1;    
  }
  return 0;
}
