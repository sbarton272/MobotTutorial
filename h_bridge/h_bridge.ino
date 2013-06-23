/*
  Spencer Barton
  Mobot Tutorial
  
  Control RC car front motor via h-bridge. There are four buttons: forward, backward, right and left. 
  Serial is also used for debugging and control.
 
  Pins, Command Char:
  forward  - 0, f/F
  backward - 1, b/B
  right    - 2, r/R
  left     - 3, l/L
  
  Commands are sent to arduino via serial as command characters. 
  The LED can also be turned on/off with the character 'a'/'A'.
 
 */
 
#include "h_bridge.h"
 
// Globals
boolean  led_on = true;
 
void setup() {
  
  // Init serial
  Serial.begin(SERIAL_BAUD); 
  
  // Init btns: all begin output low, motor is off
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(FORWARD_PWM_PIN, OUTPUT);
  pinMode(BACKWARD_PWM_PIN, OUTPUT);

  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  digitalWrite(FORWARD_PWM_PIN, LOW);
  digitalWrite(BACKWARD_PWM_PIN, LOW);
  
  // init LED, set ON to signal init all done
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("Mobot online"); 
  
  // Delay and turn off light
  delay(INIT_DELAY);
  digitalWrite(LED_PIN, LOW);

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

/* toggleLED: using led_on var (global) decide to turn LED on/off */
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
 */
int moveMotor( int movement, int time ) {
  
  Serial.print("moveMotor: ");
  Serial.print(movement);
  Serial.print(" for ");
  Serial.println(time);
  
  // enable motor driver
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);
  
  switch (movement) {
    case (FORWARD):
      analogWrite(FORWARD_PWM_PIN, MOTOR_100);
      delay(time);
      analogWrite(FORWARD_PWM_PIN, MOTOR_0);
      break;
    
    case (BACKWARD):
      analogWrite(BACKWARD_PWM_PIN, MOTOR_100);
      delay(time);
      analogWrite(BACKWARD_PWM_PIN, MOTOR_0);
      break;
    
    // not yet developed
    case (RIGHT):
    case (LEFT):
      break;
    
    default: // error
      // disable motor driver
      digitalWrite(MOTOR_ENABLE_PIN, LOW);
      return -1;    
  }
  
  // disable motor driver
  digitalWrite(MOTOR_ENABLE_PIN, LOW);
  return 0;
}
