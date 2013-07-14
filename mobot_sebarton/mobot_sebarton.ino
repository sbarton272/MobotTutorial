/*
  Spencer Barton
  Mobot Tutorial
  
  Simple mobot configuration based off of a cheap RC car.
  Parts include:
    - RC Car w/ drive motor and turning solenoid
    - 3 QRE1113 Line Sensor Breakout from Sparkfun
    - Arduino Uno
    - SN754410 Quad Half H-Bridge
    
  More about Mobot is here: http://www.cs.cmu.edu/mobot/
  More on the CMU Robotics Club is here: http://roboticsclub.org/
 
 @TODO
 - Check right/left, forward/back pins correct
 - Send sensors via serial
 - drive and turn
 - serial debug interface
 
 */

/*********************************************************************
 * Constants 
 ********************************************************************/
// pins
const int LED_PIN           = 13;
const int DRIVE_ENABLE_PIN  = 4;
const int TURN_ENABLE_PIN   = 12;
const int FORWARD_PWM_PIN   = 10;
const int BACKWARD_PWM_PIN  = 5;
const int RIGHT_PWM_PIN     = 3;
const int LEFT_PWM_PIN      = 11;
const int SENSOR_R_PIN      = A0;
const int SENSOR_C_PIN      = A1;
const int SENSOR_L_PIN      = A2;
 
// motor speeds
const int SPEED_STOP = 0;
const int SPEED_HALF = 127;
const int SPEED_FULL = 255;

// definitions
const int FORWARD  =  1;
const int BACKWARD = -1;
const int STOP     =  0;
const int RIGHT    =  1;
const int LEFT     = -1;
const int STRAIGHT =  0;

// serial communication rate
const int SERIAL_BAUD = 9600;

// predefined delays
const int INIT_DELAY = 3000; // 3 sec
 
/*********************************************************************
 * Global Variables
 ********************************************************************/
boolean  led_on = true; 
 
/*********************************************************************
 * Set-up
 ********************************************************************/


/* Initialize all I/O, note analog read pins do not need initialization */
void setup() {
  
  // Init serial
  Serial.begin(SERIAL_BAUD); 
  
  // Init btns: all begin output low, motor is off
  pinMode(DRIVE_ENABLE_PIN, OUTPUT);
  pinMode(FORWARD_PWM_PIN, OUTPUT);
  pinMode(BACKWARD_PWM_PIN, OUTPUT);
  pinMode(TURN_ENABLE_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);

  digitalWrite(DRIVE_ENABLE_PIN, LOW);
  digitalWrite(FORWARD_PWM_PIN, LOW);
  digitalWrite(BACKWARD_PWM_PIN, LOW);
  digitalWrite(TURN_ENABLE_PIN, LOW);
  digitalWrite(RIGHT_PWM_PIN, LOW);
  digitalWrite(LEFT_PWM_PIN, LOW);
  
  // init LED, set ON to signal init all done
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.println("Spencer's Mobot Online"); 
  
  // Delay and turn off light
  delay(INIT_DELAY);
  digitalWrite(LED_PIN, LOW);

} 

/*********************************************************************
 * Main 
 ********************************************************************/

/* main loop
 *  
 */
void loop() { 

  // Go forward, right, left, backwards, stop
  drive(FORWARD, SPEED_FULL);
  delay(1000);
  turn(RIGHT);
  delay(1000);
  turn(LEFT);
  delay(1000);
  turn(STRAIGHT);
  drive(BACKWARD, SPEED_FULL);
  delay(1000);
  drive(STOP, SPEED_STOP);
  delay(100000000);

} 

/*********************************************************************
 * Motor Functions
 ********************************************************************/

/* drive( int movement, int speed )
 *  Set motor to drive forward, backwards or stop
 *  Direction is forward (positive), stop (zero), backwards (negative)
 */
void drive( int direction, int speed ) {
  
  if ( direction <= BACKWARD ) {
    
    // backward
    digitalWrite(DRIVE_ENABLE_PIN, HIGH);
    analogWrite(FORWARD_PWM_PIN, SPEED_STOP);
    analogWrite(BACKWARD_PWM_PIN, speed);
    
  } else if ( direction >= FORWARD ) {
    
    // forward 
    digitalWrite(DRIVE_ENABLE_PIN, HIGH);
    analogWrite(BACKWARD_PWM_PIN, SPEED_STOP);
    analogWrite(FORWARD_PWM_PIN, speed);
    
  } else {
    
    // stop motors
    analogWrite(FORWARD_PWM_PIN, SPEED_STOP);
    analogWrite(BACKWARD_PWM_PIN, SPEED_STOP);
    digitalWrite(DRIVE_ENABLE_PIN, LOW);
  
  }
}

/* turn( int direction )
 *  Set motor to turn right, left or straight
 *  Direction is right (positive), straight (zero), left (negative)
 */
void turn( int direction ) {
  
  if ( direction <= LEFT ) {
    
    // left
    digitalWrite(TURN_ENABLE_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, SPEED_STOP);
    analogWrite(LEFT_PWM_PIN, SPEED_FULL);
    
  } else if ( direction >= RIGHT ) {
    
    // right 
    digitalWrite(TURN_ENABLE_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, SPEED_STOP);
    analogWrite(RIGHT_PWM_PIN, SPEED_FULL);
    
  } else {
    
    // straight
    analogWrite(RIGHT_PWM_PIN, SPEED_STOP);
    analogWrite(LEFT_PWM_PIN, SPEED_STOP);
    digitalWrite(TURN_ENABLE_PIN, LOW);
  
  }
}

/*********************************************************************
 * Debugging Functions
 ********************************************************************/

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
