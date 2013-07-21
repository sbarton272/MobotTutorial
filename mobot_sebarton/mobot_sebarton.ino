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
  - follow line (inside) - need better lost line decision making
  - On/off switch
  - Interpret sensor outside: line/ground?
  - Follow line outside (basic algorithm)
  - More sophisticated algorithm
  
  - Increase drive PWM percentage when turning, both are drawing from the same source
 */

/*********************************************************************
 * Constants 
 ********************************************************************/

// toggle DEBUG mode pre-compile
const boolean DEBUG = true;

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
const int SPEED_STOP  = 0;
const int SPEED_EIGTH = 32;
const int SPEED_QUART = 64;
const int SPEED_HALF  = 127;
const int SPEED_FULL  = 255;
const int SPEED_FWRD  = 150; // slowest speed it runs well at for driving
const int SPEED_TURN  = 230; // slowest speed it runs well at for turning

// definitions
const int FORWARD  =  1;
const int BACKWARD = -1;
const int STOP     =  0;
const int RIGHT    =  1;
const int LEFT     = -1;
const int STRAIGHT =  0;

// serial communication rate
const int SERIAL_BAUD = 9600;

// predefined times
const int INIT_DELAY = 3000; // 3 sec
const int SENSOR_TIMOUT = 4000; // 3 ms until sensor doesn't return value
const int LOST_TIMEOUT = 1000; // 1 sec timeout when lost to stop searching for line
  // @TODO: May not be necessary
const int SENSOR_SAFETY_DELAY = 10; // 10 us delay used to wait for sensor value

// Line detection values
const int BLACK_LINE_THRESHOLD = 2900;
 
/*********************************************************************
 * Global Variables
 ********************************************************************/
boolean ledOn = true; 
int globalSensorR = 0;
int globalSensorC = 0;
int globalSensorL = 0;
int lostTimer = 0;
 
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
  
  // Delay and turn off light
  delay(INIT_DELAY);
  digitalWrite(LED_PIN, LOW);

  if (DEBUG) {
    Serial.println("Spencer's Mobot Online"); 
    Serial.println("Debug Mode"); 
  }

} 

/*********************************************************************
 * Main 
 ********************************************************************/

/* main loop
 *  
 */
void loop() { 
  char command = '\0';

  
  if (DEBUG){
    print_sensor_values();
  }
  
  read_digital_sensors();
  serial_driving_control();
  
  follow_line();
} 

/*********************************************************************
 * Line Follow Algorithm
 ********************************************************************/

/* Basic Algoithm using the 3 sensors. O marks line seen, - marks 
 *  no line seen.
 *   Left   Center    Right     Turn      Move
 * 1   -       -         -       Stop      Stop
 * 2   -       -         O       Right     Forward - slow
 * 3   -       O         -       Straight  Forward - fast      
 * 4   -       O         O       Right     Forward - slow
 * 5   O       -         -       Left      Forward - slow      
 * 6   O       -         O       Stop      Stop (error state)      
 * 7   O       O         -       Left      Forward - slow
 * 8   O       O         O       Straight  Forward - fast      
 */

/* using the 3 sensor inputs decide how to turn and drive. Uses the above
 *  table. No input or output.
 */
void follow_line(void) {

  boolean onLineL, onLineC, onLineR;

  // do sesors see the line?
  onLineL = sensor_see_line( globalSensorL );
  onLineC = sensor_see_line( globalSensorC );
  onLineR = sensor_see_line( globalSensorR );

  if ( !onLineL && !onLineC && !onLineR ) {
      // turn: stop, drive: stop
      // LOST - continue the previous action for 1 sec.

      if ( lostTimer == 0 ) {
        // start lost timer
        lostTimer = millis(); // get time in milli sec.

      } else if ( lostTimer >= LOST_TIMEOUT ) {
        // timer overrun, so stop vehicle        
        drive( STOP, SPEED_STOP );
        turn( STRAIGHT );
      }

      if (DEBUG) { Serial.println("Line follow: 1 lost state"); };
      return;

  } else if ( !onLineL && !onLineC && onLineR ) {
      // turn: right, drive: forward slow
      drive( FORWARD, SPEED_FULL );
      turn( RIGHT );
      if (DEBUG) { Serial.println("Line follow: 2 right"); };

  } else if ( !onLineL && onLineC && !onLineR ) {
      // turn: stright, drive: forward fast
      drive( FORWARD, SPEED_HALF );
      turn( STRAIGHT );
      if (DEBUG) { Serial.println("Line follow: 3 forward"); };

  } else if ( !onLineL && onLineC && onLineR ) {
      // turn: right, drive: forward slow
      drive( FORWARD, SPEED_FULL );
      turn( RIGHT );
      if (DEBUG) { Serial.println("Line follow: 4 right"); };

  } else if ( onLineL && !onLineC && !onLineR ) {
      // turn: left, drive: forward slow
      drive( FORWARD, SPEED_FULL );
      turn( LEFT );
      if (DEBUG) { Serial.println("Line follow: 5 left"); };

  } else if ( onLineL && !onLineC && onLineR ) {
      // turn: stright, drive: stop
      // ERROR state as line only seen on edges
      drive( STOP, SPEED_STOP );
      turn( STRAIGHT );
      if (DEBUG) { Serial.println("Line follow: 6 error state"); }

  } else if ( onLineL && onLineC && !onLineR ) {
      // turn: left, drive: forward slow
      drive( FORWARD, SPEED_FULL );
      turn( LEFT );
      if (DEBUG) { Serial.println("Line follow: 7 left"); };

  } else if ( onLineL && onLineC && onLineR ) {
      // turn: stright, drive: forward fast
      drive( FORWARD, SPEED_HALF );
      turn( STRAIGHT );
      if (DEBUG) { Serial.println("Line follow: 8 forward"); };

  } else {
      // error state, not reachable
      Serial.println("Line follow: illegal state");
      return;
  }

  // reset lost timer
  lostTimer = 0;

}

/* determie if any of the sensors are seeing the line 
 *  the most basic way to do this is with a threshold
 *  @TODO: update to look at sensor differences, history
 *          and apply filtering
 */
boolean sensor_see_line( int sensorValue ) {

  return sensorValue > BLACK_LINE_THRESHOLD;

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
 * Sensor Functions
 ********************************************************************/

/* read sensor values and update sensor variables
 *   can only be used if the sensors are of the analog variety
 */
void read_analog_sensors() {
  
 globalSensorR = analogRead(SENSOR_R_PIN);
 globalSensorC = analogRead(SENSOR_C_PIN);
 globalSensorL = analogRead(SENSOR_L_PIN);
  
}

/* read sensor values and update sensor variables
 *   can only be used if the sensors are of the digital variety
 *   Code inspired from here http://bildr.org/2011/06/qre1113-arduino/
 */
void read_digital_sensors(){

  globalSensorR = ping_digital_sensor(SENSOR_R_PIN);
  globalSensorC = ping_digital_sensor(SENSOR_C_PIN);
  globalSensorL = ping_digital_sensor(SENSOR_L_PIN);
  
}

/* ping the sensor to read its value
 *   first send 1 to sensor to start read
 *   delay 10 microseconds before turning on input read to give time to safely convert to input
 *   time how long until a one is read
 *   The time is proportional to the value on the sensor
 */
int ping_digital_sensor(int pin) {
  
  // init values
  long startTime;
  int sensorTime;
  
  // ping sensor
  pinMode( pin, OUTPUT );
  digitalWrite( pin, HIGH );  
  
  // delay to safely switch to input mode
  delayMicroseconds(SENSOR_SAFETY_DELAY);
  pinMode( pin, INPUT );

  // start timer (get current time, safe to use as overflows after 70 min)
  startTime = micros();

  // time how long the input is HIGH, but quit after 3ms as nothing happens after that
  while ( (digitalRead(pin) == HIGH) && 
          ((micros() - startTime) < SENSOR_TIMOUT) ); 
  
  sensorTime = micros() - startTime;

  return sensorTime;
}

/*********************************************************************
 * Debugging Functions
 ********************************************************************/

/* toggleLED: using led_on var (global) decide to turn LED on/off */
void toggle_LED() {
  
  if (ledOn) {
    digitalWrite(LED_PIN, LOW);
    ledOn = false;
  } else {
    digitalWrite(LED_PIN, HIGH);
    ledOn = true;
  }  
  
}

/* Control mobot through serial interface
 *  A/a toggle LED
 *  F/f forward
 *  B/b backward
 *  R/r right
 *  L/l left
 *  S/s stop, straightens as well
 * Each command is executed for a predefined time (MOTOR_DELAY)
 */
void serial_driving_control() {
  char command = '\0';
 
  // check if command has been sent
  if ( Serial.available() ) {
    // read the most recent byte
    command = Serial.read();
    
    // interpret the command
    switch (command) {
      case 'a':
      case 'A':
        toggle_LED();
        break;
      case 'f':
      case 'F':
        drive( FORWARD, SPEED_HALF );
        break;
      case 'b':
      case 'B':
        drive( BACKWARD, SPEED_HALF );
        break;
      case 'r':
      case 'R':
        turn( RIGHT );
        break;
      case 'l':
      case 'L':
        turn( LEFT );
        break;
      case 's':
      case 'S':
        turn( STRAIGHT );
        drive( STOP, SPEED_STOP );
        break;
    }
  } 
  
}

/* print the current sensor values */
void print_sensor_values() {
  
  Serial.print("Sensors (R, C, L): \t");
  Serial.print(globalSensorR);
  Serial.print(",\t");
  Serial.print(globalSensorC);
  Serial.print(",\t");
  Serial.println(globalSensorL);
  
}
