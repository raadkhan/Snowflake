/* Firmware for IARRC 2017
 * Author: Vincent Yuan, 
 * Modified: Nick Wu
 * Modified: James Asefa
 * Modified: Jinhao Lu
 * Modified: Gareth Ellis
 * Modified: Valerian Ratu
 * Modified: Marinah Zhao
 * Modified: Raad Khan
 * Date Last Modified: July 12, 2018
 */

/*
 * UBC Snowbots - IARRC 2018
 * Firmware for Control of Olaf 3.0 (RC car)
 * 
 * This firmware will take in a message of the form:
 * `B<linear x> <linear y> <linear z> <angular x> <angular y> <angular z>`
 * where each `<>` is a single byte. The degree to rotate the servo controlling
 * the front wheels is determined from `angular z`, and the speed of the car by `linear x`
 * (the rest of the values are discarded)
 */

#include <SoftwareSerial.h>
#include <stdlib.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Encoder.h>

// !!!!WARNING!!!! 
// DEBUGGING might affect performance and alter normal behaviour
// !!!!WARNING!!!! 
// Uncomment this to enable debug message logging - messages sent over serial will be echoed back
//#define DEBUG_MESSAGES
// Uncomment this to enable PWM debug logging - PWM signals will be echoed
//#define DEBUG_PWM

#define BAUD_RATE 9600

// size of character buffer being passed over serial connection
#define BUFFER_SIZE 7

// Robot speed will be received in a char buffer with each value between 0 and 255
// as a result, this assigns 128 as stop. Then:
// 0 < speed < 128 is reverse
// 128 < speed <= 255 is forward
// 255 is then full speed forward, 0 is full speed backward
#define UNMAPPED_STOP_SPEED 128
#define MAPPED_STRAIGHT_ANGLE 90

// Robot will keep executing the last command unless the period exceeds the SAFETY_TIMEOUT
// SAFETY_TIMEOUT is in ms
#define SAFETY_TIMEOUT 500

#define encoderChannelA 2
#define encoderChannelB 3

void serial_read();
void convert();
void drive(int, int);

// buffer inputs
int linear_x = UNMAPPED_STOP_SPEED;
int linear_y = UNMAPPED_STOP_SPEED;
int linear_z = UNMAPPED_STOP_SPEED;
int angular_x = MAPPED_STRAIGHT_ANGLE;
int angular_y = MAPPED_STRAIGHT_ANGLE;
int angular_z = MAPPED_STRAIGHT_ANGLE;

// motor pins
const int DRIVING_MOTOR_PIN = A0;
const int STEERING_MOTOR_PIN = A2;

// defines start of buffer
const char BUFFER_HEAD = 'B';

// max and min linear speeds and stopping condition
const int LINEAR_MAX = 256;
const int LINEAR_MIN = 0;
const int LINEAR_STOP = 128;

// max and min angular speeds and stopping condition
const int ANGULAR_MAX = 180;
const int ANGULAR_MIN = 0;
const int ANGULAR_STOP = 90;

// The minimum and maximum PWM signals to map received values to
// This is the default for both the Servo and the Talon ESC
// 1000us - 2000us
const int MIN_PWM = 1000;
const int MAX_PWM = 2000;

// The safety cutoffs to prevent us drawing too much current
// from the motor
const int MIN_MOTOR_PWM_CUTOFF = 1300;
const int MAX_MOTOR_PWM_CUTOFF = 1700;

// The conversions for counts to radians and rad/s to m/s
const int CPR = 360; // Cycles Per Revolution
const int DPR = 360; // Degrees Per Revolution
const double COUNT_TO_RAD = DEG_TO_RAD * (DPR / (CPR * 4)); // CPR * 4 = Counts Per Revolution
const double ANG_TO_LIN_VEL = 0.05; // 0.05 = wheel radius (m)

// The encoder count, calculated angle, and calculated velocity values
volatile int encoder_count = 0;
volatile float angle_previous = 0,angle_next = 0;
double measured_velocity = 0, desired_velocity = 10, output_velocity;

// Counters for Timer2 to calculate the velocity and angle
volatile int tcnt2 = 131;
volatile int t = 0;

// PID values- adjust these as you see fit during testing
double Kp = 1, Ki = 1, Kd = 1;

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

Servo driving_motor;
Servo steering_motor;

Encoder encoder(encoderChannelA, encoderChannelB);

PID velocityPID(&measured_velocity, &output_velocity, &desired_velocity, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(BAUD_RATE);

    // Initialise the Servo and Motor connections
    driving_motor.attach(DRIVING_MOTOR_PIN, MIN_PWM, MAX_PWM);
    steering_motor.attach(STEERING_MOTOR_PIN, MIN_PWM, MAX_PWM);

    // Initialise the Encoder connections
    pinMode(encoderChannelA, INPUT);
    pinMode(encoderChannelB, INPUT);

    // Configure the PID veloctity controller
    velocityPID.SetMode(AUTOMATIC);
    velocityPID.SetTunings(Kp, Ki, Kd);
    
    // Set up Timer2: interrupts every 1 ms to measure and calculate velocity/angle
    noInterrupts();
    TIMSK2 &= ~(1<<TOIE2);
    TCCR2A &= ~((1<<WGM21) | (1<<WGM20));
    TCCR2B &= ~(1<<WGM22);
    ASSR &= ~(1<<AS2);
    TIMSK2 &= ~(1<<OCIE2A);
    TCCR2B |= (1<<CS22)  | (1<<CS20);
    TCCR2B &= ~(1<<CS21);
    TCNT2 = tcnt2;
    TIMSK2 |= (1<<TOIE2);
    interrupts();
}

void loop() {
    serial_read(); 
    convert();
    drive(linear_x, angular_z);
}

void serial_read(){
    // BUFFER_HEAD identifies the start of the buffer
    if ( Serial.available() >= BUFFER_SIZE && Serial.read() == BUFFER_HEAD ) {
        linear_x = Serial.read();
        linear_y = Serial.read();
        linear_z = Serial.read();
        angular_x = Serial.read();
        angular_y = Serial.read();
        angular_z = Serial.read();
        previousMillis = currentMillis;

#ifdef DEBUG_MESSAGES
        Serial.println("Got message");
        Serial.print("Linear X:"); Serial.println(linear_x);
        Serial.print("Linear Y:"); Serial.println(linear_y);
        Serial.print("Linear Z:"); Serial.println(linear_z);
        Serial.print("Angular X:"); Serial.println(angular_x);
        Serial.print("Angular Y:"); Serial.println(angular_y);
        Serial.print("Angular Z:"); Serial.println(angular_z);

        Serial.print("Encoder Count: ");
        Serial.print(encoder_count, DEC); 
        Serial.print("; ");
        Serial.print("Measured Velocity: ");
        Serial.print(measured_velocity, 4); 
        Serial.print("; ");
        Serial.print("Output Velocity: ");
        Serial.print(output_velocity, 4);
        Serial.print("; "); 
        Serial.print("Driving Motor Pin Signal: ");
        Serial.print(DRIVING_MOTOR_PIN, 4);
        Serial.print("; ");
        Serial.print("\n");
#endif
    } else {
        currentMillis = millis();
        if(currentMillis - previousMillis > SAFETY_TIMEOUT){
            linear_x = UNMAPPED_STOP_SPEED;
        }
    }
}

void convert() {
    // safety check if values are out of range
    if (linear_x > LINEAR_MAX)
        linear_x = LINEAR_MAX; 
    else if (linear_x < LINEAR_MIN)
        linear_x = LINEAR_MIN;

    if (angular_z > ANGULAR_MAX)
        angular_z = ANGULAR_MAX; 
    else if (angular_z < ANGULAR_MIN)
        angular_z = ANGULAR_MIN;
}

void drive(int linear_speed, int angular_speed){
    // Default velocity and angle to the midpoint in the PWM range
    int velocity = (MAX_PWM - MIN_PWM)/2;
    int angle = (MAX_PWM - MIN_PWM)/2;

    velocity = map(linear_speed, LINEAR_MIN, LINEAR_MAX, MIN_PWM, MAX_PWM); 
    angle = map(angular_speed, ANGULAR_MIN, ANGULAR_MAX, MIN_PWM, MAX_PWM);

    // Check that we're not outside our safety cutoffs
    if (velocity > MAX_MOTOR_PWM_CUTOFF)
        velocity = MAX_MOTOR_PWM_CUTOFF;
    else if (velocity < MIN_MOTOR_PWM_CUTOFF)
        velocity = MIN_MOTOR_PWM_CUTOFF;

    // Write the commands to the motor and the servo
    driving_motor.writeMicroseconds(velocity);
    steering_motor.writeMicroseconds(angle);

#ifdef DEBUG_PWM
    Serial.print(velocity);Serial.print(" / ");
    Serial.print(angle);Serial.print(" / ");
    Serial.print(linear_speed);Serial.print(" / ");
    Serial.print(angular_speed);Serial.print(" / ");
    Serial.println();
#endif
}

ISR(TIMER2_OVF_vect) {  
  TCNT2 = tcnt2;  // reload the timer
  t++;
  if (t == 1) {
    encoder_count = encoder.read();
    angle_previous = encoder_count * COUNT_TO_RAD;
  }
  else if (t == 20) {
    encoder_count = encoder.read();
    angle_next = encoder_count * COUNT_TO_RAD;
  }
  else if (t == 21) {
    // * 50 = 1000 ms/s / 20 ms
    measured_velocity = ANG_TO_LIN_VEL * (angle_next - angle_previous) * 50; 
    t = 0;
  }
}

