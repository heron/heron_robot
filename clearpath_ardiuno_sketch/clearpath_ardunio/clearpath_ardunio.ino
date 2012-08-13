// Kingfisher Ardiuno Firmware
// Author: James Servos
//
// Defines basic firmware behaviour for new kingfisher
//

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <kingfisher_msgs/Sense.h>
#include <kingfisher_msgs/Drive.h>

#include <Servo.h>

//PIN Defines
#define RIGHT_SERVO_PIN 10  //PB6
#define LEFT_SERVO_PIN   9  //PB5

#define RC_THROT_PIN 3   //PD0
#define RC_ROT_PIN   2   //PD1
#define RC_ENABLE_PIN 0  //PD2

#define RIGHT_LIGHT  4 //PD4
#define LEFT_LIGHT   12 //PD6

#define BATTERY_SENSE A5 //PF0

//constants
#define PULSE_NEUTRAL 1500
#define PULSE_DELTA   1000

#define RC_ROTATION_WEIGHT 0.5

#define LOW_BAT_LEVEL 1.5 //Volts

typedef enum {
  STATE_STANDBY,
  STATE_OK,
  STATE_NO_COMMS,
  STATE_LOW_BATT,
  STATE_LAST,
} state_e;

#define BLINK_SEQ_LEN 6
#define TICKS_PER_SEC  62500 //65536-16MHz/256/1Hz
const uint16_t blinks[STATE_LAST][BLINK_SEQ_LEN] = {
   {100, 1000, 100, 1000, 100, 1000},    //standby
   {1000, 1000, 1000, 1000, 1000, 1000}, //ok
   {1000, 100, 100, 100, 100, 100},     //no comms
   {100, 100, 100, 100, 100, 100} };    //low batt
uint8_t blink_index;

//Globals
ros::NodeHandle  nh;

state_e robo_state;
uint8_t rc_mode_en;

Servo right_servo;
Servo left_servo;

volatile uint16_t rc_throt;
volatile uint16_t rc_rot;

kingfisher_msgs::Sense sense;
ros::Publisher sense_pub("/sense", &sense); 

kingfisher_msgs::Drive drive;

//Command callback from ROS
void command_cb( const kingfisher_msgs::Drive& cmd_msg){
  //set servos
  drive = cmd_msg;
  if(!rc_mode_en) {
    right_servo.writeMicroseconds(drive.right*PULSE_DELTA+PULSE_NEUTRAL);
    left_servo.writeMicroseconds(drive.left*PULSE_DELTA+PULSE_NEUTRAL);
  }
}

ros::Subscriber<kingfisher_msgs::Drive> cmd_sub("/cmd_drive", command_cb);

//Calculate RC controller pulse lengths
void calcServoPulse() {
  static uint64_t rc_throt_start = 0;
  static uint64_t rc_rot_start = 0;
  
  if(digitalRead(RC_ENABLE_PIN)) {
    rc_mode_en = 1;
  } else {
    rc_mode_en = 0;
  }
  
  //Check for throtle change
  if(digitalRead(RC_THROT_PIN) == HIGH && rc_throt_start == 0) {
    rc_throt_start = micros();
  } else if(digitalRead(RC_THROT_PIN) == LOW && rc_throt_start != 0){
    rc_throt = (uint16_t)(micros() - rc_throt_start);
    rc_throt_start = 0;
  }
  
  //Check for rotation change
  if(digitalRead(RC_ROT_PIN) == HIGH && rc_rot_start == 0) {
    rc_rot_start = micros();
  } else if(digitalRead(RC_ROT_PIN) == LOW && rc_rot_start != 0){
    rc_rot = micros() - rc_rot_start;
    rc_rot_start = 0;
  }
  if(rc_mode_en) {
    //TODO: sign on rotation might be wrong double check
    int16_t rot = rc_rot - PULSE_NEUTRAL;
    uint16_t right_in = min(max(rc_throt+rc_rot*RC_ROTATION_WEIGHT, PULSE_NEUTRAL+PULSE_DELTA), PULSE_NEUTRAL-PULSE_DELTA);
    uint16_t left_in = min(max(rc_throt-rc_rot*RC_ROTATION_WEIGHT, PULSE_NEUTRAL+PULSE_DELTA), PULSE_NEUTRAL-PULSE_DELTA);
    right_servo.writeMicroseconds(right_in);
    left_servo.writeMicroseconds(left_in);
  }
}

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  TCNT1 = 65536 - 0.001*TICKS_PER_SEC*blinks[robo_state][blink_index]; // preload timer
  //TCNT1 = 65536 - 20*rc_throt;
  blink_index = (blink_index+1)%BLINK_SEQ_LEN;
  digitalWrite(RIGHT_LIGHT, blink_index & 0x1);
  digitalWrite(LEFT_LIGHT, blink_index & 0x1);
  
}

void read_battery() {
  float bat = 5.0*analogRead(BATTERY_SENSE)/1024; //volts
  if(bat < LOW_BAT_LEVEL) {
    robo_state = STATE_LOW_BATT;
  } else if (robo_state == STATE_LOW_BATT) {
    robo_state = STATE_OK; 
  }
  //publish to the sense
  sense.battery = bat;
  sense_pub.publish(&sense);
}

void setup(){
  noInterrupts();
  
  robo_state = STATE_STANDBY;

  //digital pin setup
  pinMode(RIGHT_LIGHT, OUTPUT);
  pinMode(LEFT_LIGHT, OUTPUT);
  pinMode(RC_ENABLE_PIN, OUTPUT); 
  digitalWrite(RC_ENABLE_PIN, LOW);
  analogReference(INTERNAL);
  pinMode(BATTERY_SENSE, INPUT);
  

  //blinking (setup timer)
  blink_index = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 65536 - 0.001*TICKS_PER_SEC*blinks[robo_state][blink_index]; // preload timer 65536-16MHz/256/2Hz
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
    
  //RC controller setup (pins 2/3 = D0/D1)
  attachInterrupt(0, calcServoPulse, CHANGE);  
  attachInterrupt(1, calcServoPulse, CHANGE);  
  rc_mode_en = 1;
  
  //Servo motor setup
  right_servo.attach(RIGHT_SERVO_PIN);
  left_servo.attach(LEFT_SERVO_PIN);
  
  rc_throt = PULSE_NEUTRAL;
  rc_rot = PULSE_NEUTRAL;
  
  //rosserial setup
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(sense_pub);
  
  interrupts();
}

void loop(){
  read_battery();
  //TODO: check comm connection (watchdog?)
  nh.spinOnce();
  delay(100); //throttle loop to ~10Hz (for testing)
}
