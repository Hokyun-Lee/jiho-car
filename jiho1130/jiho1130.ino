#include <Servo.h>
Servo FR, FL, BR, BL;

int pos = 0;

int FR_offset = 95;
int FL_offset = 91;
int BR_offset = 95;
int BL_offset = 86;

int FR_val, FL_val, BR_val, BL_val;

// Right Motor Pin Number
const int AHallSensor = 2;
const int BHallSensor = 3;
const int PWM_ctr = 5;
const int Dir_ctr = 24;
const int Enable = 22;

// Left Motor Pin Number
//const int AHallSensor = 2;
//const int BHallSensor = 3;
const int L_PWM_ctr = 6;
const int L_Dir_ctr = 34;
const int L_Enable = 32;

// Initial Condition
int prev_A_state = LOW; // Sensor A
const bool counterclockwise = false;
const bool clockwise = true;
bool rot_dir;
int pulse_cnt; // Encoder pulse: CCW++ CW--
unsigned long prev_time = -100; // When 0 -> NAN


// Feedback value
double prev_error;
double integ_error; // Integration of error

// Calculate variable
double cur_mot_vel;
double error;
int ctr_input;

// Setting
double des_mot_vel = 0; // Motor velocity in deg/s


void setup() {
  // put your setup code here, to run once:
  pinMode(AHallSensor, INPUT);
  pinMode(BHallSensor, INPUT);
  pinMode(PWM_ctr, OUTPUT);
  pinMode(L_PWM_ctr, OUTPUT);
  pinMode(Dir_ctr, OUTPUT);
  pinMode(L_Dir_ctr, OUTPUT);
  pinMode(Enable, OUTPUT);
  pinMode(L_Enable, OUTPUT);
  
  digitalWrite(Enable, HIGH); // Turn on the motor
  digitalWrite(L_Enable, HIGH); // Turn on the motor
  
  digitalWrite(Dir_ctr, HIGH); // CCW : LOW
  digitalWrite(L_Dir_ctr, HIGH); // CCW : LOW
  
  Serial.begin(9600);
  attachInterrupt(1, cnt_Encoder, CHANGE); // Interrupt pin number: 3

  pinMode(44, INPUT_PULLUP);
  FR.attach(8);
  FL.attach(9);
  BR.attach(10);
  BL.attach(11);
}

void loop() {
  // put your main code here, to run repeatedly:
  //for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //FR.write(pos);
    //FL.write(pos);
    //BR.write(pos);
    //BL.write(pos); // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position
  //}
  //for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    //FR.write(pos);
    //FL.write(pos);
    //BR.write(pos);
    //BL.write(pos);              // tell servo to go to position in variable 'pos'
    //delay(15);                       // waits 15ms for the servo to reach the position
  //}

  int ctr_input = computePID();
  analogWrite(PWM_ctr, ctr_input);
  analogWrite(L_PWM_ctr, ctr_input);
  //Serial.print(des_mot_vel); Serial.print(","); Serial.println(cur_mot_vel);

  int joy_x, joy_y;
  joy_x = analogRead(A0)-500;
  joy_y = 498-analogRead(A1);
  FR_val = FR_offset + joy_x/10;
  FL_val = FL_offset + joy_x/10;
  BR_val = BR_offset;
  BL_val = BL_offset;

  FR.write(FR_val);
  FL.write(FL_val);
  BR.write(BR_val);
  BL.write(BL_val);
  
  des_mot_vel = joy_y*2;

  //Serial.print("x:");
  //Serial.println(joy_x/50); // X축 값이 표기됩니다..
  //Serial.print("y:");
  //Serial.println(joy_y); // y축 값이 표기 됩니다.
  //Serial.println(digitalRead(8)); // Z축(스위치) 값이 표기됩니다.

  delay(100);
}

int computePID() {
  unsigned long cur_time = millis();
  unsigned long del_time = cur_time - prev_time; // Time inteval in seconds
  cur_mot_vel = (((double)pulse_cnt * 360 * 1000) / (26 * 19)) / del_time;

  //PID gains

  double kp = 0.05;
  double ki = 0.2;
  double kd = 0.002;
  /*
    // Change kp
    double kp = 0.23;
    double ki = 0.1;
    double kd = 0.001;
  */
  /*
    // Change kd
    double kp = 0.15;
    double ki = 0.1;
    double kd = 0.01;
  */
  /*
    // Change ki
    double kp = 0.15;
    double ki = 0.5;
    double kd = 0.001;
  */


  error = des_mot_vel - cur_mot_vel; // Compute error

  double out_p = kp * error; // Proportional
  integ_error += error * del_time / 1000; // Quadrature
  double out_i = ki * integ_error; // Integral
  double err_rate = (error - prev_error) / del_time * 1000;
  double out_d = kd * err_rate; // Differential
  int ctr_input = out_p + out_d + out_i;

  prev_error = error; // Feedback current error
  prev_time = cur_time; // Feedback current time
  pulse_cnt = 0; // Reset pulse count

  if (ctr_input >= 0) {
    digitalWrite(Dir_ctr, HIGH);
    digitalWrite(L_Dir_ctr, LOW);
    if (ctr_input > 255) {
      ctr_input = 255;
    }
  }

  else if (ctr_input < 0) {
    digitalWrite(Dir_ctr, LOW);
    digitalWrite(L_Dir_ctr, HIGH);
    if (ctr_input < -255) {
      ctr_input = -255;
    }
    ctr_input = ctr_input * (-1);
  }
  return ctr_input;
}

int cnt_Encoder() {
  int cur_A_state = digitalRead(AHallSensor);
  int cur_B_state = digitalRead(BHallSensor);
  if ((prev_A_state == LOW) && (cur_A_state == HIGH)) {
    if (cur_B_state == LOW) {// CW
      rot_dir = clockwise;
    }
    else if (cur_B_state == HIGH) {// CCW
      rot_dir = counterclockwise;
    }
  }
  else if ((prev_A_state == HIGH) && (cur_A_state == LOW)) {
    if (cur_B_state == LOW) {
      rot_dir = counterclockwise;
    }
    else if (cur_B_state == HIGH) {// CW
      rot_dir = clockwise;
    }
  }
  prev_A_state = cur_A_state; // Feedback current A state
  if (rot_dir) {
    pulse_cnt++;
  }
  else if (!rot_dir) {
    pulse_cnt--;
  }
  return pulse_cnt;
}
