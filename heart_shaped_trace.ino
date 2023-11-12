#include <IRremote.hpp>
#include <math.h>

//signals
#define IR_RECEIVE_PIN 0
#define IR_BUTTON_PLUS 21 // +, go_straight
#define IR_BUTTON_MINUS 7 // -, go_back
#define IR_BUTTON_CH_PLUS 71 // CH+, loop_right
#define IR_BUTTON_CH_MINUS 69 // CH-, loop_back
#define IR_BUTTON_PLAY_PAUSE 67 // >||, pause
#define TURN_RIGHT 64 // >>, turn_right
#define TURN_LEFT 68 // <<, turn_left
#define SPEED_INCREASE 13 // 200+, speed_increase 
#define SPEED_DECREASE 22 // 0, speed_decrease
#define STATE_DATA 9 // EQ, state_data
#define HEART_TRACE 8 // change to valid value, heart_trace

// states
#define PAUSE 0
#define STRAIGHT 1
#define BACK 2
#define LOOP_RIGHT 3
#define LOOP_LEFT 4

#define SPEED_1      5 
#define DIR_1        4
 
#define SPEED_2      6
#define DIR_2        7

#define TURN_DELAY 1000
#define SPEED_DELTA int(64)

struct iter_param { // data for one iteration
  bool dir_loop; // rigth = 1, left = 0
  double loop_time;
  double straight_time;
};

struct coordinates { 
  double x;
  double y;
};

struct direction {
  double dist;
  double curr_tg;
  double prev_tg;
};

int speed = 255;
int state = 0;

// heart shaped trace data
double diam = 1; // diameter of partition (for t)
double one_loop_time = 1700*255/speed; // for speed 
double iter_straight_time = 2000; // for straight trace

void go_straight(){
  digitalWrite(DIR_1, LOW); // set direction
  analogWrite(SPEED_1, speed); // set speed
  digitalWrite(DIR_2, LOW); // set direction
  analogWrite(SPEED_2, speed);
  state = STRAIGHT;

  Serial.println("gone_straight");
}

void go_back(){
  digitalWrite(DIR_1, HIGH); // set direction
  analogWrite(SPEED_1, speed); // set speed
  digitalWrite(DIR_2, HIGH); // set direction
  analogWrite(SPEED_2, speed); // set speed
  state = BACK;

  Serial.println("gone_back");
}

void loop_right(){          
  digitalWrite(DIR_1, HIGH); // set direction
  analogWrite(SPEED_1, speed); // set speed
  digitalWrite(DIR_2, LOW); // set direction
  analogWrite(SPEED_2, speed); // set speed
  state = LOOP_RIGHT;

  Serial.println("looped_right");
}

void loop_left(){          
  digitalWrite(DIR_1, LOW); // set direction
  analogWrite(SPEED_1, speed); // set speed
  digitalWrite(DIR_2, HIGH); // set direction
  analogWrite(SPEED_2, speed); // set speed
  state = LOOP_LEFT;

  Serial.println("looped_left");
}

void pause(){
  analogWrite(SPEED_1, 0); 
  analogWrite(SPEED_2, 0); 
  state = PAUSE; 

  Serial.println("paused");
}

void state_data(int state){
  switch(state){
    case PAUSE: {
      Serial.println("PAUSE");
      break;
    }
    case STRAIGHT: {
      Serial.println("STRAIGHT");
      break;
    }
    case BACK: {
      Serial.println("BACK");
      break;
    }
    case LOOP_RIGHT: {
      Serial.println("LOOP_RIGHT");
      break;
    }
    case LOOP_LEFT: {
      Serial.println("LOOP_LEFT");
      break;
    }
  }
}

void turn_right(){
  go_straight();
  delay(TURN_DELAY);
  loop_right();
  delay(TURN_DELAY);
  go_straight();

  Serial.println("turned_right");
}

void turn_left(){
  go_straight();
  delay(TURN_DELAY);
  loop_left();
  delay(TURN_DELAY);
  go_straight();

  Serial.println("turned_left");
}

void keep_going(){
  switch(state){
    case STRAIGHT: {
      go_straight();
      break;
    }
    case BACK: {
      go_back();
      break;
    }
    case LOOP_RIGHT: {
      loop_right();
      break;
    }
    case LOOP_LEFT: {
      loop_left();
      break;
    }
  }
}

void speed_increase(){
  int check = speed + SPEED_DELTA;
  if(check < 255){
    speed += SPEED_DELTA;
  }
  else{
    speed = 255;
  }
  Serial.println(check);
  Serial.println(speed);
  keep_going();
}

void speed_decrease(){
  int check = speed - SPEED_DELTA;
  if(check < 0){
    speed = 0;
  }
  else{
    speed -= SPEED_DELTA;
  }
  Serial.println(check);
  Serial.println(speed);
  keep_going();
}

// chosen equation for heart
// x = -sqrt(2)*pow(sin(t), 3)
// y = -pow(cos(t), 3)-pow(cos(t), 2)+2*cos(t)
// t is from [-pi, pi]
double x_pos(double i){
  return -sqrt(2)*pow(sin(i), 3);
}
double y_pos(int i){
  return -pow(cos(i), 3)-pow(cos(i), 2)+2*cos(i);
}

struct direction calc_iter_prak_lenght(double i, struct coordinates pos){
  struct direction iter_dir;
  struct coordinates prev_pos;
  prev_pos.x = x_pos(i-diam);
  prev_pos.y = y_pos(i-diam);
  double delta_x = pos.x - prev_pos.x;
  double delta_y = pos.y - prev_pos.y;
  iter_dir.dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
  iter_dir.curr_tg = delta_y/delta_x;
  struct coordinates prev_prev_pos;
  if((i-2*diam)<=-M_PI){
    iter_dir.prev_tg = 0;
  } else {
    prev_prev_pos.x = x_pos(i-2*diam);
    prev_prev_pos.y = y_pos(i-2*diam);
    double prev_delta_x = prev_pos.x - prev_prev_pos.x;
    double prev_delta_y = prev_pos.y - prev_prev_pos.y;
    iter_dir.prev_tg = delta_y/delta_x;
  }
  return iter_dir;
}

bool get_iter_dir_loop(struct direction iter_dir){
  if((iter_dir.curr_tg - iter_dir.prev_tg) > 0){
    return true;
  } else {
    return false;
  }
}

double get_iter_loop_time(struct direction iter_dir){
  return (atan(iter_dir.curr_tg) - atan(iter_dir.prev_tg))*one_loop_time;
}

struct iter_param get_iter_param(double i){
  struct coordinates pos;
  pos.x = x_pos(i);
  pos.y = y_pos(i);
  struct direction iter_dir = calc_iter_prak_lenght(i, pos);
  struct iter_param iter;
  iter.dir_loop = get_iter_dir_loop(iter_dir);
  iter.loop_time = get_iter_loop_time(iter_dir);
  iter.straight_time = iter_straight_time*iter_dir.dist*255/speed; // 255 - max possible speed
  return iter;
}

void heart_trace(){
  for(double i=-M_PI+diam; i<=M_PI; i+diam){
    struct iter_param iter = get_iter_param(i);
    if(iter.dir_loop){
      turn_right();
      delay(iter.loop_time);
    }else{
      turn_left();
      delay(iter.loop_time);
    }
    go_straight();
    delay(iter.straight_time);
  }
  pause();
}

void execute(int command){
  switch (command) {
    case HEART_TRACE: {
      heart_trace();
      break;
    }
    case SPEED_INCREASE: {
      speed_increase();
      break;
    }
    case SPEED_DECREASE: {
      speed_decrease();
      break;
    }
    case STATE_DATA: {
      state_data(state);
      break;
    }    
    case TURN_RIGHT: {
      turn_right();
      break;
    }
    case TURN_LEFT: {
      turn_left();
      break;
    }
    case IR_BUTTON_PLUS: {
      go_straight();
      break;
    }
    case IR_BUTTON_MINUS: {
      go_back();
      break;
    }
    case IR_BUTTON_CH_PLUS: { 
      loop_right();
      break;
        }
    case IR_BUTTON_CH_MINUS: { 
      loop_left();
      break;
    }
    case IR_BUTTON_PLAY_PAUSE: { 
      pause();
      break;
    }
  }
}

void setup(){
  Serial.begin(9600);
  IrReceiver.begin(IR_RECEIVE_PIN);

  for (int i = 4; i < 8; i++) {     
    pinMode(i, OUTPUT);
  }
}

void loop(){
   if (IrReceiver.decode()) {
      IrReceiver.resume(); // Enable receiving of the next value
      int command = IrReceiver.decodedIRData.command;
      Serial.println(command);

      execute(command);
  }
}
