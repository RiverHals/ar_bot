#include <IRremote.hpp>

//signals
#define IR_RECEIVE_PIN 0
#define IR_BUTTON_PLUS 21 // +, strait
#define IR_BUTTON_MINUS 7 // -, back
#define IR_BUTTON_CH_PLUS 71 // CH+, loop_right
#define IR_BUTTON_CH_MINUS 69 // CH-, loop_back
#define IR_BUTTON_PLAY_PAUSE 67 // >||, pause
#define TURN_RIGHT 64 // >>, turn right
#define TURN_LEFT 68 // <<, turn left
#define SPEED_INCREASE 13 // 200+, increase speed
#define SPEED_DECREASE 22 // 0, decrease speed
#define STATE_DATA 9 // EQ, state data

// states
#define PAUSE 0
#define STRAIT 1
#define BACK 2
#define LOOP_RIGHT 3
#define LOOP_LEFT 4

#define SPEED_1      5 
#define DIR_1        4
 
#define SPEED_2      6
#define DIR_2        7

#define TURN_DELAY 1000
#define SPEED_DELTA int(64)

int speed = 63;
int state = 0;

void go_strait(){
  digitalWrite(DIR_1, LOW); // set direction
  analogWrite(SPEED_1, speed); // set speed
  digitalWrite(DIR_2, LOW); // set direction
  analogWrite(SPEED_2, speed);
  state = STRAIT;

  Serial.println("gone_strait");

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
    case STRAIT: {
      Serial.println("STRAIT");
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
  go_strait();
  delay(TURN_DELAY);
  loop_right();
  delay(TURN_DELAY);
  go_strait();

  Serial.println("turned_right");

}

void turn_left(){
  go_strait();
  delay(TURN_DELAY);
  loop_left();
  delay(TURN_DELAY);
  go_strait();

  Serial.println("turned_left");

}

void keep_going(){
  switch(state){
    case STRAIT: {
      go_strait();
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

void execute(int command){
  if(command == IR_BUTTON_PLAY_PAUSE){
    pause();
  }
  switch (command) {
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
          go_strait();
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