
/*
* ros_navigation 
* author yoon

Reference 
https://atadiat.com/en/e-rosserial-arduino-introduction/
*/


#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define EN_L 11
#define IN1_L 13
#define IN2_L 12
#define EN_R 10
#define IN1_R 9
#define IN2_R 8

#define ENC_IN_LEFT 2
#define ENC_IN_RIGHT 3

boolean Direction_left = true;
boolean Direction_right = true;

// 500ms interval for measurements
const int interval = 500;
long previousMillis = 0;
long currentMillis = 0;

double w_r = 0, w_l = 0;
// 바퀴의 반지름, 바퀴 사이의 거리
double wheel_rad = 0.033, wheel_sep = 0.125;
double speed_ang=0, speed_lin=0;

void MotorControl(const geometry_msgs::Twist& msg){
    speed_ang = msg.angular.z;
    speed_lin = msg.linear.x;

    w_r = (speed_lin/wheel_rad) + (7*(speed_ang*wheel_sep)/(2.0*wheel_rad));
    w_l = (speed_lin/wheel_rad) - (7*(speed_ang*wheel_sep)/(2.0*wheel_rad));

    MotorL(w_l*10);
    MotorR(w_r*10);
}

// 노드핸들러 객체 생성
ros::NodeHandle nh;
// cmd_vel 이라는 토픽을 subscribe할 sub 객체 생성
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &MotorControl);
std_msgs::Float64 Left_encoder_msg;
std_msgs::Float64 Right_encoder_msg;
ros::Publisher L_pub_encoder("L_encoder_pulse", &Left_encoder_msg);
ros::Publisher R_pub_encoder("R_encoder_pulse", &Right_encoder_msg);

void Encoder_R_count() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(IN1_R);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
      Right_encoder_msg.data++;      
  }

  else {
      Right_encoder_msg.data--;     
  }
}
 
void Encoder_L_count() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(IN1_L);
 
  if(val == LOW) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
      Left_encoder_msg.data++;    
  }

  else {
      Left_encoder_msg.data--;     
  }
}



// 모터 초기화 및 노드 초기화
void setup(){
    Motors_init();
    pinMode(ENC_IN_LEFT, INPUT);
    pinMode(ENC_IN_RIGHT, INPUT);

    attachInterrupt(digitalPinToInterrupt(2), Encoder_L_count, RISING);
    attachInterrupt(digitalPinToInterrupt(3), Encoder_R_count, RISING);

    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(L_pub_encoder);
    nh.advertise(R_pub_encoder);
}

void loop(){
    currentMillis = millis();

    if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
     
    R_pub_encoder.publish(&Right_encoder_msg);
    L_pub_encoder.publish(&Left_encoder_msg);
    Right_encoder_msg.data = 0;
    Left_encoder_msg.data = 0;
    }

    nh.spinOnce();    
}

void Motors_init(){
    pinMode(EN_L, OUTPUT);
    pinMode(EN_R, OUTPUT);
    pinMode(IN1_L, OUTPUT);
    pinMode(IN2_L, OUTPUT);
    pinMode(IN1_R, OUTPUT);
    pinMode(IN2_R, OUTPUT);
    digitalWrite(EN_L, LOW);
    digitalWrite(EN_R, LOW);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
}

void MotorL(int Pulse_Width1){
    if (Pulse_Width1 > 0){
        analogWrite(EN_L, Pulse_Width1);
        digitalWrite(IN1_L, HIGH);
        digitalWrite(IN2_L, LOW);
    }
    if (Pulse_Width1 < 0){
        Pulse_Width1=abs(Pulse_Width1);
        analogWrite(EN_L, Pulse_Width1);
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, HIGH);
    }
    if (Pulse_Width1 == 0){
        analogWrite(EN_L, Pulse_Width1);
        digitalWrite(IN1_L, LOW);
        digitalWrite(IN2_L, LOW);
    }
}

void MotorR(int Pulse_Width2){
    if (Pulse_Width2 > 0){
        analogWrite(EN_R, Pulse_Width2);
        digitalWrite(IN1_R, HIGH);
        digitalWrite(IN2_R, LOW);
    }
    if (Pulse_Width2 < 0){
        Pulse_Width2=abs(Pulse_Width2);
        analogWrite(EN_R, Pulse_Width2);
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, HIGH);
    }
    if (Pulse_Width2 == 0){
        analogWrite(EN_R, Pulse_Width2);
        digitalWrite(IN1_R, LOW);
        digitalWrite(IN2_R, LOW);
    } 
} 
