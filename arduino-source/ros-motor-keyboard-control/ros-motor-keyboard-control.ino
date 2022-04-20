/*
* rosserial dc motor control 
* author yoon
*/

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define EN_L 11
#define IN1_L 13
#define IN2_L 12
#define EN_R 10
#define IN1_R 9
#define IN2_R 8

double w_r=0, w_l=0;

// 바퀴의 반지름, 바퀴 사이의 거리
double wheel_rad = 0.033, wheel_sep = 0.125;

// 노드핸들러 객체 생성
ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;

// msg에 따라 바퀴의 속도를 조절하는 콜백함수 정의
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  // 오른쪽 바퀴
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  // 왼쪽 바퀴
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
// cmd_vel 이라는 토픽을 subscribe할 sub 객체 생성
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

// 모터 초기화 함수 선언
void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);

// 모터 초기화 및 노드 초기화
void setup(){
 Motors_init();
 nh.initNode();
 nh.subscribe(sub);
}
// 메인루프
void loop(){
 MotorL(w_l*10);
 MotorR(w_r*10);
 // 콜백함수 호출
 nh.spinOnce();
}
/*
ros에서는 msg가 토픽으로 수신되면 그것을 큐(queue)에 쌓음
그 후 subscribe쪽에서 spin() 메소드를 이용해서 큐에 있는 메세지를 처리하도록 구성되어있음
spin, spinOnce 모두 Subscribe 할 때 해당 callBack함수를 실행시키는 역할을 함

1) ros::spin()은 콜백만 처리하다 죽는다. (Blocking)
2) ros::spinOnce()는 한번 콜백을 처리하고 넘어간다. (Non-blocking)
3) rate.sleep()은 무한루프에서 설정한 주기를 맞추기 위해 기다리는 함수이다.
*/

// 모터 초기화 함수 정의
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
