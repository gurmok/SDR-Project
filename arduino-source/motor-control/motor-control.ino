
#define EN_L 11
#define IN1_L 13
#define IN2_L 12
#define EN_R 10
#define IN1_R 9
#define IN2_R 8

/*
 * ENA    IN1   IN2
 * HIGH   HIGH  LOW   모터 A 정회전(앞으로)
 * HIGH   LOW   HIGH  모터 A 역회전(뒤로)
 * HIGH   HIGH  HIGH  모터 A 급정지(브레이크)
 * HIGH   LOW   LOW   모터 A 급정지(브레이크)
 * LOW    X     X     모터 A 천천히정지(액셀 발뗌)
 */
void setup() {

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

void loop() {
  //전진
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  analogWrite(EN_L, 255);
  analogWrite(EN_R, 255);
  delay(2000);
  //후진
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  analogWrite(EN_L, 255);
  analogWrite(EN_R, 255);
  delay(2000);
  //좌회전
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, HIGH);
  digitalWrite(IN1_R, HIGH);
  digitalWrite(IN2_R, LOW);
  analogWrite(EN_L, 255);
  analogWrite(EN_R, 255);
  delay(2000);
  //우회전
  digitalWrite(IN1_L, HIGH);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, HIGH);
  analogWrite(EN_L, 255);
  analogWrite(EN_R, 255);
  delay(2000);
  

}
