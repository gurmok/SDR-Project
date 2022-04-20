int L_sensor = 2;
int R_sensor = 3;
int L_count = 0;
int R_count = 0;
int L_val;
int R_val;

void setup() {
  Serial.begin(9600);
  pinMode(L_sensor, INPUT);
  pinMode(R_sensor, INPUT);
  // encoder count
  attachInterrupt(digitalPinToInterrupt(2), L_objectDetection, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), R_objectDetection, HIGH);
}

void loop() {
  L_val = digitalRead(L_sensor);
  R_val = digitalRead(R_sensor);
  Serial.print("Encoder State");
  Serial.print(L_val);
  Serial.print(R_val);
  Serial.println();
  Serial.print("L_Count");
  Serial.println(L_count);
  Serial.print("R_Count");
  Serial.println(R_count);
  delay(1000);
}

void L_objectDetection(){
  if(L_val == LOW){
    L_count++;
  };
 }

void R_objectDetection(){
  if(R_val == LOW){
    R_count++;
  };
 }
