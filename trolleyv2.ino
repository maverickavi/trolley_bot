int dir1PinA = 5;
int dir2PinA = 4;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed
int dir1PinB = 3;
int dir2PinB = 2;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed
long sensor[]={0,0,0,0,0};
int flag = 0;
float proportional = 0.00;
float integral = 0.00;
float derivative = 0.00;
float last_proportional = 0.00;
int error_value = 0;
int scaled_error = 0;
float Ki = 0.00;
float Kp = 0.27;
float Kd = 0.00;
int right_speed, left_speed;
int max_speed = 200;
int flagS = 0;
const int dangerDist = 10;

const int trig = 12;
const int echo = A0;
long duration;

int dist_f, new_dist, d1 =0, d2=0, d3=0, d4=0, d5=0;

void setup() {  
      Serial.begin(9600);
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);
      pinMode(dir1PinA,OUTPUT);
      pinMode(dir2PinA,OUTPUT);
      pinMode(speedPinA,OUTPUT);
      pinMode(dir1PinB,OUTPUT);
      pinMode(dir2PinB,OUTPUT);
      pinMode(speedPinB,OUTPUT);
      pinMode(6,INPUT);
      pinMode(7,INPUT);
      pinMode(8,INPUT);
      pinMode(11,INPUT);
      pinMode(13,INPUT);
      pinMode(A5, INPUT);
}
int average(int d1, int d2, int d3, int d4, int dist_f){
  return ((d1+d2+d3+d4+dist_f)/5);
}

long ping(){

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  
   duration = pulseIn(echo, HIGH);
   return (duration/29/2);
}


int pid_calc(float rpos){
  proportional = rpos - 3000;
  if(proportional > 600) integral = 0;
  else integral = integral + proportional;
  derivative = proportional - last_proportional;
  last_proportional = proportional;
  error_value = int(proportional*Kp + integral*Ki + derivative*Kd);
  scaled_error = 1*error_value;
  return scaled_error;
}


void calc_turn(int error){
  if(error < (-1*max_speed)) error = (-1*max_speed);
  if(error > max_speed) error = max_speed;
  if(error<0){
    right_speed = max_speed + error;
    left_speed = max_speed;    
  }
  else{
    right_speed = max_speed;
    left_speed = max_speed - error;
  }
}


void motorDriveF(int rspd, int lspd){
  analogWrite(speedPinB, rspd);
  digitalWrite(dir1PinB, 1);
  digitalWrite(dir2PinB, 0);
  analogWrite(speedPinA, lspd);
  digitalWrite(dir1PinA, 1);
  digitalWrite(dir2PinA, 0);
  delay(50);    
}


void motorDriveS(){
  digitalWrite(dir1PinB, 0);
  digitalWrite(dir2PinB, 0);
  digitalWrite(dir1PinA, 0);
  digitalWrite(dir2PinA, 0);    
}


void loop() {
// Initialize the Serial interface:
    long sensors_weight = 0;
    long sensors_sum = 0;
    if (Serial.available() > 0) {
      int inByte = Serial.read();
      int speed; // Local variable
      
      if(inByte == 'F') flag = 1;
      else if(inByte == 'S') flag = 0;
      else if(inByte == 'P') Kp = Kp + 0.01;
      else if(inByte == 'Q') Kp = Kp + 0.10;
      else if(inByte == 'R') Kp = Kp + 1.00;
      else if(inByte == 'p') Kp = Kp - 0.01;
      else if(inByte == 'q') Kp = Kp - 0.10;
      else if(inByte == 'r') Kp = Kp - 1.00; 
      else if(inByte == 'D') Kd = Kd + 0.01;
      else if(inByte == 'E') Kd = Kd + 0.10;
      else if(inByte == 'F') Kd = Kd + 1.00;
      else if(inByte == 'd') Kd = Kd - 0.01;
      else if(inByte == 'e') Kd = Kd - 0.10;
      else if(inByte == 'f') Kd = Kd - 1.00; 
      else if(inByte == 'I') Ki = Ki + 0.01;
      else if(inByte == 'J') Ki = Ki + 0.10;
      else if(inByte == 'K') Ki = Ki + 1.00;
      else if(inByte == 'i') Ki = Ki - 0.01;
      else if(inByte == 'j') Ki = Ki - 0.10;
      else if(inByte == 'k') Ki = Ki - 1.00;
      else if(inByte == '+') max_speed = max_speed + 5;
      else if(inByte == '-') max_speed = max_speed - 5;
    }
     dist_f = ping();
      new_dist = average(d1, d2, d3, d4, dist_f);
      if(new_dist <= dangerDist){       //danger stop
        flagS = 1;
      }
      else if(new_dist>dangerDist && new_dist<500 && (d4-new_dist)<100){     //clear move
        flagS=0;
    }
    
    
    
      d1=dist_f;
      d2=d1;
      d3=d2;
      d4=d3; 
    sensor[0] = digitalRead(6);
    sensor[1] = digitalRead(7);
    sensor[2] = digitalRead(8);
    sensor[3] = digitalRead(11);
    sensor[4] = digitalRead(13);
    
    for(int j=0; j<5; j++){
      sensors_weight += (j+1)*1000*sensor[j];
      sensors_sum += sensor[j];
    }
    float pos = sensors_weight/sensors_sum;
    int error = pid_calc(pos);
    calc_turn(error);
    if(flag == 1) {
      if(flagS==1||sensors_sum==5||sensors_sum==0)motorDriveS();
      if(flagS==0&&sensors_sum!=5) motorDriveF(right_speed, left_speed);
      if(digitalRead(A5)==1&&flagS==0) motorDriveF(max_speed, max_speed);
    }
    if(flag == 0) {
      if(digitalRead(A5)==0||flagS==1||sensors_sum==5||sensors_sum==0) motorDriveS();
      if(digitalRead(A5)==1&&flagS==0) motorDriveF(max_speed, max_speed);
    }
    
//    Serial.print(digitalRead(A5));
//    Serial.print("  ");
//    Serial.print(sensors_sum);
//    Serial.print("  ");
//    Serial.print(flagS);
//    Serial.print("  ");
//    Serial.print(flag);
//    Serial.print("  ");
    Serial.print(sensor[0]);
    Serial.print("  ");
    Serial.print(sensor[1]);
    Serial.print("  "); 
    Serial.print(sensor[2]);
    Serial.print("  "); 
    Serial.print(sensor[3]);
    Serial.print("  "); 
    Serial.print(sensor[4]);
    Serial.print("  "); 
    Serial.print(pos);
    Serial.print("  ");  
    Serial.print(Kp);
    Serial.print("  ");
    Serial.print(Kd);
    Serial.print("  ");
    Serial.print(Ki);
    Serial.print("  ");
    Serial.print(right_speed);
    Serial.print("  ");
    Serial.println(left_speed);
    
    
        
}

