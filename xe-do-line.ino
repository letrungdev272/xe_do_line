//ĐỒ ÁN ĐIỀU KHIỂN TỰ ĐỘNG//
///////XE DÒ LINE///////

//KHAI BÁO CẢM BIẾN
#define ss0 A5
#define ss1 A4
#define ss2 A3
#define ss3 A2
#define ss4 A1
//KHAI BÁO CẦU H
#define inA1 4
#define inA2 5
#define inB1 7
#define inB2 8
#define EnA 3  //CHÂN BĂM XUNG
#define EnB 9  //CHÂN BĂM XUNG
#define stby 6

float error, previous_error = 0;
float Kp = 15, Kd = 0.04, Ki = 0.01;  // Thay đổi thông số cho phù hợp
float P, I, D, PID_value;
float SamplingTime = 0.1;
int a[5];
int initial_motor_speed = 100;  //100

void setup() {
  digitalWrite(stby, 1);  // CHÂN STANBY LÊN 1 ĐỂ CẦU H HOẠT ĐỘNG
  pinMode(ss0, INPUT);
  pinMode(ss1, INPUT);
  pinMode(ss2, INPUT);
  pinMode(ss3, INPUT);
  pinMode(ss4, INPUT);
  pinMode(stby, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  Serial.begin(9600);
}
void loop() {
  // read_sensor_values();
  // calculate_pid();
  // motor_control();
  detect_line_with_2_sensor();
}

void detect_line_with_2_sensor() {
  a[0] = !digitalRead(ss0);  // 1: line đen 0: line trắng
  a[1] = !digitalRead(ss1);
  a[2] = !digitalRead(ss2);
  a[3] = !digitalRead(ss3);
  a[4] = !digitalRead(ss4);

  if (a[1] == 1) {
    digitalWrite(inA1, 0);
    digitalWrite(inA2, 0);
    digitalWrite(inB1, 0);
    digitalWrite(inB2, 1);
    analogWrite(EnB, 255);
    analogWrite(EnA, 0);
  } else if (a[3] == 1) {
    digitalWrite(inA1, 0);
    digitalWrite(inA2, 1);
    digitalWrite(inB1, 0);
    digitalWrite(inB2, 0);
    analogWrite(EnB, 0);
    analogWrite(EnA, 255);
  } else if (a[3] == 0 && a[1] == 0) {
    digitalWrite(inA1, 0);
    digitalWrite(inA2, 1);
    digitalWrite(inB1, 0);
    digitalWrite(inB2, 1);
    analogWrite(EnB, 255);
    analogWrite(EnA, 255);
  }
}

void read_sensor_values()  //ĐỌC CẢM BIẾN
{
  a[0] = !digitalRead(ss0);  // 1: line đen 0: line trắng
  a[1] = !digitalRead(ss1);
  a[2] = !digitalRead(ss2);
  a[3] = !digitalRead(ss3);
  a[4] = !digitalRead(ss4);

  if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0) && (a[3] == 0) && (a[4] == 1)) error = 4;        //00001
  else if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0) && (a[3] == 1) && (a[4] == 1)) error = 3;   //00011
  else if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0) && (a[3] == 1) && (a[4] == 0)) error = 2;   //00010
  else if ((a[0] == 0) && (a[1] == 0) && (a[2] == 1) && (a[3] == 1) && (a[4] == 0)) error = 1;   //00110
  else if ((a[0] == 0) && (a[1] == 0) && (a[2] == 1) && (a[3] == 0) && (a[4] == 0)) error = 0;   //00100
  else if ((a[0] == 0) && (a[1] == 1) && (a[2] == 1) && (a[3] == 0) && (a[4] == 0)) error = -1;  //01100
  else if ((a[0] == 0) && (a[1] == 1) && (a[2] == 0) && (a[3] == 0) && (a[4] == 0)) error = -2;  //01000
  else if ((a[0] == 1) && (a[1] == 1) && (a[2] == 0) && (a[3] == 0) && (a[4] == 0)) error = -3;  //11000
  else if ((a[0] == 1) && (a[1] == 0) && (a[2] == 0) && (a[3] == 0) && (a[4] == 0)) error = -4;  //10000
  else if ((a[0] == 0) && (a[1] == 0) && (a[2] == 0) && (a[3] == 0) && (a[4] == 0)) error = 0;   //00000
  else error = 10;
}
//TÍNH TOÁN PID
void calculate_pid() {
  error = 0 - error;
  P = error * Kp;
  I += Ki * error * SamplingTime;
  D = (Kd * (error - previous_error)) / SamplingTime;
  PID_value = P + I + D;
  previous_error = error;
}

void motor_control()  // Tính toán tốc độ động cơ
{
  float left_motor_speed = initial_motor_speed + PID_value;   // dau -
  float right_motor_speed = initial_motor_speed - PID_value;  // dau +

  if (left_motor_speed > 255)
    left_motor_speed = 255;
  else if (left_motor_speed < 0)
    left_motor_speed = 0;

  if (right_motor_speed > 255)
    right_motor_speed = 255;
  else if (right_motor_speed < 0)
    right_motor_speed = 0;

  // Serial.print("PID= ");
  // Serial.print(PID_value);
  // Serial.print(" trai= ");
  // Serial.print(left_motor_speed);
  // Serial.print(" phai= ");
  // Serial.println(right_motor_speed);
  for (int i = 0; i < 5; i++) {
    if (i != 4) {
      Serial.print(a[i]);
      Serial.print(" ");
    } else {
      Serial.println(a[i]);
    }
  }
  analogWrite(EnB, left_motor_speed);
  analogWrite(EnA, right_motor_speed);
  digitalWrite(inA1, 0);  // đảo chiều quay
  digitalWrite(inA2, 1);
  digitalWrite(inB1, 0);
  digitalWrite(inB2, 1);
}
