#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#define PIN_MOTOR_DIR_L 4 // направление левого мотора
#define PIN_MOTOR_DIR_R 7 // направление правого мотора
#define PIN_MOTOR_SPD_L 5 // скорость левого мотора
#define PIN_MOTOR_SPD_R 6 // скорость правого мотора
#define PIN_LINE_L A0 // левый датчик линии
#define PIN_LINE_R A1 // правый датчик линии
const int base_vel = 80; // Базовая скорость

ros::NodeHandle  nh;

std_msgs::Int16 lline_data;
ros::Publisher line_left_pub("/frob/sensors/left_line_sensor", &lline_data);

std_msgs::Int16 rline_data;
ros::Publisher line_right_pub("/frob/sensors/right_line_sensor", &lline_data);

long long dataTimer = millis();
void move_handler(const geometry_msgs::Twist & msg) {
  float x = msg.linear.x;
  float z_rot = msg.angular.z;
  
  float r_cmd = (-z_rot*1.8)/2.0 + x;
  float l_cmd = 2.0*x - r_cmd;
  
  digitalWrite(PIN_MOTOR_DIR_R, r_cmd > 0); // Установка направлений колес
  digitalWrite(PIN_MOTOR_DIR_L, l_cmd > 0);

  int l_speed = abs(int(base_vel * l_cmd));
  int r_speed = abs(int(base_vel * r_cmd));

  analogWrite(PIN_MOTOR_SPD_R, r_speed);
  analogWrite(PIN_MOTOR_SPD_L,  l_speed);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", move_handler);

void setup() {
  pinMode(PIN_MOTOR_SPD_R, OUTPUT);
  pinMode(PIN_MOTOR_DIR_R, OUTPUT);
  pinMode(PIN_MOTOR_SPD_L, OUTPUT);
  pinMode(PIN_MOTOR_DIR_L, OUTPUT);
  pinMode(PIN_LINE_L, INPUT);
  pinMode(PIN_LINE_R, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(line_left_pub);
  nh.advertise(line_right_pub);
}

void loop() {
  if (millis() - dataTimer > 300) {
    lline_data.data = analogRead(PIN_LINE_L);
    rline_data.data = analogRead(PIN_LINE_R);
    line_left_pub.publish(&lline_data);
    line_right_pub.publish(&rline_data);

    dataTimer = millis();
  }
  nh.spinOnce();
  delay(1);
}
