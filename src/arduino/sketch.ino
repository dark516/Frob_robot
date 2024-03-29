#include <ros.h>
#include <std_msgs/Float64.h>
//#include <sensor_msgs/Range.h>

//sensor_msgs::Range range_msg;
//ros::Publisher pub_range( "/ultrasound", &range_msg);
ros::NodeHandle  nh;

#define PIN_MOTOR_DIR_L 4 // направление левого мотора
#define PIN_MOTOR_DIR_R 7 // направление правого мотора
#define PIN_MOTOR_SPD_L 5 // скорость левого мотора
#define PIN_MOTOR_SPD_R 6 // скорость правого мотора
void setLeftMotor(const std_msgs::Float64& msg){
  digitalWrite(PIN_MOTOR_DIR_L, msg.data < 0);
  analogWrite(PIN_MOTOR_SPD_L, map(abs(msg.data), 0, 10.23, 0, 255));
}

void setRightMotor(const std_msgs::Float64& msg){
  digitalWrite(PIN_MOTOR_DIR_R, msg.data < 0);
  analogWrite(PIN_MOTOR_SPD_R, map(abs(msg.data), 0, 10.23, 0, 255));
}

ros::Subscriber<std_msgs::Float64> sub("/abot/left_wheel/pwm", setLeftMotor);
ros::Subscriber<std_msgs::Float64> sub1("/abot/right_wheel/pwm", setRightMotor);


void setup() {
  pinMode(PIN_MOTOR_DIR_L, OUTPUT);
  pinMode(PIN_MOTOR_DIR_R, OUTPUT);
  pinMode(PIN_MOTOR_SPD_L, OUTPUT);
  pinMode(PIN_MOTOR_SPD_R, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub1);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
