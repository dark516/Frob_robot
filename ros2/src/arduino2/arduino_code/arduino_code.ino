#include "motor_regulator.h"
#include "ros2_communication.hpp"

// Форвард-декларации
class Regulator;
extern Regulator left_regulator;
extern Regulator right_regulator;

// В файле arduino_code.ino

Motor left_motor(7, 6);
Motor right_motor(4, 5);

// Создаем энкодеры с корректными пинами и настройками
Encoder left_enc(2, 13, []{ left_regulator.encoder.encoder_int(); }, false);   // INT0 (pin 2), B=11, invert=true
Encoder right_enc(3, 12, []{ right_regulator.encoder.encoder_int(); }, true); // INT1 (pin 3), B=12, invert=false

// Остальной код остается без изменений

// Создаем PID-регуляторы
PID left_pid(4.0, 0.2, 0.04, 100);
PID right_pid(4.0, 0.2, 0.04, 100);

// Создаем регуляторы, передавая созданные объекты
Regulator left_regulator(left_motor, left_enc, left_pid);
Regulator right_regulator(right_motor, right_enc, right_pid);

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  set_velocity(0.0, 0.0);
}

void loop() {
  static uint32_t t = millis();
  
  command_spin();
  
  if(millis() - t >= 10) { // DT = 0.01 сек
    t = millis();
    left_regulator.update();
    right_regulator.update();
  }
  Serial.print(left_enc.ticks);
  Serial.print("   ");
  Serial.println(right_enc.ticks);
}
