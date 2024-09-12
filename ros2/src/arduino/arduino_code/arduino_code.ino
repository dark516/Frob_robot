#include "motor_regulator.h"
#include "ros2_communication.hpp"

//ЛЕВЫЙ МОТОР
void __left_motor_enc(); //Заголовок функции
//Создание экземпляра левого регулятора
Regulator left_regulator(
  Motor(7, 6),
  Encoder(3, 8, __left_motor_enc),
  PID(0.5, 0.01, 0.002, 100) 
);

void __left_motor_enc() {
  left_regulator.encoder.encoder_int();
}

//ПРАВЫЙ МОТОР
void __right_motor_enc(); //Заголовок функции
//Создание экземпляра правого регулятора
Regulator right_regulator(
  Motor(4, 5),
  Encoder(2, 9, __right_motor_enc),
  PID(0.5, 0.01, 0.002, 100) 
);

void __right_motor_enc() {
  right_regulator.encoder.encoder_int();
}

void setup() {
//  left_regulator.set_delta(5);
//  right_regulator.set_delta(5);
  Serial.begin(115200);
}

#define PT(x) Serial.print(x); Serial.print('\t')



void loop() {
  static unsigned long freq = millis();
  command_spin();
  


  if (millis() - freq >= 1000 * DT) {
    freq = millis();
    left_regulator.update(); //Не трогать.
    right_regulator.update(); //Не трогать.


    //Вывод для отладки в формате: реальная_скорость_левого_мотора установочная_скорость_левого_мотора    реальная_скорость_правого_мотора установочная_скорость_правого_мотора
//    PT(left_regulator.encoder.calc_delta()); //Отправка реальной скорости левого мотора
//    PT(left_regulator.delta); //Отправка установочной скорости левого мотора
//    Serial.print("\t");
//    PT(right_regulator.encoder.calc_delta()); //Отправка реальной скорости левого мотора
//    PT(right_regulator.delta); //Отправка установочной скорости левого мотора
//    Serial.println();
  }

  
}
