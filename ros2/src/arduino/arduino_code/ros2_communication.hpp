#pragma once

#include "motor_regulator.h"

extern Regulator left_regulator;
extern Regulator right_regulator;

//Команды с ROS ноды
enum Commands : uint8_t {
  SET_MOTORS = 0x10,
  GET_DATA = 0x11,
};

struct Data {
  int16_t left_encoder_delta;
  int16_t right_encoder_delta;
};

template <typename T> void serial_read(T& dest) {
  wait_bytes(sizeof(dest));
  Serial.readBytes((uint8_t* ) &dest, sizeof(T));
}

template <typename T> void serial_send(T& src) {
  Serial.write((uint8_t *) &src, sizeof(T));
}

void wait_bytes(uint8_t b) {
  while (Serial.available() < b) {
    ; // Ожидание прибытия всего пакета
  }
}

void set_motors(){
  wait_bytes(sizeof(int8_t) + sizeof(int8_t));

  int8_t left_motor_speed = Serial.read();
  int8_t right_motor_speed = Serial.read();
  left_regulator.set_delta(left_motor_speed);
  right_regulator.set_delta(right_motor_speed);
}

void get_data(){
  wait_bytes(sizeof(int8_t));
  left_regulator.encoder.calc_delta();
  right_regulator.encoder.calc_delta();
  Data ret{left_regulator.encoder.speed, right_regulator.encoder.speed};
  Serial.write((uint8_t*)&ret, sizeof(ret));  
}

void command_spin(){
    //Обработчик комманд
  if (Serial.available() > 1) {
    uint8_t code = Serial.read();
    switch (code) {
    case SET_MOTORS: set_motors(); break;
    case GET_DATA: get_data(); break; 
    }
  }
}
