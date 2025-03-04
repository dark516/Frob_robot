#pragma once
#include "robot_params.h"
struct PID {
  float kp, ki, kd, max_i;
  float integral = 0;
  float prev_error = 0;

  PID(float p, float i, float d, float mi) 
    : kp(p), ki(i), kd(d), max_i(mi) {}

  int calc(float error) {
    integral += error * DT * ki;
    integral = constrain(integral, -max_i, max_i);
    float diff = (error - prev_error) * kd / DT;
    prev_error = error;
    return static_cast<int>(error * kp + integral + diff);
  }
};

struct Encoder {
  volatile long ticks = 0;
  long prev_ticks = 0;
  int speed = 0;
  byte pin_a, pin_b;
  bool invert;

  Encoder(byte a, byte b, void(*isr)(), bool inv) 
    : pin_a(a), pin_b(b), invert(inv) {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), isr, RISING);
  }

  void encoder_int() {
    int dir = (digitalRead(pin_b) ^ invert) ? 1 : -1;
    ticks += dir;
  }

  void calc_delta() {
    noInterrupts();
    speed = ticks - prev_ticks;
    prev_ticks = ticks;
    interrupts();
  }
};

struct Motor {
  byte pin_dir, pin_pwm;
  
  Motor(byte dir, byte pwm) : pin_dir(dir), pin_pwm(pwm) {
    pinMode(pin_dir, OUTPUT);
    pinMode(pin_pwm, OUTPUT);
  }

  void set_pwmdir(int speed) {
    speed = constrain(speed, -255, 255);
    digitalWrite(pin_dir, speed > 0);
    analogWrite(pin_pwm, abs(speed));
  }
};

struct Regulator {
  Motor& motor;
  Encoder& encoder;
  PID& pid;
  
  float target_speed = 0;
  float current_speed = 0;
  double position_target = 0;
  float max_accel;
  
  // Добавляем переменные для управления таймером
  unsigned long last_nonzero_speed_time = 0;
  bool is_position_reset = false;

  Regulator(Motor& m, Encoder& e, PID& p)
    : motor(m), encoder(e), pid(p) {
    max_accel = MAX_LIN_ACCEL * TICKS_PER_METER;
  }

  void set_speed(float new_speed) {
    target_speed = constrain(new_speed, -MAX_DELTA_TICKS, MAX_DELTA_TICKS);
    
    // Сбрасываем таймер при получении любой новой скорости
    if (target_speed != 0) {
      last_nonzero_speed_time = millis();
      is_position_reset = false;
    }
  }

  void update() {
    // Плавное изменение скорости
    float speed_diff = target_speed - current_speed;
    float allowed_diff = copysignf(max_accel * DT, speed_diff);
    
    if (fabs(speed_diff) > fabs(allowed_diff)) {
      current_speed += allowed_diff;
    } else {
      current_speed = target_speed;
    }

    // Обновление целевой позиции
    position_target += current_speed * DT;

    // Если скорость нулевая более 1 секунды - синхронизируем позицию
    if (target_speed == 0 && current_speed == 0) {
      if (!is_position_reset) {
        if (millis() - last_nonzero_speed_time > 1000) {
          position_target = encoder.ticks;
          is_position_reset = true;
        }
      }
    }

    // ПИД-регулятор
    int error = position_target - encoder.ticks;
    int pwm = pid.calc(error);
    
    // Если позиция синхронизирована - отключаем мотор
    if (is_position_reset) {
      pwm = 0;
    }
    
    motor.set_pwmdir(pwm);
    encoder.calc_delta();
  }
};
