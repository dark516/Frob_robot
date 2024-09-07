#define DT 0.01
#define MAX_DELTA 13 //Максимальная фактическая скорость

struct PID {
  float kp, ki, kd, max_i;
  float integral = 0;
  int old_error = 0;

  PID (float p, float i, float d, float mi) : kp(p), ki(i), kd(d), max_i(mi) {}
  
  int calc(int error) {
    integral += error * DT * ki;
    integral = constrain(integral, -max_i, max_i);
    float diff = (error - old_error) * kd / DT;
    return error * kp + integral + diff;
  }
};

struct Encoder {
  volatile long ticks = 0, prev_ticks = 0;
  byte pin_a, pin_b;
  
  Encoder (byte pin_enc_a, byte pin_enc_b, void(*on_enc)(void)) : pin_a(pin_enc_a), pin_b(pin_enc_b) {
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), on_enc, RISING);
  }

  void encoder_int() {
    if (digitalRead(pin_b)) ticks++;
    else ticks--;
  }

  int calc_delta() {
    int ret = ticks - prev_ticks;
    prev_ticks = ticks;
    return ret;
  }
  
};

struct Motor {
  byte dir, speed;
  
  Motor (byte pin_dir, byte pin_speed) : dir(pin_dir), speed(pin_speed) {
    pinMode(dir, OUTPUT);
    pinMode(speed, OUTPUT);
  }

  void set_pwmdir(int pwm_dir) {
    int pwm = constrain(abs(pwm_dir), 0, 255);
    analogWrite(speed, pwm);
    digitalWrite(dir, pwm_dir > 0);
  }
};

struct Regulator { 

  Motor motor;
  Encoder encoder;
  PID pid;

  long next = 0;
  int delta = 0;
  
  
  Regulator (Motor&& motor, Encoder&& encoder, PID&& pid) : motor(motor), encoder(encoder), pid(pid) {}

  void update() {
    next += delta;
    motor.set_pwmdir(pid.calc(next - encoder.ticks));
  }

  void set_delta(int new_delta) {
    delta = constrain(new_delta, -MAX_DELTA, MAX_DELTA);
  }
    
};

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
  //left_regulator.set_delta(2);
  Serial.begin(9600);
}

#define PT(x) Serial.print(x); Serial.print('\t')

void loop() {
  static unsigned long freq = millis();

  if (Serial.available() >= 4) {  // Ожидаем 4 байта данных (2 числа по 2 байта)
    int left_motor_speed = 0;
    int right_motor_speed = 0;
    
    byte buffer[4];

    // Читаем 4 байта
    for (int i = 0; i < 4; i++) {
      buffer[i] = Serial.read();
    }

    // Преобразуем байты обратно в два числа (тип short int)
    left_motor_speed = (short)((buffer[1] << 8) | buffer[0]);
    right_motor_speed = (short)((buffer[3] << 8) | buffer[2]);
    
    left_regulator.set_delta(left_motor_speed);
    right_regulator.set_delta(right_motor_speed);
  }

  if (millis() - freq >= 1000 * DT) {
    freq = millis();
    left_regulator.update(); //Не трогать.
    right_regulator.update(); //Не трогать.


    //Вывод для отладки в формате: реальная_скорость_левого_мотора установочная_скорость_левого_мотора    реальная_скорость_правого_мотора установочная_скорость_правого_мотора
//    PT(left_regulator.encoder.calc_delta()); //Отправка реальной скорости левого мотора
//    PT(left_regulator.delta); //Отправка установочной скорости левого мотора
//    Serial.print("/t");
//    PT(right_regulator.encoder.calc_delta()); //Отправка реальной скорости левого мотора
//    PT(right_regulator.delta); //Отправка установочной скорости левого мотора
//    Serial.println();
  }

  
}
