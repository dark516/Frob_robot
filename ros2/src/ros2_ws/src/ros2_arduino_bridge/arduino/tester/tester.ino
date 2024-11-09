// Перечисление со всеми командами
enum Commands : uint8_t {
  PIN_MODE = 0x10,
  DIGITAL_WRITE = 0x11,
  DIGITAL_READ = 0x12,
  DELAY_MS = 0x13,
};

void setup() {
  // повышенная скорость порта
  Serial.begin(115200);

  pinMode(13, OUTPUT);
}

void wait_bytes(uint8_t b) {
  while (Serial.available() < b) {
    ; // Ожидание прибытия всего пакета
  }
}

void on_pin_mode() {
  wait_bytes(sizeof(uint8_t) + sizeof(uint8_t));

  uint8_t pin = Serial.read();
  uint8_t mode = Serial.read();

  pinMode(pin, mode);
}

void on_di_write() {
  wait_bytes(sizeof(uint8_t) + sizeof(bool));

  uint8_t pin = Serial.read();
  bool state = Serial.read();

  digitalWrite(pin, state);
}

void on_DIGITAL_READ() {
  wait_bytes(sizeof(uint8_t));

  uint8_t pin = Serial.read();

  uint8_t state = digitalRead(pin);
  Serial.write(state);
}

void on_DELAY_MS () {
  wait_bytes(sizeof(uint32_t);

  uint32_t t;
  Serial.readBytes((uint8_t*)&t, sizeof(t));

  delay(t);
}

void loop() {
  if (Serial.available() < 1) return;
  uint8_t code = Serial.read();

  switch (code) {
    case PIN_MODE: on_pin_mode(); break;
    case DIGITAL_WRITE: on_di_write(); break;
    case DIGITAL_READ: on_DIGITAL_READ(); break;
    case DELAY_MS: on_DELAY_MS(); break;
  }
}
