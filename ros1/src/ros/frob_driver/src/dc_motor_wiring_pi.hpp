
#ifndef DC_MOTOR_WIRING_PI_HPP_
#define DC_MOTOR_WIRING_PI_HPP_

#include <ros/ros.h>
#include <wiringPi.h>

#define RPI_MAX_PWM_VALUE 1023

class DCMotorWiringPi {
public:
    DCMotorWiringPi(int8_t direction_pin, int8_t enable_pin);
    void cw(uint16_t val);
    void ccw(uint16_t val);
    void stop();
private:
    int8_t _direction_pin;
    int8_t _enable_pin;
    uint16_t protectOutput(uint16_t val);
};

DCMotorWiringPi::DCMotorWiringPi(int8_t direction_pin, int8_t enable_pin) {
    _direction_pin = direction_pin;
    _enable_pin = enable_pin;
    if (wiringPiSetupGpio() < 0) {
        ROS_ERROR("DCMotor wiringPi error: GPIO setup error");
        throw std::runtime_error("");
    }
    ROS_INFO("DCMotor wiringPi: GPIO setup");
    pinMode(_direction_pin, OUTPUT);
    pinMode(_enable_pin, PWM_OUTPUT);
    stop();
    ROS_INFO("DCMotor wiringPi: Motor setup");
}

void DCMotorWiringPi::stop() {
    pwmWrite(_enable_pin, 0);
    digitalWrite(_direction_pin, 0);
}

void DCMotorWiringPi::cw(uint16_t val) {
    pwmWrite(_enable_pin, protectOutput(val));
    digitalWrite(_direction_pin, 1);
}

void DCMotorWiringPi::ccw(uint16_t val) {
    pwmWrite(_enable_pin, protectOutput(val));
    digitalWrite(_direction_pin, 0);
}

uint16_t DCMotorWiringPi::protectOutput(uint16_t val) {
    return val > RPI_MAX_PWM_VALUE ? RPI_MAX_PWM_VALUE : val;
}

#endif // DC_MOTOR_WIRING_PI_HPP_