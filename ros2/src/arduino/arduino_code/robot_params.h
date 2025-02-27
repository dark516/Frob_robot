#pragma once
#include <math.h>
constexpr float DT = 0.01f; // Важно: Добавить эту строку!
// Железо робота
constexpr byte PIN_TRIG = 11;
constexpr byte PIN_ECHO = 12;

// Параметры колесной базы
constexpr float WHEEL_DIAMETER = 0.067f;     // [m]
constexpr int TICKS_PER_REV = 372;           // [ticks/rev]
constexpr float WHEEL_BASE = 0.18f;         // [m]

// Ограничения
constexpr float MAX_LIN_SPEED = 0.4863f;        // [m/s]
constexpr float MAX_ANG_SPEED = 5.4f;        // [rad/s]
constexpr float MAX_LIN_ACCEL = 5.0f;        // [m/s²]
constexpr float MAX_ANG_ACCEL = 5.0f;        // [rad/s²]

// Расчетные константы
constexpr float METERS_PER_TICK = 0.00057f;
constexpr float TICKS_PER_METER = 1.0f / METERS_PER_TICK;
constexpr int MAX_DELTA_TICKS = static_cast<int>(MAX_LIN_SPEED * TICKS_PER_METER);
