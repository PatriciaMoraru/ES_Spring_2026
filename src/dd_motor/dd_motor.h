// DC motor device driver — controls a single DC motor via an L298N H-bridge driver.
// Speed is specified as a percentage (0-100%) and mapped to a PWM duty cycle.
// Direction is controlled by IN1 / IN2 logic levels.
#ifndef DD_MOTOR_H
#define DD_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define DD_MOTOR_MAX_COUNT 2

// Initialise one motor slot. Must be called once before any other dd_motor call.
//   id      : 0 .. DD_MOTOR_MAX_COUNT-1
//   pinEN   : Arduino PWM-capable pin connected to L298N ENA/ENB
//   pinIN1  : Arduino digital pin connected to L298N IN1/IN3
//   pinIN2  : Arduino digital pin connected to L298N IN2/IN4
void ddMotorSetup(int id, int pinEN, int pinIN1, int pinIN2);

// Set motor speed and direction.
//   speedPct : 0-100 (clamped internally; driver may apply its own cap)
//   forward  : true = forward (IN1=H, IN2=L), false = reverse (IN1=L, IN2=H)
void ddMotorSetSpeed(int id, uint8_t speedPct, bool forward);

// Brake the motor to a stop (IN1=L, IN2=L, ENA=0).
void ddMotorStop(int id);

// Return last applied speed percentage (0-100).
uint8_t ddMotorGetSpeed(int id);

// Return last applied direction (true = forward).
bool ddMotorGetDirection(int id);

#endif
