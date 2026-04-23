// Discrete PID controller module
//
// Implements R(t) = Kp*e(t) + Ki*sum(e*dt) + Kd*(e(t)-e(t-dt))/dt
// with output clamping and clamp-based anti-windup (integral is not
// accumulated when the output is saturated against the error sign).
#ifndef CTRL_PID_H
#define CTRL_PID_H

#include <stdbool.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prevError;

    float outMin;
    float outMax;
    float dt;          // control period in seconds

    bool  firstRun;    // derivative term is skipped on the first tick
} CtrlPid_t;

void  ctrlPidInit(CtrlPid_t *p, float kp, float ki, float kd,
                  float outMin, float outMax, float dt);
void  ctrlPidSetGains(CtrlPid_t *p, float kp, float ki, float kd);
void  ctrlPidReset(CtrlPid_t *p);
float ctrlPidCompute(CtrlPid_t *p, float setPoint, float measured);

#endif
