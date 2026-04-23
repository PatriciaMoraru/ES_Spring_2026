// Discrete PID controller implementation
#include "ctrl_pid.h"

void ctrlPidInit(CtrlPid_t *p, float kp, float ki, float kd,
                 float outMin, float outMax, float dt)
{
    if (!p)
        return;

    p->kp = kp;
    p->ki = ki;
    p->kd = kd;

    p->outMin = outMin;
    p->outMax = outMax;
    p->dt     = (dt > 0.0f) ? dt : 0.001f;

    ctrlPidReset(p);
}

void ctrlPidSetGains(CtrlPid_t *p, float kp, float ki, float kd)
{
    if (!p)
        return;

    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void ctrlPidReset(CtrlPid_t *p)
{
    if (!p)
        return;

    p->integral  = 0.0f;
    p->prevError = 0.0f;
    p->firstRun  = true;
}

float ctrlPidCompute(CtrlPid_t *p, float setPoint, float measured)
{
    if (!p)
        return 0.0f;

    float error = setPoint - measured;

    // Derivative (skip on first tick to avoid a large spurious kick)
    float derivative = 0.0f;
    if (!p->firstRun)
        derivative = (error - p->prevError) / p->dt;

    // Tentative integral update
    float newIntegral = p->integral + error * p->dt;

    // Raw PID output using the tentative integral
    float output = p->kp * error
                 + p->ki * newIntegral
                 + p->kd * derivative;

    // Clamp output and decide whether to commit the integral update
    // (clamp-based anti-windup: only integrate when not pushing further
    //  into saturation in the same direction as the error).
    bool saturatedHigh = (output > p->outMax);
    bool saturatedLow  = (output < p->outMin);

    if (saturatedHigh)
        output = p->outMax;
    else if (saturatedLow)
        output = p->outMin;

    bool blockIntegration =
        (saturatedHigh && error > 0.0f) ||
        (saturatedLow  && error < 0.0f);

    if (!blockIntegration)
        p->integral = newIntegral;

    p->prevError = error;
    p->firstRun  = false;

    return output;
}
