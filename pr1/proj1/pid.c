/*
 * pid.c - see pid.h
 */
#include "pid.h"

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void pid_init(Pid *p, double kp, double ki, double kd, double i_limit, double out_limit)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->i_limit = i_limit < 0 ? -i_limit : i_limit;
    p->out_limit = out_limit < 0 ? -out_limit : out_limit;
    p->d_alpha = 0.2;
    pid_reset(p);
}

void pid_reset(Pid *p)
{
    p->integ = 0.0;
    p->deriv = 0.0;
    p->prev_err = 0.0;
    p->primed = 0;
    p->last_p = p->last_i = p->last_d = 0.0;
}

double pid_integral_output(const Pid *p)
{
    return p->ki * p->integ;
}

void pid_set_integral_output(Pid *p, double value)
{
    if (p->ki > 0.0) p->integ = clampd(value, -p->i_limit, p->i_limit) / p->ki;
    else             p->integ = 0.0;
}

double pid_step(Pid *p, double err, double dt)
{
    if (dt <= 0.0) dt = 1e-3;

    /* integral with anti-windup (limit expressed on the term's output) */
    if (p->ki > 0.0) {
        double lim = p->i_limit / p->ki;
        p->integ = clampd(p->integ + err * dt, -lim, lim);
    }

    /* filtered derivative; first sample has no history */
    double raw_d = p->primed ? (err - p->prev_err) / dt : 0.0;
    p->deriv = p->primed ? (1.0 - p->d_alpha) * p->deriv + p->d_alpha * raw_d : 0.0;
    p->prev_err = err;
    p->primed = 1;

    p->last_p = p->kp * err;
    p->last_i = p->ki * p->integ;
    p->last_d = p->kd * p->deriv;
    return clampd(p->last_p + p->last_i + p->last_d, -p->out_limit, p->out_limit);
}
