/*
 * pid.h
 *
 *   u = kp*e + ki*integral(e dt) + kd*de/dt
 *   - the integral term alone is clamped to +-i_limit
 *   - output clamp to +-out_limit
 * 
 * Usage:
 *   Pid h; pid_init(&h, kp, ki, kd, i_limit, out_limit);
 *   loop: u = pid_step(&h, error, dt_seconds);
 */
#ifndef PID_H
#define PID_H

typedef struct {
    double kp, ki, kd;
    double i_limit;        /* max |ki * integral| */
    double out_limit;      /* max |u| */
    double d_alpha;        /* derivative filter*/

    double integ;          /* integral of e dt */
    double deriv;          /* filtered de/dt */
    double prev_err;
    int    primed;         /* if prev_err valid */
    double last_p, last_i, last_d;   /* last term values, for logging */
} Pid;

void   pid_init(Pid *p, double kp, double ki, double kd, double i_limit, double out_limit);
void   pid_reset(Pid *p);
double pid_step(Pid *p, double err, double dt);

/* integral term's contribution (ki * integ), e.g. to carry over as trim */
double pid_integral_output(const Pid *p);
/* preload so that the integral term contributes so that we can carry over information between segments of grids*/
void   pid_set_integral_output(Pid *p, double value);

#endif /* PID_H */
