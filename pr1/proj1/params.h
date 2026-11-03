/*
 * params.h
 * Defaults parameters come from config.h
 *
 * proj1.cfg format (one "key value" per line, '#' starts a comment):
 *
 *   wheel_diameter 0.0556
 *   gyro_sign -1
 *   motor_trim -0.2
 *   drive_kp 2.0
 *
 * course.txt format:
 *
 *   start 0.305 1.219
 *   goal  3.658 1.829
 *   obstacle 0.61 2.743
 *   obstacle 0.915 2.743
 *   heading 0         
 */
#ifndef PARAMS_H
#define PARAMS_H

typedef struct {
    /* geometry */
    double wheel_diameter;      /* m */
    double track_width;         /* m */
    double robot_radius;        /* m */
    double safety_margin;       /* m */
    /* ports & signs */
    int    left_motor, right_motor;   /* OUT_A..OUT_D bitmasks */
    int    gyro_port;                 /* 0..3 */
    int    drive_sign, gyro_sign;     /* +1 / -1 */
    double start_heading;             /* deg */
    /* driving */
    int    drive_speed, drive_speed_min;
    double slow_zone, brake_m;
    double drive_kp, drive_ki, drive_kd, corr_max;
    double motor_trim;
    /* turning */
    int    turn_speed_max, turn_speed_min;
    double turn_kp, turn_ki, turn_kd, turn_tol, turn_lag;
    int    turn_settle_ms, turn_retries;
    /* goal */
    double goal_tol;
} Params;

extern Params P;

/*config.h defaults*/
void params_defaults(void);
int params_load(const char *path, int *unknown);

/*summarizes the logs into one line*/                         
void params_log(void);

/*Course file: fills the arrays (max n), and returns the number of ostacles, * -1 if the file does not exist. start and goal are only changed if the file is present*/
int course_load(const char *path, double obstacles[][2], int max,
                double start[2], double goal[2], double *heading_deg);

#endif /* PARAMS_H */
