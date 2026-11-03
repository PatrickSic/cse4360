#ifndef ROBOT_H
#define ROBOT_H

#include <stdbool.h>

/*a pose is the location*/
typedef struct {
    double x, y;          /* m */
    double heading;       /* deg, CCW, 0 = +X */
} Pose;

bool robot_init(void);
void robot_shutdown(void);

/* Set the pose of the robot;
   call once with the start location + START_HEADING_DEG */

void robot_set_pose(double x, double y, double heading_deg);
Pose robot_get_pose(void);

double robot_heading(void);

bool robot_turn_to(double heading_deg);

/*drives straight forward for a certain distance while holding an angle*/
bool robot_drive(double dist_m, double heading_deg);

bool robot_goto(double x, double y);

/* Current learned straight-line trim (speed units, + = right harder).  */
double robot_get_trim(void);
void   robot_set_trim(double trim);

/* Statistics of the last robot_drive() / robot_turn_to() call.*/
typedef struct {
    double dist_cmd, dist_enc;      /* commanded vs encoder-measured (m) */
    double max_abs_err, final_err;  /* heading error (deg) */
    double trim_before, trim_after; /* speed units */
    int    enc_left, enc_right;     /* encoder degrees this run */
    unsigned long time_ms;
} DriveStats;

typedef struct {
    double target, final_err;       /* deg */
    int    attempts;
    int    enc_left, enc_right;     /* encoder degrees this turn */
    unsigned long time_ms;
} TurnStats;

DriveStats robot_last_drive(void);
TurnStats  robot_last_turn(void);

void robot_log_to_file(const char *path);

/*more diagnostics*/
int  robot_enc_left(void);
int  robot_enc_right(void);
int  robot_gyro_raw(void);
void robot_motors(int left, int right);   /* start both, speeds in robot-forward units */

/*brake*/
void robot_stop(void);

bool robot_abort_requested(void);

void robot_test_gyro(void); /*shows live gyro heading*/           
void robot_test_drive(double dist_m); /*drive straight and show encorder*/
void robot_test_square(double side_m); /*drive a square and compare pose*/

#endif /* ROBOT_H */