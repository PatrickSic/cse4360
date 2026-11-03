/*
 * robot.c - see robot.h, this is the implementation of those functions
 */
#include <ev3.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

#include "config.h"
#include "params.h"
#include "pid.h"
#include "robot.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* mmetres travelled per encoder degree (one wheel) */
#define M_PER_DEG   (M_PI * P.wheel_diameter / 360.0)

static Pose   g_pose;
static double g_gyro_offset = 0.0;   /* heading = P.gyro_sign*raw + offset */
static int    g_last_raw    = 0;
static double g_trim        = 0.0;          
static DriveStats g_drive;
static TurnStats  g_turn;

/*HELPER FUNCTIONS*/
static unsigned long now_ms(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (unsigned long)(tv.tv_sec * 1000UL + tv.tv_usec / 1000UL);
}

static double wrap180(double a)
{
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static int gyro_raw(void)
{
    int v = ReadEV3GyroSensorAngle(P.gyro_port, EV3GyroNormalAngle);
    if (v == INT_MIN) return g_last_raw;      
    g_last_raw = v;
    return v;
}

double robot_heading(void)
{
    return wrap180(P.gyro_sign * (double)gyro_raw() + g_gyro_offset);
}

static int enc_left(void)  { return P.drive_sign * MotorRotationCount((uint8_t)P.left_motor); }
static int enc_right(void) { return P.drive_sign * MotorRotationCount((uint8_t)P.right_motor); }

/* send to the firmware exactly one of each if:
 *   speed != 0 : opOutputSpeed, plus opOutputStart if the motor was stopped
 *   speed == 0 : opOutputStop (brake)
*/
static int g_running_left = 0, g_running_right = 0;

static void motor_cmd(uint8_t out, int speed, int *running)
{
    speed = (int)clampd(speed, -100, 100) * P.drive_sign;
    if (speed == 0) {
        if (*running) { OutputStop(out, true); *running = 0; }
        return;
    }
    OutputSpeed(out, (int8_t)speed);
    if (!*running) { OutputStart(out); *running = 1; }
}

static void motors_set(int left, int right)
{
    motor_cmd((uint8_t)P.left_motor,  left,  &g_running_left);
    motor_cmd((uint8_t)P.right_motor, right, &g_running_right);
}

static void motors_start(void)
{
    Fwd((uint8_t)(P.left_motor | P.right_motor));
}

void robot_stop(void)
{
    Off((uint8_t)(P.left_motor | P.right_motor));     /* brake */
    g_running_left = g_running_right = 0;
}

bool robot_abort_requested(void)
{
    return ButtonIsDown(BTNEXIT);
}

static void lcd_status(const char *what, double extra)
{
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(1), "%s", what);
    LcdTextf(1, 0, LcdRowToY(3), "x=%.3f y=%.3f", g_pose.x, g_pose.y);
    LcdTextf(1, 0, LcdRowToY(4), "hdg=%.1f gyro=%d", robot_heading(), g_last_raw);
    LcdTextf(1, 0, LcdRowToY(5), "val=%.3f", extra);
    LcdTextf(1, 0, LcdRowToY(7), "EXIT = abort");
}

/* integrate encoder motion along the current heading                  */
static double odom_update(int *lp, int *rp)
{
    int l = enc_left(), r = enc_right();
    double ds = ((l - *lp) + (r - *rp)) * 0.5 * M_PER_DEG;
    *lp = l; *rp = r;
    double h = robot_heading() * M_PI / 180.0;
    g_pose.x += ds * cos(h);
    g_pose.y += ds * sin(h);
    g_pose.heading = robot_heading();
    return ds;
}

/*initalize and shutdown*/
static bool gyro_calibrate(void)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(1), "Gyro calibrating");
        LcdTextf(1, 0, LcdRowToY(2), "DO NOT MOVE ROBOT");
        LcdTextf(1, 0, LcdRowToY(3), "attempt %d", attempt + 1);

        if (attempt == 0)
            ResetEV3GyroSensor(P.gyro_port, EV3GyroHardwareCommand0x88);
        else
            ResetEV3GyroSensor(P.gyro_port, EV3GyroHardwareReboot);
        Wait(1000);
        gyro_raw();
        ResetEV3GyroSensor(P.gyro_port, EV3GyroSoftwareOffset);
        Wait(200);

        int a = gyro_raw();
        Wait(1500);
        int b = gyro_raw();
        if (abs(b - a) <= 1) {
            g_gyro_offset = -P.gyro_sign * (double)gyro_raw();  /* heading 0 */
            return true;
        }
        PlayTone(300, 200);
    }
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(1), "GYRO DRIFT!");
    LcdTextf(1, 0, LcdRowToY(2), "unplug/replug gyro");
    LcdTextf(1, 0, LcdRowToY(3), "and restart");
    Wait(3000);
    return false;
}

bool robot_init(void)
{
    if (!InitEV3()) {
        fprintf(stderr, "InitEV3 failed\n");
        return false;
    }

    SensorHandler *ports[4] = { NULL, NULL, NULL, NULL };
    ports[P.gyro_port] = EV3Gyro;
    if (!SetAllSensors(ports[0], ports[1], ports[2], ports[3])) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(1), "Sensor init FAILED");
        LcdTextf(1, 0, LcdRowToY(2), "gyro on port %d?", P.gyro_port + 1);
        Wait(4000);
        return false;
    }
    Wait(500);                       

    ResetRotationCount((uint8_t)(P.left_motor | P.right_motor));
    robot_stop();
    g_trim = P.motor_trim;

    if (!gyro_calibrate()) return false;

    g_pose.x = g_pose.y = 0.0;
    g_pose.heading = 0.0;
    return true;
}

void robot_shutdown(void)
{
    robot_stop();
    FreeEV3();
}

void robot_set_pose(double x, double y, double heading_deg)
{
    g_pose.x = x;
    g_pose.y = y;
    g_pose.heading = heading_deg;
    /* re-zero: current physical heading == heading_deg */
    g_gyro_offset = heading_deg - P.gyro_sign * (double)gyro_raw();
}

Pose robot_get_pose(void) { return g_pose; }

/*TURN IN PLACE*/
bool robot_turn_to(double target)
{
    target = wrap180(target);
    unsigned long tstart = now_ms();
    int l_start = enc_left(), r_start = enc_right();
    memset(&g_turn, 0, sizeof(g_turn));
    g_turn.target = target;
    bool ok = false;

    /* PID on heading error -> signed wheel speed (+ = CCW).  
       P term does most of the work, D damps the approach
       I pushes static friction on the final degree, but this is toggelable and not really neccesary*/
    Pid pid;
    pid_init(&pid, P.turn_kp, P.turn_ki, P.turn_kd, P.turn_speed_min * 2.0, P.turn_speed_max);

    for (int attempt = 0; attempt < P.turn_retries; attempt++) {
        g_turn.attempts = attempt;
        double err = wrap180(target - robot_heading());
        if (fabs(err) <= P.turn_tol) { ok = true; break; }

        motors_start();
        pid_reset(&pid);
        unsigned long t0 = now_ms();
        int lp = enc_left(), rp = enc_right();
        double prev_h = robot_heading(), rate = 0.0;   /* deg/s, filtered */
        for (;;) {
            if (robot_abort_requested()) { robot_stop(); return false; }
            double h = robot_heading();
            rate = 0.7 * rate + 0.3 * wrap180(h - prev_h) / (LOOP_MS / 1000.0);
            prev_h = h;
            err = wrap180(target - h);
            /* the gyro reports late: stop when we WILL be inside tolerance */
            if (fabs(err) <= P.turn_tol + fabs(rate) * P.turn_lag) break;
            if (now_ms() - t0 > 10000) break;          /* stuck? */

            double u = pid_step(&pid, err, LOOP_MS / 1000.0);
            /* below turn_speed_min the motors do not move at all */
            if (fabs(u) < P.turn_speed_min) u = (err > 0 ? 1 : -1) * P.turn_speed_min;
            int sp = (int)(fabs(u) + 0.5);
            if (u > 0) motors_set(-sp, +sp);           /* CCW: left back */
            else       motors_set(+sp, -sp);

            odom_update(&lp, &rp);  /* keeps heading field fresh; ds ~ 0 */
            Wait(LOOP_MS);
        }
        robot_stop();
        Wait(P.turn_settle_ms);
        odom_update(&lp, &rp);
        lcd_status("turning", wrap180(target - robot_heading()));
        g_turn.attempts = attempt + 1;
    }
    g_turn.final_err = wrap180(target - robot_heading());
    g_turn.enc_left  = enc_left() - l_start;
    g_turn.enc_right = enc_right() - r_start;
    g_turn.time_ms   = now_ms() - tstart;
    if (!ok) ok = fabs(g_turn.final_err) <= 3.0 * P.turn_tol;
    return ok;
}
/*drives straight with a gyro head hold*/
bool robot_drive(double dist_m, double heading_deg)
{
    if (dist_m <= 0.0) return true;
    heading_deg = wrap180(heading_deg);

    int lp = enc_left(), rp = enc_right();
    const int l_start = lp, r_start = rp;
    memset(&g_drive, 0, sizeof(g_drive));
    g_drive.dist_cmd = dist_m;
    g_drive.trim_before = g_trim;
    double travelled = 0.0;
    unsigned long t0 = now_ms();

    /*The speed difference between the wheels is the heading hold (PID on heading error)*/
    Pid hold;
    pid_init(&hold, P.drive_kp, P.drive_ki, P.drive_kd, P.corr_max, P.corr_max);
    pid_set_integral_output(&hold, g_trim);
    unsigned long timeout = (unsigned long)(dist_m / 0.04 * 1000.0) + 5000UL;
    int tick = 0;

    motors_start();
    for (;;) {
        if (robot_abort_requested()) { robot_stop(); return false; }

        travelled += odom_update(&lp, &rp);
        double remaining = dist_m - travelled;
        if (remaining <= P.brake_m) break;         /*robot is rolling*/
        if (now_ms() - t0 > timeout) { robot_stop(); return false; }

        /*cruise then slow down*/
        double base = P.drive_speed;
        if (remaining < P.slow_zone)
            base = P.drive_speed_min + (P.drive_speed - P.drive_speed_min) * remaining / P.slow_zone;
        if (base < P.drive_speed_min) base = P.drive_speed_min;

        double err  = wrap180(heading_deg - robot_heading());
        if (fabs(err) > g_drive.max_abs_err) g_drive.max_abs_err = fabs(err);
        double corr = pid_step(&hold, err, LOOP_MS / 1000.0);
        motors_set((int)(base - corr + 0.5), (int)(base + corr + 0.5));

        if ((++tick % 20) == 0) lcd_status("driving", remaining);
        Wait(LOOP_MS);
    }
    robot_stop();
    Wait(150);
    odom_update(&lp, &rp);     /*need to account for the difference in distance due to braking*/

    /*take the information from the integrator (the trim) and apply it to the next segment*/
    if (travelled > 0.3)
        g_trim = clampd(pid_integral_output(&hold), -TRIM_MAX, TRIM_MAX);

    g_drive.enc_left  = enc_left() - l_start;
    g_drive.enc_right = enc_right() - r_start;
    g_drive.dist_enc  = (g_drive.enc_left + g_drive.enc_right) * 0.5 * M_PER_DEG;
    g_drive.final_err = wrap180(heading_deg - robot_heading());
    g_drive.trim_after = g_trim;
    g_drive.time_ms = now_ms() - t0;
    return true;
}

void robot_log_to_file(const char *path)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd < 0) return;
    dup2(fd, 1);
    dup2(fd, 2);
    close(fd);
    setvbuf(stdout, NULL, _IOLBF, 0);
}

double robot_get_trim(void)        { return g_trim; }
void   robot_set_trim(double trim) { g_trim = clampd(trim, -TRIM_MAX, TRIM_MAX); }
DriveStats robot_last_drive(void)  { return g_drive; }
TurnStats  robot_last_turn(void)   { return g_turn; }
int  robot_enc_left(void)          { return enc_left(); }
int  robot_enc_right(void)         { return enc_right(); }
int  robot_gyro_raw(void)          { return gyro_raw(); }
void robot_motors(int left, int right) { motors_start(); motors_set(left, right); }

bool robot_goto(double x, double y)
{
    double dx = x - g_pose.x, dy = y - g_pose.y;
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < P.goal_tol) return true;
    double hdg = atan2(dy, dx) * 180.0 / M_PI;

    if (!robot_turn_to(hdg)) return false;

    /*after doing a turn the robot reevaulates*/
    dx = x - g_pose.x; dy = y - g_pose.y;
    dist = sqrt(dx * dx + dy * dy);
    hdg = atan2(dy, dx) * 180.0 / M_PI;
    return robot_drive(dist, hdg);
}

/*TESTS*/
void robot_test_gyro(void)
{
    robot_set_pose(0, 0, 0);
    while (!robot_abort_requested()) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(0), "GYRO TEST");
        LcdTextf(1, 0, LcdRowToY(2), "raw    = %d", gyro_raw());
        LcdTextf(1, 0, LcdRowToY(3), "heading= %.1f", robot_heading());
        LcdTextf(1, 0, LcdRowToY(5), "Turn robot LEFT by");
        LcdTextf(1, 0, LcdRowToY(6), "hand: heading must");
        LcdTextf(1, 0, LcdRowToY(7), "go UP (+90). Else");
        LcdTextf(1, 0, LcdRowToY(8), "flip P.gyro_sign.");
        LcdTextf(1, 0, LcdRowToY(10), "EXIT = back");
        Wait(100);
    }
    Wait(500);
}

void robot_test_drive(double dist_m)
{
    robot_set_pose(0, 0, 0);
    int l0 = enc_left(), r0 = enc_right();
    bool ok = robot_drive(dist_m, 0.0);
    int dl = enc_left() - l0, dr = enc_right() - r0;
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "DRIVE TEST %s", ok ? "done" : "ABORTED");
    LcdTextf(1, 0, LcdRowToY(2), "cmd  = %.3f m", dist_m);
    LcdTextf(1, 0, LcdRowToY(3), "encL = %d deg", dl);
    LcdTextf(1, 0, LcdRowToY(4), "encR = %d deg", dr);
    LcdTextf(1, 0, LcdRowToY(5), "hdg  = %.1f trim %.1f", robot_heading(), g_trim);
    LcdTextf(1, 0, LcdRowToY(7), "Measure real dist d:");
    LcdTextf(1, 0, LcdRowToY(8), "D_new = D * d/%.1f", dist_m);
    LcdTextf(1, 0, LcdRowToY(10), "EXIT = back");
    printf("drive test: cmd=%.3f encL=%d encR=%d hdg=%.1f\n", dist_m, dl, dr, robot_heading());
    while (!robot_abort_requested()) Wait(50);
    Wait(500);
}

void robot_test_square(double side_m)
{
    robot_set_pose(0, 0, 0);
    for (int k = 0; k < 4; k++) {
        if (!robot_drive(side_m, 90.0 * k)) return;
        if (!robot_turn_to(90.0 * (k + 1))) return;
    }
    Pose p = robot_get_pose();
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "SQUARE TEST done");
    LcdTextf(1, 0, LcdRowToY(2), "pose x=%.3f", p.x);
    LcdTextf(1, 0, LcdRowToY(3), "     y=%.3f", p.y);
    LcdTextf(1, 0, LcdRowToY(4), "   hdg=%.1f", robot_heading());
    LcdTextf(1, 0, LcdRowToY(6), "Robot should be back");
    LcdTextf(1, 0, LcdRowToY(7), "at start, facing +X");
    LcdTextf(1, 0, LcdRowToY(10), "EXIT = back");
    printf("square test: pose %.3f %.3f hdg %.1f\n", p.x, p.y, robot_heading());
    while (!robot_abort_requested()) Wait(50);
    Wait(500);
}
