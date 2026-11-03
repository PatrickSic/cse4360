/*
 path_compute.c -> standalone ev3-less build
 */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "config.h"
#include "params.h"
#include "robot.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static Pose g_pose;
static double g_trim;
static DriveStats g_drive;
static TurnStats g_turn;
static int g_step = 0;
static double g_total_dist = 0, g_total_turn = 0;

static double wrap180(double a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }

void robot_log_to_file(const char *path) { (void)path; }
bool robot_init(void) { g_trim = P.motor_trim; printf("[robot] simulated robot ready (no hardware)\n"); return true; }
void robot_shutdown(void)
{
    printf("[robot] done: %d motions, %.2f m driven, %.0f deg turned, final pose (%.3f, %.3f, %.1f)\n",
           g_step, g_total_dist, g_total_turn, g_pose.x, g_pose.y, g_pose.heading);
}
void robot_set_pose(double x, double y, double h) { g_pose.x = x; g_pose.y = y; g_pose.heading = wrap180(h);
    printf("[robot] placed at (%.3f, %.3f) heading %.0f\n", x, y, h); }
Pose   robot_get_pose(void)   { return g_pose; }
double robot_heading(void)    { return g_pose.heading; }
void   robot_stop(void)       { }
bool   robot_abort_requested(void) { return false; }

bool robot_turn_to(double target)
{
    target = wrap180(target);
    double d = wrap180(target - g_pose.heading);
    memset(&g_turn, 0, sizeof g_turn);
    g_turn.target = target;
    if (fabs(d) <= P.turn_tol) return true;
    printf("  %2d. turn %+4.0f deg  -> heading %4.0f  (%s)\n", ++g_step, d, target,
           fabs(fabs(d) - 180) < 1 ? "about-face" : d > 0 ? "left" : "right");
    g_total_turn += fabs(d);
    g_pose.heading = target;
    return true;
}

bool robot_drive(double dist_m, double heading_deg)
{
    if (dist_m <= 0) return true;
    double h = wrap180(heading_deg) * M_PI / 180.0;
    g_pose.x += dist_m * cos(h);
    g_pose.y += dist_m * sin(h);
    g_pose.heading = wrap180(heading_deg);
    printf("  %2d. drive %.3f m     -> (%.3f, %.3f)\n", ++g_step, dist_m, g_pose.x, g_pose.y);
    g_total_dist += dist_m;
    memset(&g_drive, 0, sizeof g_drive);
    g_drive.dist_cmd = g_drive.dist_enc = dist_m;
    return true;
}

bool robot_goto(double x, double y)
{
    double dx = x - g_pose.x, dy = y - g_pose.y, dist = sqrt(dx * dx + dy * dy);
    if (dist < P.goal_tol) return true;
    double hdg = atan2(dy, dx) * 180.0 / M_PI;
    if (!robot_turn_to(hdg)) return false;
    return robot_drive(dist, hdg);
}

double robot_get_trim(void) { return g_trim; }
void   robot_set_trim(double t) { g_trim = t; }
DriveStats robot_last_drive(void) { return g_drive; }
TurnStats  robot_last_turn(void)  { return g_turn; }
int  robot_enc_left(void)  { return 0; }
int  robot_enc_right(void) { return 0; }
int  robot_gyro_raw(void)  { return 0; }
void robot_motors(int l, int r) { (void)l; (void)r; }
void robot_test_gyro(void) { }
void robot_test_drive(double d) { (void)d; }
void robot_test_square(double s) { (void)s; }
