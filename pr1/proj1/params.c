/*
 * params.c - see params.h
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#ifdef HOST_BUILD             /* laptop test build: stand-ins for EV3 port macros */
#define OUT_A 1
#define OUT_B 2
#define OUT_C 4
#define OUT_D 8
#define IN_1 0
#define IN_2 1
#define IN_3 2
#define IN_4 3
#else
#include <ev3.h>              /* OUT_x / IN_x for config.h defaults */
#endif
#include "config.h"
#include "params.h"

Params P;

void params_defaults(void)
{
    memset(&P, 0, sizeof P);
    P.wheel_diameter  = WHEEL_DIAMETER_M;
    P.track_width     = TRACK_WIDTH_M;
    P.robot_radius    = ROBOT_RADIUS_M;
    P.safety_margin   = SAFETY_MARGIN_M;
    P.left_motor      = LEFT_MOTOR;
    P.right_motor     = RIGHT_MOTOR;
    P.gyro_port       = GYRO_PORT;
    P.drive_sign      = DRIVE_SIGN;
    P.gyro_sign       = GYRO_SIGN;
    P.start_heading   = START_HEADING_DEG;
    P.drive_speed     = DRIVE_SPEED;
    P.drive_speed_min = DRIVE_SPEED_MIN;
    P.slow_zone       = DRIVE_SLOW_ZONE_M;
    P.brake_m         = DRIVE_BRAKE_M;
    P.drive_kp        = DRIVE_KP;
    P.drive_ki        = DRIVE_KI;
    P.drive_kd        = DRIVE_KD;
    P.corr_max        = DRIVE_CORR_MAX;
    P.motor_trim      = MOTOR_TRIM;
    P.turn_speed_max  = TURN_SPEED_MAX;
    P.turn_speed_min  = TURN_SPEED_MIN;
    P.turn_kp         = TURN_KP;
    P.turn_ki         = TURN_KI;
    P.turn_kd         = TURN_KD;
    P.turn_tol        = TURN_TOL_DEG;
    P.turn_lag        = TURN_LAG_S;
    P.turn_settle_ms  = TURN_SETTLE_MS;
    P.turn_retries    = TURN_MAX_RETRIES;
    P.goal_tol        = GOAL_TOL_M;
}

/* "B" / "C" / "OUT_B" / "2" -> OUT_x bitmask;  returns -1 if invalid   */
static int parse_motor(const char *v)
{
    if (!strncasecmp(v, "OUT_", 4)) v += 4;
    if (strlen(v) == 1 && toupper((unsigned char)v[0]) >= 'A' && toupper((unsigned char)v[0]) <= 'D')
        return 1 << (toupper((unsigned char)v[0]) - 'A');
    int n = atoi(v);
    if (n == 1 || n == 2 || n == 4 || n == 8) return n;
    return -1;
}

/* "2" / "IN_2" -> 1  (port number 1..4 -> index 0..3)                  */
static int parse_port(const char *v)
{
    if (!strncasecmp(v, "IN_", 3)) v += 3;
    int n = atoi(v);
    return (n >= 1 && n <= 4) ? n - 1 : -1;
}

static int apply(const char *key, const char *val)
{
    double d = atof(val);
    int i = atoi(val);
#define D(k, f) if (!strcasecmp(key, k)) { P.f = d; return 1; }
#define I(k, f) if (!strcasecmp(key, k)) { P.f = i; return 1; }
    D("wheel_diameter",  wheel_diameter)
    D("track_width",     track_width)
    D("robot_radius",    robot_radius)
    D("safety_margin",   safety_margin)
    D("start_heading",   start_heading)
    D("slow_zone",       slow_zone)
    D("brake_m",         brake_m)
    D("drive_kp",        drive_kp)
    D("drive_ki",        drive_ki)
    D("drive_kd",        drive_kd)
    D("corr_max",        corr_max)
    D("motor_trim",      motor_trim)
    D("turn_kp",         turn_kp)
    D("turn_ki",         turn_ki)
    D("turn_kd",         turn_kd)
    D("turn_tol",        turn_tol)
    D("turn_lag",        turn_lag)
    D("goal_tol",        goal_tol)
    I("drive_speed",     drive_speed)
    I("drive_speed_min", drive_speed_min)
    I("turn_speed_max",  turn_speed_max)
    I("turn_speed_min",  turn_speed_min)
    I("turn_settle_ms",  turn_settle_ms)
    I("turn_retries",    turn_retries)
#undef D
#undef I
    if (!strcasecmp(key, "drive_sign") || !strcasecmp(key, "gyro_sign")) {
        if (i != 1 && i != -1) return 0;
        if (key[0] == 'd' || key[0] == 'D') P.drive_sign = i; else P.gyro_sign = i;
        return 1;
    }
    if (!strcasecmp(key, "left_motor"))  { int m = parse_motor(val); if (m < 0) return 0; P.left_motor = m;  return 1; }
    if (!strcasecmp(key, "right_motor")) { int m = parse_motor(val); if (m < 0) return 0; P.right_motor = m; return 1; }
    if (!strcasecmp(key, "gyro_port"))   { int p = parse_port(val);  if (p < 0) return 0; P.gyro_port = p;   return 1; }
    return 0;
}

static char *trim_ws(char *s)
{
    while (isspace((unsigned char)*s)) s++;
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
    return s;
}

int params_load(const char *path, int *unknown)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[160];
    int applied = 0, bad = 0;
    while (fgets(line, sizeof line, f)) {
        char *hash = strchr(line, '#');
        if (hash) *hash = 0;
        char *s = trim_ws(line);
        if (!*s) continue;
        char key[64], val[64];
        if (sscanf(s, "%63s %63s", key, val) != 2) { bad++; continue; }
        char *eq = strchr(key, '=');       /* allow "key=value" too */
        if (eq) { *eq = 0; }
        if (apply(key, val)) applied++; else bad++;
    }
    fclose(f);
    if (unknown) *unknown = bad;
    return applied;
}

static const char *motor_name(int m)
{
    return m == 1 ? "A" : m == 2 ? "B" : m == 4 ? "C" : m == 8 ? "D" : "?";
}

void params_log(void)
{
    printf("params: wheel=%.4f track=%.3f radius=%.3f margin=%.3f L=%s R=%s gyro=%d dsign=%d gsign=%d "
           "hdg0=%.0f speed=%d/%d slow=%.3f brake=%.3f kp=%.2f ki=%.2f kd=%.2f corr=%.0f trim=%.2f "
           "turn=%d/%d tkp=%.2f tki=%.2f tkd=%.3f ttol=%.1f tlag=%.2f settle=%d retries=%d\n",
           P.wheel_diameter, P.track_width, P.robot_radius, P.safety_margin,
           motor_name(P.left_motor), motor_name(P.right_motor), P.gyro_port + 1,
           P.drive_sign, P.gyro_sign, P.start_heading, P.drive_speed, P.drive_speed_min,
           P.slow_zone, P.brake_m, P.drive_kp, P.drive_ki, P.drive_kd, P.corr_max, P.motor_trim,
           P.turn_speed_max, P.turn_speed_min, P.turn_kp, P.turn_ki, P.turn_kd, P.turn_tol, P.turn_lag,
           P.turn_settle_ms, P.turn_retries);
}

int course_load(const char *path, double obstacles[][2], int max,
                double start[2], double goal[2], double *heading_deg)
{
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    char line[160];
    int n = 0;
    while (fgets(line, sizeof line, f)) {
        char *hash = strchr(line, '#');
        if (hash) *hash = 0;
        char *s = trim_ws(line);
        if (!*s) continue;
        char key[32]; double a = 0, b = 0;
        int got = sscanf(s, "%31s %lf %lf", key, &a, &b);
        if (got < 2) continue;
        if (!strcasecmp(key, "obstacle") && got == 3) {
            if (n < max) { obstacles[n][0] = a; obstacles[n][1] = b; n++; }
        } else if (!strcasecmp(key, "start") && got == 3) {
            start[0] = a; start[1] = b;
        } else if (!strcasecmp(key, "goal") && got == 3) {
            goal[0] = a; goal[1] = b;
        } else if (!strcasecmp(key, "heading") && heading_deg) {
            *heading_deg = a;
        }
    }
    fclose(f);
    return n;
}
