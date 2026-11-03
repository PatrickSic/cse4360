/*
 * diag.c - Diagnostic test suite.  Each test ends on a results screen and
 *          appends one line to  /media/card/myapps/diag_results.txt
 *          (plus diag.log via the .rbf), so the numbers can be sent back
 *          for tuning without copying them by hand.
 *
 * Menu:
 *   UP     T1 gyro drift     robot still 5 s            -> calibration quality
 *   RIGHT  T2 gyro scale     you turn it 90 deg by hand -> P.gyro_sign, mounting
 *   DOWN   T3 straight 2 m   gyro heading hold          -> WHEEL_DIAMETER, KP/KI, trim
 *   LEFT   T4 four 90 turns  turn controller            -> TURN_* gains, track width
 *   CENTER T5 motors         each motor alone, braking  -> motor asymmetry, slow zone
 *   EXIT   quit
 *
 * Build & copy:  make diag  ->  diag.elf + diag.rbf  into myapps/
 * Uses robot.c, so every test exercises exactly the code proj1 uses.
 */
#include <ev3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../config.h"
#include "../params.h"
#include "../robot.h"

#ifndef DIAG_RESULTS
#define DIAG_RESULTS "/media/card/myapps/diag_results.txt"
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define M_PER_DEG (M_PI * P.wheel_diameter / 360.0)

static int g_run = 0;

/* ------------------------------------------------------------------ */
static void record(const char *line)
{
    printf("%s\n", line);
    FILE *f = fopen(DIAG_RESULTS, "a");
    if (f) { fprintf(f, "%s\n", line); fclose(f); }
}

static void wait_release(void)
{
    while (ButtonIsDown(BTNCENTER) || ButtonIsDown(BTNUP) || ButtonIsDown(BTNDOWN) ||
           ButtonIsDown(BTNLEFT) || ButtonIsDown(BTNRIGHT) || ButtonIsDown(BTNEXIT))
        Wait(20);
    Wait(150);
}

static void wait_any(void)
{
    wait_release();
    while (ButtonWaitForAnyPress(100) == 0) { }
    wait_release();
}

static void countdown(const char *title, const char *l1, const char *l2)
{
    for (int s = 3; s > 0; s--) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(0), "%s", title);
        LcdTextf(1, 0, LcdRowToY(2), "%s", l1);
        LcdTextf(1, 0, LcdRowToY(3), "%s", l2);
        LcdTextf(1, 0, LcdRowToY(6), "starting in %d", s);
        PlayTone(660, 80);
        Wait(1000);
    }
}

static void header(const char *name)
{
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "%s  run %d", name, g_run);
}

/* ------------------------------------------------------------------ */
/* T1: gyro drift while standing still                                  */
static void t1_gyro_drift(void)
{
    countdown("T1 gyro drift", "Do NOT touch the", "robot for 6 s");
    int first = robot_gyro_raw(), mn = first, mx = first, last = first;
    unsigned long t0 = TimerGetMS();
    while (TimerGetMS() - t0 < 5000) {
        last = robot_gyro_raw();
        if (last < mn) mn = last;
        if (last > mx) mx = last;
        Wait(50);
    }
    double drift = (last - first) / 5.0;
    char line[128];
    snprintf(line, sizeof line, "T1 run%d drift=%.2f deg/s span=%d start=%d end=%d",
             g_run, drift, mx - mn, first, last);
    record(line);

    header("T1 gyro drift");
    LcdTextf(1, 0, LcdRowToY(2), "drift %.2f deg/s", drift);
    LcdTextf(1, 0, LcdRowToY(3), "span  %d deg", mx - mn);
    LcdTextf(1, 0, LcdRowToY(5), fabs(drift) < 0.15 ? "GOOD (<0.15)" : "BAD: replug gyro,");
    if (fabs(drift) >= 0.15) LcdTextf(1, 0, LcdRowToY(6), "keep still, restart");
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    wait_any();
}

/* T2: gyro sign and scale, manual 90 degree turn                        */
static void t2_gyro_scale(void)
{
    robot_set_pose(0, 0, 0);
    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "T2 gyro scale");
    LcdTextf(1, 0, LcdRowToY(2), "Turn robot LEFT by");
    LcdTextf(1, 0, LcdRowToY(3), "EXACTLY 90 deg by");
    LcdTextf(1, 0, LcdRowToY(4), "hand (use a tile");
    LcdTextf(1, 0, LcdRowToY(5), "edge), then CENTER");
    wait_release();
    double h = 0;
    for (;;) {
        h = robot_heading();
        LcdTextf(1, 0, LcdRowToY(7), "heading %6.1f   ", h);
        if (ButtonIsDown(BTNCENTER)) break;
        if (ButtonIsDown(BTNEXIT)) { wait_release(); return; }
        Wait(50);
    }
    wait_release();
    const char *verdict = (h > 80 && h < 100) ? "GOOD" :
                          (h < -80 && h > -100) ? "FLIP P.gyro_sign" :
                          (fabs(h) < 45) ? "sensor tilted?" : "scale off?";
    char line[128];
    snprintf(line, sizeof line, "T2 run%d manual_left_90 heading=%.1f raw=%d verdict=%s",
             g_run, h, robot_gyro_raw(), verdict);
    record(line);

    header("T2 gyro scale");
    LcdTextf(1, 0, LcdRowToY(2), "heading %.1f", h);
    LcdTextf(1, 0, LcdRowToY(3), "expect  +90");
    LcdTextf(1, 0, LcdRowToY(5), "%s", verdict);
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    wait_any();
}

/* T3: straight 2 m with heading hold                                   */
static void t3_straight(void)
{
    countdown("T3 straight 2 m", "needs 2.5 m clear,", "mark the start!");
    robot_set_pose(0, 0, 0);
    bool ok = robot_drive(2.0, 0.0);
    DriveStats d = robot_last_drive();
    Wait(500);
    double settled = robot_heading();
    char line[200];
    snprintf(line, sizeof line,
             "T3 run%d ok=%d encL=%d encR=%d dist_enc=%.3f maxerr=%.1f finalerr=%.1f settled_hdg=%.1f "
             "trim_before=%.1f trim_after=%.1f time=%lu ms KP=%.1f KI=%.1f speed=%d wheel=%.4f",
             g_run, ok, d.enc_left, d.enc_right, d.dist_enc, d.max_abs_err, d.final_err, settled,
             d.trim_before, d.trim_after, d.time_ms, (double)P.drive_kp, (double)P.drive_ki,
             P.drive_speed, (double)P.wheel_diameter);
    record(line);

    header(ok ? "T3 straight" : "T3 ABORTED");
    LcdTextf(1, 0, LcdRowToY(1), "encL %d encR %d", d.enc_left, d.enc_right);
    LcdTextf(1, 0, LcdRowToY(2), "maxerr %.1f fin %.1f", d.max_abs_err, d.final_err);
    LcdTextf(1, 0, LcdRowToY(3), "trim %.1f -> %.1f", d.trim_before, d.trim_after);
    LcdTextf(1, 0, LcdRowToY(4), "time %.1f s", d.time_ms / 1000.0);
    LcdTextf(1, 0, LcdRowToY(6), "MEASURE real dist");
    LcdTextf(1, 0, LcdRowToY(7), "(cmd 2.000 m) and");
    LcdTextf(1, 0, LcdRowToY(8), "sideways offset, cm");
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    wait_any();
}

/* T4: four 90 degree turns in place                                     */
static void t4_turns(void)
{
    countdown("T4 four turns", "robot turns left", "4 x 90 in place");
    robot_set_pose(0, 0, 0);
    double sum_err = 0, max_err = 0, track_sum = 0;
    int attempts = 0; unsigned long time_sum = 0; int track_n = 0;
    char line[160];
    for (int k = 1; k <= 4; k++) {
        double before = robot_heading();
        bool ok = robot_turn_to(90.0 * k);
        TurnStats t = robot_last_turn();
        Wait(700);
        double turned = robot_heading() - before;
        while (turned < -180) turned += 360;
        while (turned <= 0) turned += 360;
        double track = 0;
        if (turned > 10) {
            track = (abs(t.enc_left) + abs(t.enc_right)) * M_PER_DEG / (turned * M_PI / 180.0);
            track_sum += track; track_n++;
        }
        sum_err += fabs(t.final_err);
        if (fabs(t.final_err) > max_err) max_err = fabs(t.final_err);
        attempts += t.attempts; time_sum += t.time_ms;
        snprintf(line, sizeof line, "T4 run%d turn%d ok=%d err=%.1f attempts=%d time=%lu encL=%d encR=%d track=%.3f",
                 g_run, k, ok, t.final_err, t.attempts, t.time_ms, t.enc_left, t.enc_right, track);
        record(line);
        if (!ok) break;
    }
    double final_h = robot_heading();
    double track_avg = track_n ? track_sum / track_n : 0;
    snprintf(line, sizeof line, "T4 run%d summary final_hdg=%.1f avg_err=%.2f max_err=%.1f attempts=%d avg_time=%lu track_est=%.3f",
             g_run, final_h, sum_err / 4.0, max_err, attempts, time_sum / 4, track_avg);
    record(line);

    header("T4 turns");
    LcdTextf(1, 0, LcdRowToY(1), "final hdg %.1f (0?)", final_h);
    LcdTextf(1, 0, LcdRowToY(2), "avg err %.2f max %.1f", sum_err / 4.0, max_err);
    LcdTextf(1, 0, LcdRowToY(3), "retries %d  %.1f s/turn", attempts, time_sum / 4000.0);
    LcdTextf(1, 0, LcdRowToY(4), "track est %.3f m", track_avg);
    LcdTextf(1, 0, LcdRowToY(6), "Is it facing the");
    LcdTextf(1, 0, LcdRowToY(7), "same way as start?");
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    wait_any();
}

/* T5: motors one at a time, then braking distance                      */
static void t5_motors(void)
{
    countdown("T5 motors", "lift robot or give", "it 1 m of space");
    int l0, r0, dl[2], dr[2];
    for (int m = 0; m < 2; m++) {
        l0 = robot_enc_left(); r0 = robot_enc_right();
        if (m == 0) robot_motors(30, 0); else robot_motors(0, 30);
        Wait(1000);
        robot_stop();
        Wait(400);
        dl[m] = robot_enc_left() - l0; dr[m] = robot_enc_right() - r0;
    }
    /* braking: both at P.drive_speed for 1.2 s, cut, see how far it rolls */
    robot_set_pose(0, 0, 0);
    l0 = robot_enc_left(); r0 = robot_enc_right();
    robot_motors(P.drive_speed, P.drive_speed);
    Wait(1200);
    int lc = robot_enc_left(), rc = robot_enc_right();
    robot_stop();
    Wait(600);
    int lb = robot_enc_left() - lc, rb = robot_enc_right() - rc;
    double brake_mm = (lb + rb) * 0.5 * M_PER_DEG * 1000.0;
    double hdg = robot_heading();
    double dist = ((lc - l0) + (rc - r0)) * 0.5 * M_PER_DEG;
    double mismatch = dist > 0.05 ? (hdg * M_PI / 180.0) * P.track_width / dist : 0;
    double trim_sugg = P.motor_trim - 0.5 * mismatch * P.drive_speed;

    char line[200];
    snprintf(line, sizeof line,
             "T5 run%d leftonly_L=%d leftonly_R=%d rightonly_L=%d rightonly_R=%d brake_deg_L=%d brake_deg_R=%d brake_mm=%.0f "
             "openloop_hdg=%.1f openloop_dist=%.3f trim_sugg=%.1f",
             g_run, dl[0], dr[0], dl[1], dr[1], lb, rb, brake_mm, hdg, dist, trim_sugg);
    record(line);

    header("T5 motors");
    LcdTextf(1, 0, LcdRowToY(1), "L alone %d  R alone %d", dl[0], dr[1]);
    LcdTextf(1, 0, LcdRowToY(2), "(other: %d / %d)", dr[0], dl[1]);
    LcdTextf(1, 0, LcdRowToY(3), "brake %d/%d deg", lb, rb);
    LcdTextf(1, 0, LcdRowToY(4), "      = %.0f mm", brake_mm);
    LcdTextf(1, 0, LcdRowToY(5), "openloop hdg %.1f", hdg);
    LcdTextf(1, 0, LcdRowToY(6), "trim sugg %.1f", trim_sugg);
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    wait_any();
}

/* ------------------------------------------------------------------ */
int main(void)
{
    robot_log_to_file("/media/card/myapps/diag.log");
    params_defaults();
    int bad = 0, keys = params_load("/media/card/myapps/proj1.cfg", &bad);
    printf("cfg: %d keys, %d bad\n", keys, bad);
    params_log();
    if (!robot_init()) { robot_shutdown(); return 1; }

    char line[160];
    snprintf(line, sizeof line, "=== diag start: L=%d R=%d gyro=%d sign=%d drivesign=%d wheel=%.4f KP=%.1f KI=%.1f trim=%.1f spd=%d turn=%d/%d kp=%.2f tol=%.1f",
             P.left_motor, P.right_motor, P.gyro_port + 1, P.gyro_sign, P.drive_sign, (double)P.wheel_diameter,
             (double)P.drive_kp, (double)P.drive_ki, (double)P.motor_trim, P.drive_speed,
             P.turn_speed_max, P.turn_speed_min, (double)P.turn_kp, (double)P.turn_tol);
    record(line);

    for (;;) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(0), "DIAG  hdg %.1f", robot_heading());
        LcdTextf(1, 0, LcdRowToY(2), "UP     T1 gyro drift");
        LcdTextf(1, 0, LcdRowToY(3), "RIGHT  T2 gyro scale");
        LcdTextf(1, 0, LcdRowToY(4), "DOWN   T3 straight 2m");
        LcdTextf(1, 0, LcdRowToY(5), "LEFT   T4 4x90 turns");
        LcdTextf(1, 0, LcdRowToY(6), "CENTER T5 motors");
        LcdTextf(1, 0, LcdRowToY(7), "EXIT   quit");
        LcdTextf(1, 0, LcdRowToY(9), "results -> myapps/");
        LcdTextf(1, 0, LcdRowToY(10), "diag_results.txt");

        if      (ButtonIsDown(BTNEXIT))   { wait_release(); break; }
        else if (ButtonIsDown(BTNUP))     { wait_release(); g_run++; t1_gyro_drift(); }
        else if (ButtonIsDown(BTNRIGHT))  { wait_release(); g_run++; t2_gyro_scale(); }
        else if (ButtonIsDown(BTNDOWN))   { wait_release(); g_run++; t3_straight(); }
        else if (ButtonIsDown(BTNLEFT))   { wait_release(); g_run++; t4_turns(); }
        else if (ButtonIsDown(BTNCENTER)) { wait_release(); g_run++; t5_motors(); }
        Wait(50);
    }
    record("=== diag end");
    robot_shutdown();
    return 0;
}
