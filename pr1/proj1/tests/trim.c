/*
 * trim.c - measure how much the robot pulls to one side when both motors
 *          get the SAME speed command, and suggest a MOTOR_TRIM value.
 *
 * Method: drive open-loop (no gyro correction) for TRIM_RUN_MS at
 * DRIVE_SPEED, then read the gyro heading change and both encoders.
 *
 *   curvature      = heading change / distance driven
 *   speed mismatch = curvature * track width          (differential drive)
 *   trim           = -(speed mismatch / 2) * DRIVE_SPEED
 *
 * Trim is ADDED to the right motor and SUBTRACTED from the left one, so a
 * robot that drifted LEFT (heading +) gets a negative trim: right slower,
 * left faster.
 *
 * Run it 2-3 times on the lab floor and average.  Put the result in
 * config.h as MOTOR_TRIM, rebuild proj1.  Re-run to verify: the heading
 * change should now be within about +-2 degrees.
 *
 * Build & copy:  make trim  ->  trim.elf + trim.rbf  into myapps/
 */
#include <ev3.h>
#include <stdio.h>
#include <math.h>
#include "../config.h"
#include <fcntl.h>
#include <unistd.h>
static void log_to_file(const char *path)
{
    int fd = open(path, O_WRONLY | O_CREAT | O_APPEND, 0644);
    if (fd < 0) return;
    dup2(fd, 1); dup2(fd, 2); close(fd);
    setvbuf(stdout, NULL, _IOLBF, 0);
}

#define TRIM_RUN_MS 2500

int main(void)
{
    log_to_file("/media/card/myapps/trim.log");
    InitEV3();

    SensorHandler *ports[4] = { NULL, NULL, NULL, NULL };
    ports[GYRO_PORT] = EV3Gyro;
    if (!SetAllSensors(ports[0], ports[1], ports[2], ports[3])) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(1), "gyro not on port %d", GYRO_PORT + 1);
        Wait(3000);
        FreeEV3();
        return 1;
    }
    Wait(500);

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(1), "TRIM TEST");
    LcdTextf(1, 0, LcdRowToY(2), "current trim %.1f", (double)MOTOR_TRIM);
    LcdTextf(1, 0, LcdRowToY(4), "needs ~1.5 m of floor");
    LcdTextf(1, 0, LcdRowToY(5), "starts in 3 s, hands off");
    Wait(3000);

    /* open-loop run, same command to both motors (plus the current trim) */
    int g0 = ReadEV3GyroSensorAngle(GYRO_PORT, EV3GyroNormalAngle);
    ResetRotationCount(LEFT_MOTOR | RIGHT_MOTOR);
    int sl = (int)(DRIVE_SPEED - MOTOR_TRIM + 0.5) * DRIVE_SIGN;
    int sr = (int)(DRIVE_SPEED + MOTOR_TRIM + 0.5) * DRIVE_SIGN;
    OnFwdRegEx(LEFT_MOTOR,  (int8_t)sl, OUT_REGMODE_SPEED, RESET_NONE);
    OnFwdRegEx(RIGHT_MOTOR, (int8_t)sr, OUT_REGMODE_SPEED, RESET_NONE);
    Wait(TRIM_RUN_MS);
    Off(LEFT_MOTOR | RIGHT_MOTOR);
    Wait(400);

    int encL = MotorRotationCount(LEFT_MOTOR)  * DRIVE_SIGN;
    int encR = MotorRotationCount(RIGHT_MOTOR) * DRIVE_SIGN;
    int g1 = ReadEV3GyroSensorAngle(GYRO_PORT, EV3GyroNormalAngle);

    double dtheta_deg = GYRO_SIGN * (double)(g1 - g0);        /* + = turned left */
    double dist = (encL + encR) * 0.5 * M_PI * WHEEL_DIAMETER_M / 360.0;
    double mismatch = (dist > 0.05) ? (dtheta_deg * M_PI / 180.0) * TRACK_WIDTH_M / dist : 0.0;
    double suggest = MOTOR_TRIM - 0.5 * mismatch * DRIVE_SPEED;

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "TRIM RESULT");
    LcdTextf(1, 0, LcdRowToY(2), "encL %d  encR %d", encL, encR);
    LcdTextf(1, 0, LcdRowToY(3), "dist %.2f m", dist);
    LcdTextf(1, 0, LcdRowToY(4), "heading %+.1f deg", dtheta_deg);
    LcdTextf(1, 0, LcdRowToY(5), "(%s)", dtheta_deg > 1 ? "drifts LEFT" : dtheta_deg < -1 ? "drifts RIGHT" : "straight");
    LcdTextf(1, 0, LcdRowToY(7), "MOTOR_TRIM = %.1f", suggest);
    LcdTextf(1, 0, LcdRowToY(8), "(now %.1f)", (double)MOTOR_TRIM);
    LcdTextf(1, 0, LcdRowToY(10), "any button = exit");
    printf("trim test: encL=%d encR=%d dist=%.3f dtheta=%.1f mismatch=%.4f suggest=%.2f\n",
           encL, encR, dist, dtheta_deg, mismatch, suggest);

    while (ButtonWaitForAnyPress(100) == 0) { }
    FreeEV3();
    return 0;
}
