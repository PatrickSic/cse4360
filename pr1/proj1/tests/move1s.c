/*
 * move1s.c - smoke test: drive forward for 1 second, show what happened.
 *
 * Uses the ports from config.h, so it doubles as a port/polarity check:
 *   - robot should move FORWARD (wheels leading, caster trailing)
 *     -> if it goes backwards, flip DRIVE_SIGN in config.h
 *     -> if it spins, LEFT_MOTOR / RIGHT_MOTOR are not both plugged in
 *   - encL / encR should both be positive and roughly equal (~300-400 deg)
 *   - gyro should stay near 0 if the robot went straight
 *
 * Build & copy:  make move1s   ->  move1s.elf + move1s.rbf  into myapps/
 */
#include <ev3.h>
#include <stdio.h>
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

int main(void)
{
    log_to_file("/media/card/myapps/move1s.log");
    InitEV3();

    SensorHandler *ports[4] = { NULL, NULL, NULL, NULL };
    ports[GYRO_PORT] = EV3Gyro;
    bool gyro_ok = SetAllSensors(ports[0], ports[1], ports[2], ports[3]);
    Wait(500);

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(1), "MOVE TEST");
    LcdTextf(1, 0, LcdRowToY(2), "gyro port %d: %s", GYRO_PORT + 1, gyro_ok ? "ok" : "FAILED");
    LcdTextf(1, 0, LcdRowToY(4), "forward 1 s in 2 s...");
    Wait(2000);

    int g0 = gyro_ok ? ReadEV3GyroSensorAngle(GYRO_PORT, EV3GyroNormalAngle) : 0;
    ResetRotationCount(LEFT_MOTOR | RIGHT_MOTOR);

    OnFwdRegEx(LEFT_MOTOR,  30 * DRIVE_SIGN, OUT_REGMODE_SPEED, RESET_NONE);
    OnFwdRegEx(RIGHT_MOTOR, 30 * DRIVE_SIGN, OUT_REGMODE_SPEED, RESET_NONE);
    Wait(1000);
    Off(LEFT_MOTOR | RIGHT_MOTOR);
    Wait(300);

    int encL = MotorRotationCount(LEFT_MOTOR)  * DRIVE_SIGN;
    int encR = MotorRotationCount(RIGHT_MOTOR) * DRIVE_SIGN;
    int g1 = gyro_ok ? ReadEV3GyroSensorAngle(GYRO_PORT, EV3GyroNormalAngle) : 0;

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(1), "done");
    LcdTextf(1, 0, LcdRowToY(3), "encL = %d deg", encL);
    LcdTextf(1, 0, LcdRowToY(4), "encR = %d deg", encR);
    LcdTextf(1, 0, LcdRowToY(5), "gyro = %d deg", g1 - g0);
    LcdTextf(1, 0, LcdRowToY(7), "want: both + and");
    LcdTextf(1, 0, LcdRowToY(8), "equal, gyro ~ 0");
    LcdTextf(1, 0, LcdRowToY(10), "any button = exit");
    printf("move1s: encL=%d encR=%d gyro=%d gyro_ok=%d\n", encL, encR, g1 - g0, gyro_ok);

    while (ButtonWaitForAnyPress(100) == 0) { }

    FreeEV3();
    return 0;
}
