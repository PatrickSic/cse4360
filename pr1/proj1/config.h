/*config.h*/
#ifndef CONFIG_H
#define CONFIG_H

/*planner workspace*/
#define TILE_M            0.305
#define WORKSPACE_W_M     (16 * TILE_M)     /* 4.88 m, along X */
#define WORKSPACE_H_M     (10 * TILE_M)     /* 3.05 m, along Y */
#define OBSTACLE_HALF_M   (TILE_M / 2.0)    /* obstacle square is 1 tile */
#define GOAL_RADIUS_M     TILE_M            

/*robot geometry*/
/*If the robot wheels are ever changed, this needs to be changed as well (alongside wheel diameter)*/
#define ROBOT_RADIUS_M    0.18

/* padding for when the robot is deciding if a grid is drivable */
#define SAFETY_MARGIN_M   0.04
#define GRID_CELL_M       (TILE_M / 2.0)
#define WHEEL_DIAMETER_M  0.056

/*ports*/
#define LEFT_MOTOR        OUT_B
#define RIGHT_MOTOR       OUT_C
#define GYRO_PORT         IN_2        /* IN_1 .. IN_4 */

/*polarity*/
#define DRIVE_SIGN        (+1)
#define GYRO_SIGN         (-1)
#define START_HEADING_DEG 0.0

/*motion & pid*/
#define DRIVE_SPEED       35    
#define DRIVE_SPEED_MIN   12    
#define DRIVE_SLOW_ZONE_M 0.12  /* start slowing this far before the target */
#define DRIVE_BRAKE_M     0.015 /* braking roll */
#define DRIVE_KP          2.0   /* heading hold gain (for example the gain would be lower if the robot was wiggling*/
#define DRIVE_KI          1.0   /* integral gain */
#define DRIVE_KD          0.0   /* derivative gain */
#define DRIVE_CORR_MAX    15    

#define MOTOR_TRIM        0.0
#define TRIM_MAX          8.0

/* distance between wheels */
#define TRACK_WIDTH_M     0.108

#define TURN_SPEED_MAX    25
#define TURN_SPEED_MIN    5     /* lower if it overshoots, higher if it creeps*/
#define TURN_KP           0.6   /* turn speed per degree of remaining error */
#define TURN_KI           0.0   /* this value is usually zero*/
#define TURN_KD           0.0  
#define TURN_TOL_DEG      2.0   /* accept heading only when within this margin of degrees*/
#define TURN_LAG_S        0.05  /* seconds of gyro lag compensated*/
#define TURN_SETTLE_MS    250
#define TURN_MAX_RETRIES  4

#define LOOP_MS           10    /* control loop period */
#define GOAL_TOL_M        0.03 

#endif /* CONFIG_H */
