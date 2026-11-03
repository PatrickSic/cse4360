/*
 * course.h - ACTIVE course, included by main.c.  Currently: Hand-out example course (figure in the PDF).
 * Same format as the hand-out.  To use it on the robot:
 *     cp courses/example.h courses/course.h   (then make)
 * or copy courses/example.txt to the SD card as myapps/course.txt (no rebuild).
 */
#ifndef COURSE_H
#define COURSE_H

#define MAX_OBSTACLES 25                 /* maximum number of obstacles */

int num_obstacles = 13;                  /* number of obstacles         */

double obstacle[MAX_OBSTACLES][2] =      /* obstacle locations          */
{{0.61, 2.745},{0.915, 2.745},{1.22, 2.745},{1.83, 1.22},
 {1.83, 1.525},{1.83, 1.83},{1.83, 2.135},{2.745, 0.305},
 {2.745, 0.61},{2.745, 0.915},{2.745, 2.745},{3.05, 2.745},
 {3.355, 2.745},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},
 {-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1},{-1, -1}};

double start[2] = {0.305, 1.219};        /* start location              */

double goal[2]  = {3.658, 1.829};        /* goal location               */

#endif /* COURSE_H */
