/* open_diagonal: No obstacles, goal diagonally across. BEST = 1 turn, ZIGZAG = staircase of ~20 turns.
 * Copy over courses/course.h (keep the name course.h) and rebuild to use it. */
#ifndef COURSE_H
#define COURSE_H

#define MAX_OBSTACLES 25                 /* maximum number of obstacles */

int num_obstacles = 0;                  /* number of obstacles         */

double obstacle[MAX_OBSTACLES][2] =      /* obstacle locations          */
{{-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}};

double start[2] = {0.61, 0.61};        /* start location              */

double goal[2]  = {3.965, 2.44};        /* goal location               */

#endif /* COURSE_H */
