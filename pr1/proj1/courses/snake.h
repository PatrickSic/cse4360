/* slalom: Alternating bars force an S-shaped route: tests repeated left/right 90s.
 * Copy over courses/course.h (keep the name course.h) and rebuild to use it. */
#ifndef COURSE_H
#define COURSE_H

#define MAX_OBSTACLES 25                 /* maximum number of obstacles */

int num_obstacles = 18;                  /* number of obstacles         */

double obstacle[MAX_OBSTACLES][2] =      /* obstacle locations          */
{{1.22, 0.305}, {1.22, 0.61}, {1.22, 0.915}, {1.22, 1.22},
 {1.22, 1.525}, {1.22, 1.83}, {2.44, 1.22}, {2.44, 1.525},
 {2.44, 1.83}, {2.44, 2.135}, {2.44, 2.44}, {2.44, 2.745},
 {3.66, 0.305}, {3.66, 0.61}, {3.66, 0.915}, {3.66, 1.22},
 {3.66, 1.525}, {3.66, 1.83}, {-1,-1}, {-1,-1},
 {-1,-1}, {-1,-1}, {-1,-1}, {-1,-1},
 {-1,-1}};

double start[2] = {0.305, 1.525};        /* start location              */

double goal[2]  = {4.575, 1.525};        /* goal location               */

#endif /* COURSE_H */
