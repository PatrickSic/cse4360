/*  1. Break the workspace into an n by n grid with
    2. Do a bfs expansion starting from the goal to over every possible "free" node, each node recieves its manhattan distance to the goal.
    Then, path from the start node repeatedly 
    3. Start at the start node and step into one of the 4 neighbors with the least amount of distance (gradient descent in class)
    Ties are solved by comparing direction, then amount of turns, and then finally if there is higher "clearance" (which is to say, that the option avoids an obstacle)
    I tried to keep the robot to only 90 degree turns for simplicity sakes.
    4. Consecutive steps in the same direction are merged
 */
#ifndef PLANNER_H
#define PLANNER_H

#define PLAN_MAX_OBSTACLES 32
#define PLAN_MAX_WAYPOINTS 128

#ifndef GRID_CELL_M
#define GRID_CELL_M 0.1525
#endif

typedef struct {
    double x, y;
} Point;

typedef struct {
    double width, height;          /* workspace size (m) */
    double obstacle_half;          /* half side of an obstacle square (m) */
    int    num_obstacles;
    Point  obstacle[PLAN_MAX_OBSTACLES];
    Point  start, goal;

    double robot_radius;           /* m */
    double safety_margin;          /* m */
} Course;

typedef struct {
    int   n;                        /* number of waypoints (excl. start) */
    Point wp[PLAN_MAX_WAYPOINTS];   /* wp[n-1] == goal on success */
    double length;                  /* total path length (m) */
} Plan;

void course_init(Course *c, double width, double height, double obstacle_half,
                 double robot_radius, double safety_margin);

/* Distance from point p to nearest obstacle square edge
 * Negative or zero if inside an obstacle or outside the grid*/
double course_clearance(const Course *c, Point p);
int plan_path(const Course *c, Plan *out);

/*
There are only two path modes, plan_man is the standard manhattan distance application, plan_zigzag... zigzags, i felt seeing the robot take the worst possible path and still be accurate would be important though
*/
#define PLAN_MAN   0
#define PLAN_ZIGZAG 1
int plan_path_mode(const Course *c, Plan *out, int mode);

/*the number of 90 degree turns in a plan*/
int plan_count_turns(const Course *c, const Plan *p, double start_heading_deg);

/* Print an ASCII map of the course
 *   #  obstacle   .  blocked (too close)   *  path   o  waypoint
 *   S  start      G  goal                  show_dist: also the distance map */
void plan_draw(const Course *c, const Plan *p, int show_dist);
int plan_grid_cols(const Course *c);          
int plan_grid_rows(const Course *c);          
int plan_node_blocked(const Course *c, int i, int j);
int plan_node_distance(int i, int j);         /*steps to goal*/

#endif /* PLANNER_H */
