/*
 * test_planner.c - Run the Manhattan/wavefront planner on the LAPTOP.
 *
 *   make test_planner      (or: gcc -std=c99 -O2 -DHOST_BUILD -o test_planner
 *                                test_planner.c planner.c params.c -lm)
 *
 *   ./test_planner                        course from courses/course.h
 *   ./test_planner courses/snake2.txt     any course file
 *   ./test_planner [file] zigzag          show the WORST (most turns) path
 *   ./test_planner [file] dist            also print the distance map
 *   ./test_planner [file] summary         both paths, map of each, no dist map
 *   ./test_planner random 20              20 random courses, sanity check
 *
 * Map legend:  #  obstacle tile       .  too close to obstacle/wall (blocked)
 *              *  path                o  waypoint (robot turns 90 deg here)
 *              S  start      G  goal       (space) free node
 * One character = one grid node (half a tile).  Y grows upward.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* stand-ins for EV3 port macros so config.h compiles on the host */
#define OUT_A 1
#define OUT_B 2
#define OUT_C 4
#define OUT_D 8
#define IN_1 0
#define IN_2 1
#define IN_3 2
#define IN_4 3

#include "config.h"
#include "courses/course.h"
#include "planner.h"
#include "params.h"

static double g_heading = START_HEADING_DEG;

static void build_course(Course *c)
{
    course_init(c, WORKSPACE_W_M, WORKSPACE_H_M, OBSTACLE_HALF_M,
                ROBOT_RADIUS_M, SAFETY_MARGIN_M);
    c->num_obstacles = 0;
    for (int k = 0; k < num_obstacles && k < PLAN_MAX_OBSTACLES; k++) {
        if (obstacle[k][0] < 0) continue;
        c->obstacle[c->num_obstacles].x = obstacle[k][0];
        c->obstacle[c->num_obstacles].y = obstacle[k][1];
        c->num_obstacles++;
    }
    c->start.x = start[0]; c->start.y = start[1];
    c->goal.x = goal[0];   c->goal.y = goal[1];
}

static void draw(const Course *c, const Plan *p, int show_dist) { plan_draw(c, p, show_dist); }

static void print_plan(const Course *c, const Plan *p, int verbose)
{
    printf("Segments: %d, turns: %d, Manhattan length %.2f m\n",
           p->n, plan_count_turns(c, p, g_heading), p->length);
    if (!verbose) return;
    Point prev = c->start;
    double heading = g_heading;
    for (int k = 0; k < p->n; k++) {
        double dx = p->wp[k].x - prev.x, dy = p->wp[k].y - prev.y;
        double h = atan2(dy, dx) * 180.0 / 3.14159265358979323846;
        double turn = h - heading;
        while (turn > 180) turn -= 360;
        while (turn < -180) turn += 360;
        printf("  %2d: turn %+5.0f deg -> heading %4.0f, drive %.3f m  to (%.3f, %.3f)\n",
               k + 1, turn, h, fabs(dx) + fabs(dy), p->wp[k].x, p->wp[k].y);
        heading = h;
        prev = p->wp[k];
    }
}

static void random_course(Course *c, unsigned seed)
{
    srand(seed);
    c->num_obstacles = 0;
    int n = 5 + rand() % 12;
    for (int k = 0; k < n; k++) {
        c->obstacle[c->num_obstacles].x = (1 + rand() % 15) * TILE_M;
        c->obstacle[c->num_obstacles].y = (1 + rand() % 9) * TILE_M;
        c->num_obstacles++;
    }
    for (;;) {
        Point p = { (1 + rand() % 15) * TILE_M, (1 + rand() % 9) * TILE_M };
        if (course_clearance(c, p) > TILE_M) { c->start = p; break; }
    }
    for (;;) {
        Point p = { (1 + rand() % 15) * TILE_M, (1 + rand() % 9) * TILE_M };
        if (course_clearance(c, p) > TILE_M &&
            fabs(p.x - c->start.x) + fabs(p.y - c->start.y) > 1.0) { c->goal = p; break; }
    }
}

int main(int argc, char **argv)
{
    Course c;
    Plan plan;
    const char *file = NULL;
    int zigzag = 0, dist = 0, summary = 0;

    for (int a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "random")) {
            int count = a + 1 < argc ? atoi(argv[a + 1]) : 10, ok = 0;
            for (int t = 0; t < count; t++) {
                build_course(&c);
                random_course(&c, 1000 + t);
                int r = plan_path(&c, &plan);
                printf("\n=== random course %d: %s (%d segments, %.2f m)\n", t,
                       r ? "PATH FOUND" : "NO PATH", plan.n, plan.length);
                draw(&c, &plan, 0);
                ok += r;
            }
            printf("\n%d / %d random courses solved (random courses do not respect the\n"
                   "2-tile-corridor guarantee, so some 'NO PATH' are expected)\n", ok, count);
            return 0;
        }
        else if (!strcmp(argv[a], "zigzag"))  zigzag = 1;
        else if (!strcmp(argv[a], "dist"))    dist = 1;
        else if (!strcmp(argv[a], "summary")) summary = 1;
        else file = argv[a];
    }

    build_course(&c);
    if (file) {
        double st[2] = { c.start.x, c.start.y }, go[2] = { c.goal.x, c.goal.y };
        static double ob[PLAN_MAX_OBSTACLES][2];
        int n = course_load(file, ob, PLAN_MAX_OBSTACLES, st, go, &g_heading);
        if (n < 0) { fprintf(stderr, "cannot open %s\n", file); return 2; }
        c.num_obstacles = n;
        for (int k = 0; k < n; k++) { c.obstacle[k].x = ob[k][0]; c.obstacle[k].y = ob[k][1]; }
        c.start.x = st[0]; c.start.y = st[1]; c.goal.x = go[0]; c.goal.y = go[1];
    }
    printf("Course: %s, %d obstacles, start (%.3f, %.3f) goal (%.3f, %.3f), heading %.0f\n",
           file ? file : "courses/course.h", c.num_obstacles, c.start.x, c.start.y,
           c.goal.x, c.goal.y, g_heading);
    printf("Grid %.4f m, inflation: robot %.2f + margin %.2f = %.2f m\n",
           GRID_CELL_M, c.robot_radius, c.safety_margin, c.robot_radius + c.safety_margin);

    if (summary) {
        for (int z = 0; z < 2; z++) {
            printf("\n--- %s path ---\n", z ? "WORST (zigzag)" : "BEST");
            if (!plan_path_mode(&c, &plan, z ? PLAN_ZIGZAG : PLAN_BEST)) { printf("NO PATH\n"); continue; }
            draw(&c, &plan, 0);
            print_plan(&c, &plan, 0);
        }
        return 0;
    }

    if (!plan_path_mode(&c, &plan, zigzag ? PLAN_ZIGZAG : PLAN_BEST)) {
        printf("NO PATH\n"); draw(&c, &plan, 1); return 1;
    }
    printf("%s path:\n", zigzag ? "WORST (zigzag)" : "BEST");
    draw(&c, &plan, dist);
    printf("\n");
    print_plan(&c, &plan, 1);
    return 0;
}
