/*
 * main.c - 
 *      This file is to take a given course and navigate it
 *         The course comes from courses/course.h by default
 *
 * Flow:  menu -> plan path (planner.c) -> wait for CENTER button
 *        -> zero gyro at P.start_heading -> drive waypoint by waypoint
 *          The robot is to plan a path -> recalibrate the gyro -> then drive waypoint to waypoint once ready
 *
 * Buttons in the menu:
 *   CENTER : run the manhattan path
 *   LEFT   : run the zigzag path
 *   RIGHT  : show the best plan     UP : gyro sign test
 *   DOWN   : drive 2.000 m test   EXIT : quit
 */
#ifdef HOST_BUILD
#include "host/ev3_stub.h"
#else
#include <ev3.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "config.h"
#include "courses/course.h"  
#include "planner.h"
#include "params.h"
#include "robot.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef LOG_PATH
#define LOG_PATH "/media/card/myapps/proj1.log"
#endif
#ifndef CFG_PATH
#define CFG_PATH "/media/card/myapps/proj1.cfg"
#endif
#ifndef COURSE_PATH
#define COURSE_PATH "/media/card/myapps/course.txt"
#endif
static int g_cfg_keys = -1, g_cfg_bad = 0, g_course_file = -1;
#ifdef HOST_BUILD
static const char *g_course_path = COURSE_PATH, *g_cfg_path = CFG_PATH;
#undef  COURSE_PATH
#undef  CFG_PATH
#define COURSE_PATH g_course_path
#define CFG_PATH    g_cfg_path
#endif


static Course g_course;
static Plan   g_plan;          
static Plan   g_plan_worst;    
static int    g_plan_ok = 0, g_worst_ok = 0;

static void build_course(void)
{
    course_init(&g_course, WORKSPACE_W_M, WORKSPACE_H_M, OBSTACLE_HALF_M,
                P.robot_radius, P.safety_margin);
    /* course.txt on the card overrides the compiled-in course.h */
    g_course_file = course_load(COURSE_PATH, obstacle, MAX_OBSTACLES, start, goal, &P.start_heading);
    if (g_course_file >= 0) num_obstacles = g_course_file;
    g_course.num_obstacles = 0;
    for (int k = 0; k < num_obstacles && k < PLAN_MAX_OBSTACLES; k++) {
        if (obstacle[k][0] < 0) continue;            /* {-1,-1} = unused */
        g_course.obstacle[g_course.num_obstacles].x = obstacle[k][0];
        g_course.obstacle[g_course.num_obstacles].y = obstacle[k][1];
        g_course.num_obstacles++;
    }
    g_course.start.x = start[0]; g_course.start.y = start[1];
    g_course.goal.x  = goal[0];  g_course.goal.y  = goal[1];
}

static void wait_release(void)
{
    while (ButtonIsDown(BTNCENTER) || ButtonIsDown(BTNUP) || ButtonIsDown(BTNDOWN) ||
           ButtonIsDown(BTNLEFT) || ButtonIsDown(BTNRIGHT) || ButtonIsDown(BTNEXIT))
        Wait(20);
    Wait(100);
}

static void show_plan(const Plan *plan, int ok, const char *title)
{
    LcdClean();
    if (!ok) {
        LcdTextf(1, 0, LcdRowToY(1), "NO PATH FOUND!");
        LcdTextf(1, 0, LcdRowToY(2), "check course.h /");
        LcdTextf(1, 0, LcdRowToY(3), "ROBOT_RADIUS_M");
    } else {
        LcdTextf(1, 0, LcdRowToY(0), "%s %d segs %.2fm", title, plan->n, plan->length);
        Point prev = g_course.start;
        for (int k = 0; k < plan->n && k < 9; k++) {
            double dx = plan->wp[k].x - prev.x, dy = plan->wp[k].y - prev.y;
            LcdTextf(1, 0, LcdRowToY(k + 1), "%d:%4.0f deg %.2fm", k + 1,
                     atan2(dy, dx) * 180.0 / M_PI, sqrt(dx * dx + dy * dy));
            prev = plan->wp[k];
        }
        if (plan->n > 9) LcdTextf(1, 0, LcdRowToY(10), "... +%d more", plan->n - 9);
    }
    LcdTextf(1, 0, LcdRowToY(11), "any button = back");
    wait_release();
    while (ButtonWaitForAnyPress(100) == 0) { }
    wait_release();
}

static void log_one(const char *name, const Plan *p, int ok)
{
    if (!ok) { printf("%s: NO PATH\n", name); return; }
    printf("%s: %d waypoints, %d turns, %.3f m\n", name, p->n,
           plan_count_turns(&g_course, p, P.start_heading), p->length);
    for (int k = 0; k < p->n; k++)
        printf("  wp %d: (%.3f, %.3f)\n", k + 1, p->wp[k].x, p->wp[k].y);
}

static void log_plan(void)
{
    printf("course: %d obstacles, start (%.3f,%.3f) goal (%.3f,%.3f)\n",
           g_course.num_obstacles, g_course.start.x, g_course.start.y,
           g_course.goal.x, g_course.goal.y);
    log_one("plan", &g_plan, g_plan_ok);
    log_one("worst", &g_plan_worst, g_worst_ok);
}

static void run_course(const Plan *plan, int ok, const char *title)
{
    if (!ok) { show_plan(plan, ok, title); return; }

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), "%s: %d segs", title, plan->n);
    LcdTextf(1, 0, LcdRowToY(2), "Place robot on START");
    LcdTextf(1, 0, LcdRowToY(3), "facing %.0f deg (+X=0)", P.start_heading);
    LcdTextf(1, 0, LcdRowToY(4), "centre of wheels on");
    LcdTextf(1, 0, LcdRowToY(5), "the tile corner.");
    LcdTextf(1, 0, LcdRowToY(7), "CENTER = GO");
    LcdTextf(1, 0, LcdRowToY(8), "EXIT   = back");
    wait_release();
    for (;;) {
        if (ButtonIsDown(BTNEXIT)) { wait_release(); return; }
        if (ButtonIsDown(BTNCENTER)) break;
        Wait(20);
    }
    wait_release();

    /* 2 s to take hands off; gyro offset is taken while it stands still */
    for (int s = 3; s > 0; s--) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(2), "Starting in %d", s);
        LcdTextf(1, 0, LcdRowToY(3), "hands off!");
        PlayTone(880, 100);
        Wait(1000);
    }
    robot_set_pose(g_course.start.x, g_course.start.y, P.start_heading);
    printf("run: start pose (%.3f, %.3f, %.1f)\n", g_course.start.x, g_course.start.y, P.start_heading);

    bool done = true;
    for (int k = 0; k < plan->n && done; k++) {
        Pose p = robot_get_pose();
        printf("seg %d: from (%.3f,%.3f,%.1f) to (%.3f,%.3f)\n", k + 1,
               p.x, p.y, p.heading, plan->wp[k].x, plan->wp[k].y);
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(0), "%s seg %d/%d", title, k + 1, plan->n);
        done = robot_goto(plan->wp[k].x, plan->wp[k].y);
    }
    robot_stop();

    Pose p = robot_get_pose();
    double dx = g_course.goal.x - p.x, dy = g_course.goal.y - p.y;
    printf("run %s %s: final pose (%.3f,%.3f,%.1f) dist to goal %.3f\n", title,
           done ? "finished" : "ABORTED", p.x, p.y, p.heading, sqrt(dx * dx + dy * dy));

    LcdClean();
    LcdTextf(1, 0, LcdRowToY(0), done ? "GOAL REACHED" : "ABORTED");
    LcdTextf(1, 0, LcdRowToY(2), "x=%.3f y=%.3f", p.x, p.y);
    LcdTextf(1, 0, LcdRowToY(3), "hdg=%.1f", p.heading);
    LcdTextf(1, 0, LcdRowToY(10), "any button = menu");
    if (done) { PlayTone(1047, 150); Wait(170); PlayTone(1319, 150); Wait(170); PlayTone(1568, 300); }
    else      { PlayTone(220, 600); }
    wait_release();
    while (ButtonWaitForAnyPress(100) == 0) { }
    wait_release();
}


int main(int argc, char **argv)
{
    (void)argc; (void)argv;
#ifdef HOST_BUILD
    /*when built not on a brick*/
    int host_worst = 0, host_dist = 0;
    g_course_path = "courses/course.txt-not-present";   
    g_cfg_path = "proj1.cfg";
    for (int a = 1; a < argc; a++) {
        if (!strcmp(argv[a], "worst")) host_worst = 1;
        else if (!strcmp(argv[a], "dist")) host_dist = 1;
        else if (strstr(argv[a], ".cfg")) g_cfg_path = argv[a];
        else g_course_path = argv[a];
    }
#endif
    robot_log_to_file(LOG_PATH);
    params_defaults();
    g_cfg_keys = params_load(CFG_PATH, &g_cfg_bad);
    params_log();

    build_course();
    g_plan_ok  = plan_path_mode(&g_course, &g_plan, PLAN_MAN);
    g_worst_ok = plan_path_mode(&g_course, &g_plan_worst, PLAN_ZIGZAG);
    printf("cfg: %s (%d keys, %d bad), course.txt: %s\n",
           g_cfg_keys < 0 ? "not found" : "loaded", g_cfg_keys < 0 ? 0 : g_cfg_keys, g_cfg_bad,
           g_course_file < 0 ? "not found (using course.h)" : "loaded");
    log_plan();

#ifdef HOST_BUILD
    printf("\n=== %s path ===\n", host_worst ? "WORST (zigzag)" : "MAN");
    const Plan *hp = host_worst ? &g_plan_worst : &g_plan;
    int hok = host_worst ? g_worst_ok : g_plan_ok;
    plan_draw(&g_course, hok ? hp : NULL, host_dist);
    if (!hok) { printf("NO PATH\n"); return 1; }
    printf("%d segments, %d turns, %.2f m\n\n=== dry run ===\n", hp->n,
           plan_count_turns(&g_course, hp, P.start_heading), hp->length);
    if (!robot_init()) return 1;
    run_course(hp, hok, host_worst ? "WORST" : "MAN");
    robot_shutdown();
    return 0;
#endif

    if (!robot_init()) {
        robot_shutdown();
        return 1;
    }

    for (;;) {
        LcdClean();
        LcdTextf(1, 0, LcdRowToY(0), "PROJ1 NAVIGATION");
        if (g_plan_ok)
            LcdTextf(1, 0, LcdRowToY(1), "best %d segs, worst %d", g_plan.n, g_plan_worst.n);
        else
            LcdTextf(1, 0, LcdRowToY(1), "plan: NO PATH!");
        LcdTextf(1, 0, LcdRowToY(3), "CENTER run MANNHATTAN path");
        LcdTextf(1, 0, LcdRowToY(4), "LEFT   run curvy path");
        LcdTextf(1, 0, LcdRowToY(5), "RIGHT  show MANNHATTAN plan");
        LcdTextf(1, 0, LcdRowToY(6), "UP     gyro test");
        LcdTextf(1, 0, LcdRowToY(7), "DOWN   drive 2 m");
        LcdTextf(1, 0, LcdRowToY(8), "EXIT   quit");
        LcdTextf(1, 0, LcdRowToY(9), "hdg %.1f", robot_heading());
        LcdTextf(1, 0, LcdRowToY(10), "cfg:%s course:%s",
                 g_cfg_keys < 0 ? "default" : g_cfg_bad ? "ERR" : "file",
                 g_course_file < 0 ? "course.h" : "file");

        if (ButtonIsDown(BTNEXIT))        { wait_release(); break; }
        else if (ButtonIsDown(BTNCENTER)) { wait_release(); run_course(&g_plan, g_plan_ok, "BEST"); }
        else if (ButtonIsDown(BTNLEFT))   { wait_release(); run_course(&g_plan_worst, g_worst_ok, "WORST"); }
        else if (ButtonIsDown(BTNRIGHT))  { wait_release(); show_plan(&g_plan, g_plan_ok, "best"); }
        else if (ButtonIsDown(BTNUP))     { wait_release(); robot_test_gyro(); }
        else if (ButtonIsDown(BTNDOWN))   { wait_release(); robot_test_drive(2.0); }
        Wait(50);
    }

    robot_shutdown();
    return 0;
}
