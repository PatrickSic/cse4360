/*
 * planner.c - Manhattan-distance, See planner.h.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "planner.h"

#define MAX_COLS 40                 /* 4.88 m / 0.1525 m = 32 cells -> 33 nodes */
#define MAX_ROWS 28                 /* 3.05 m / 0.1525 m = 20 cells -> 21 nodes */
#define MAX_NODES (MAX_COLS * MAX_ROWS)
#define UNREACHED (-1)

/*for drawing*/
static int   g_dist[MAX_NODES];
static int   g_cols = 0;

/*Each grid only has four neighbors, +X, -X, +Y, -Y*/
static const int DI[4] = { 1, -1, 0, 0 };
static const int DJ[4] = { 0, 0, 1, -1 };

void course_init(Course *c, double width, double height, double obstacle_half,
                 double robot_radius, double safety_margin)
{
    memset(c, 0, sizeof(*c));
    c->width = width;
    c->height = height;
    c->obstacle_half = obstacle_half;
    c->robot_radius = robot_radius;
    c->safety_margin = safety_margin;
}

/* THe signed distance from one grid to another*/
static double square_sdist(Point p, double cx, double cy, double h)
{
    double dx = fabs(p.x - cx) - h;
    double dy = fabs(p.y - cy) - h;
    if (dx > 0.0 && dy > 0.0) return sqrt(dx * dx + dy * dy);
    return (dx > dy) ? dx : dy;
}

double course_clearance(const Course *c, Point p)
{
    double d = p.x;                                   /* walls */
    if (p.y < d) d = p.y;
    if (c->width - p.x < d) d = c->width - p.x;
    if (c->height - p.y < d) d = c->height - p.y;

    for (int k = 0; k < c->num_obstacles; k++) {      /* obstacle squares */
        double o = square_sdist(p, c->obstacle[k].x, c->obstacle[k].y, c->obstacle_half);
        if (o < d) d = o;
    }
    return d;
}

int plan_grid_cols(const Course *c) { return (int)floor(c->width  / GRID_CELL_M + 0.5) + 1; }
int plan_grid_rows(const Course *c) { return (int)floor(c->height / GRID_CELL_M + 0.5) + 1; }

static Point node_point(int i, int j)
{
    Point p = { i * GRID_CELL_M, j * GRID_CELL_M };
    return p;
}

static void nearest_node(const Course *c, Point p, int *i, int *j)
{
    *i = (int)floor(p.x / GRID_CELL_M + 0.5);
    *j = (int)floor(p.y / GRID_CELL_M + 0.5);
    if (*i < 0) *i = 0;
    if (*j < 0) *j = 0;
    if (*i >= plan_grid_cols(c)) *i = plan_grid_cols(c) - 1;
    if (*j >= plan_grid_rows(c)) *j = plan_grid_rows(c) - 1;
}

int plan_node_blocked(const Course *c, int i, int j)
{
    return course_clearance(c, node_point(i, j)) < c->robot_radius + c->safety_margin;
}

int plan_node_distance(int i, int j)
{
    if (g_cols == 0) return UNREACHED;
    return g_dist[j * g_cols + i];
}

/* ------------------------------------------------------------------ */
int plan_path_mode(const Course *c, Plan *out, int mode)
{
    static unsigned char blocked[MAX_NODES];
    static int queue[MAX_NODES];

    memset(out, 0, sizeof(*out));
    const int cols = plan_grid_cols(c), rows = plan_grid_rows(c);
    const int N = cols * rows;
    g_cols = cols;

    int si, sj, gi, gj;
    nearest_node(c, c->start, &si, &sj);
    nearest_node(c, c->goal,  &gi, &gj);
    const int s = sj * cols + si, g = gj * cols + gi;

    /*Fills up the grid with obstacles and walls*/
    for (int n = 0; n < N; n++) {
        blocked[n] = plan_node_blocked(c, n % cols, n / cols);
        g_dist[n] = UNREACHED;
    }
    blocked[s] = blocked[g] = 0;    /*I guess*/

    /*bfs of manhattan distance to the goal*/
    int head = 0, tail = 0;
    g_dist[g] = 0;
    queue[tail++] = g;
    while (head < tail) {
        int u = queue[head++];
        int ui = u % cols, uj = u / cols;
        for (int k = 0; k < 4; k++) {
            int vi = ui + DI[k], vj = uj + DJ[k];
            if (vi < 0 || vj < 0 || vi >= cols || vj >= rows) continue;
            int v = vj * cols + vi;
            if (blocked[v] || g_dist[v] != UNREACHED) continue;
            g_dist[v] = g_dist[u] + 1;
            queue[tail++] = v;
        }
    }
    if (g_dist[s] == UNREACHED) return 0;             

    /*Descend the manhattan distance map from the starting node to the goal*/
    int ci = si, cj = sj, dir = -1;
    int n = 0;
    Point seg_start = c->start;                         
    while (!(ci == gi && cj == gj)) {
        int cur = g_dist[cj * cols + ci];
        int best = -1;
        double best_clear = -1.0;
        for (int k = 0; k < 4; k++) {
            int vi = ci + DI[k], vj = cj + DJ[k];
            if (vi < 0 || vj < 0 || vi >= cols || vj >= rows) continue;
            int v = vj * cols + vi;
            if (g_dist[v] != cur - 1) continue;        
            double cl = course_clearance(c, node_point(vi, vj));
            int k_straight = (k == dir), b_straight = (best == dir);
            int take;
            if (best < 0)                     take = 1;
            else if (mode == PLAN_ZIGZAG)     /* turning beats straight */
                take = (!k_straight && b_straight) || (k_straight == b_straight && cl > best_clear);
            else                              /* straight beats turning */
                take = (k_straight && !b_straight) || (k_straight == b_straight && cl > best_clear);
            if (take) { best = k; best_clear = cl; }
        }
        if (best < 0) return 0;                         /* cannot happen after BFS */

        if (dir >= 0 && best != dir) {                  /* direction change: */
            if (n >= PLAN_MAX_WAYPOINTS) return 0;      /*close the current segment since we changed direction*/
            out->wp[n++] = node_point(ci, cj);
            seg_start = out->wp[n - 1];
        }
        dir = best;
        ci += DI[best]; cj += DJ[best];
    }
    if (n >= PLAN_MAX_WAYPOINTS) return 0;
    out->wp[n++] = c->goal;                           
    out->n = n;
    (void)seg_start;

    /* total length*/
    Point prev = c->start;
    for (int k = 0; k < n; k++) {
        out->length += fabs(out->wp[k].x - prev.x) + fabs(out->wp[k].y - prev.y);
        prev = out->wp[k];
    }
    return 1;
}

int plan_count_turns(const Course *c, const Plan *p, double start_heading_deg)
{
    int turns = 0;
    Point prev = c->start;
    double h = start_heading_deg;
    for (int k = 0; k < p->n; k++) {
        double dx = p->wp[k].x - prev.x, dy = p->wp[k].y - prev.y;
        if (fabs(dx) < 1e-6 && fabs(dy) < 1e-6) continue;
        double seg_h = atan2(dy, dx) * 180.0 / 3.14159265358979323846;
        double d = seg_h - h;
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        if (fabs(d) > 1.0) turns++;
        h = seg_h;
        prev = p->wp[k];
    }
    return turns;
}

int plan_path(const Course *c, Plan *out)
{
    return plan_path_mode(c, out, PLAN_MAN);
}

/* ------------------------------------------------------------------ */
static int ni(double x) { return (int)floor(x / GRID_CELL_M + 0.5); }

void plan_draw(const Course *c, const Plan *p, int show_dist)
{
    int cols = plan_grid_cols(c), rows = plan_grid_rows(c);
    static char map[64][64];

    for (int j = 0; j < rows; j++)
        for (int i = 0; i < cols; i++) {
            Point q = { i * GRID_CELL_M, j * GRID_CELL_M };
            map[j][i] = course_clearance(c, q) <= 0.0 ? '#'
                      : plan_node_blocked(c, i, j)   ? '.' : ' ';
        }
    Point prev = c->start;
    for (int k = 0; p && k < p->n; k++) {
        int i0 = ni(prev.x), j0 = ni(prev.y), i1 = ni(p->wp[k].x), j1 = ni(p->wp[k].y);
        int steps = abs(i1 - i0) + abs(j1 - j0);
        for (int s = 0; s <= steps; s++)
            map[j0 + (j1 - j0) * s / (steps ? steps : 1)][i0 + (i1 - i0) * s / (steps ? steps : 1)] = '*';
        prev = p->wp[k];
    }
    for (int k = 0; p && k < p->n; k++) map[ni(p->wp[k].y)][ni(p->wp[k].x)] = 'o';
    map[ni(c->start.y)][ni(c->start.x)] = 'S';
    map[ni(c->goal.y)][ni(c->goal.x)] = 'G';

    printf("   +"); for (int i = 0; i < cols; i++) putchar('-'); printf("+\n");
    for (int j = rows - 1; j >= 0; j--) {
        printf("%2d |", j);
        for (int i = 0; i < cols; i++) putchar(map[j][i]);
        printf("|\n");
    }
    printf("   +"); for (int i = 0; i < cols; i++) putchar('-'); printf("+\n");
    printf("    0         1         2         3    (x, tiles*2)\n");

    if (show_dist) {
        printf("\nManhattan distance to goal (grid steps, '##' = obstacle, '..' = blocked):\n");
        for (int j = rows - 1; j >= 0; j--) {
            printf("%2d |", j);
            for (int i = 0; i < cols; i++) {
                int d = plan_node_distance(i, j);
                if (map[j][i] == '#') printf("##");
                else if (d < 0) printf("..");
                else printf("%2d", d % 100);
            }
            printf("\n");
        }
    }
}
