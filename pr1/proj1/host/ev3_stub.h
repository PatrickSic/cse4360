#ifndef EV3_STUB_H
#define EV3_STUB_H
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#define OUT_A 1
#define OUT_B 2
#define OUT_C 4
#define OUT_D 8
#define IN_1 0
#define IN_2 1
#define IN_3 2
#define IN_4 3

enum { BTNEXIT = 1, BTNRIGHT, BTNLEFT, BTNCENTER, BTNUP, BTNDOWN };

/* Buttons are done on every other poll because there is no physical possible interaction with the simulation*/
/* Same reasoning for the stderr-free silence*/
static inline bool  ButtonIsDown(int b)                 { static int n; return b == BTNCENTER && ((n++ & 1) == 0); }
static inline unsigned ButtonWaitForAnyPress(unsigned t){ (void)t; return BTNCENTER; }
static inline void  Wait(unsigned long ms)              { (void)ms; }
static inline void  PlayTone(unsigned f, unsigned d)    { (void)f; (void)d; }
static inline bool  LcdClean(void)                      { return true; }
static inline short LcdRowToY(int row)                  { return (short)(row * 10); }
static inline bool  LcdTextf(char c, short x, short y, const char *fmt, ...) { (void)c; (void)x; (void)y; (void)fmt; return true; }

#endif
