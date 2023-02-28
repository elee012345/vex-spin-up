/*vex-vision-config:begin*/
#include "vex.h"


vex::vision::signature GOAL_RED = vex::vision::signature (1, 6411, 7849, 7130, -1347, -391, -869, 3.8, 0);
vex::vision::signature GOAL_BLUE = vex::vision::signature (2, -2091, 1, -1045, -1, 6121, 3060, 0.500, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision camera = vex::vision (vex::PORT10, 50, GOAL_RED, GOAL_BLUE, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/