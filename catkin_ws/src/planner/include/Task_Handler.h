#ifndef TASK_HANDLER_H
#define TASK_HANDLER_H

#include "planner.h"

void run_routine(int start_task, int end_task);

int run_gate();

int run_lane();

int run_buoy();

int end_routine();

#endif
