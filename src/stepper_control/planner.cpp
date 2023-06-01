#include "planner.h"
#include "../config.h"
#include "../utility/fixed_queue.h"


FixedQueue<PlannerBlock, PLANNER_QUEUE_SIZE> planner_queue;