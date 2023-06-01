#pragma once

struct PlannerBlock {

    uint32_t steps;
    uint32_t accel_until;
    uint32_t decel_after;

    float end_speed;
    float start_speed;
    float cruise_speed;

    bool decelerating;

};


void clear_planner();

bool planner_active();