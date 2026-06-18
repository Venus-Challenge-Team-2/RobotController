#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <utility>

enum ObjectData {
    NO_OBJECT = 0,
    SMALL_CUBE = 1,
    BIG_CUBE = 2,
    MOUNTAIN = 3,
    HOLE = 4
};

const int MAP_SIZE_X = 333;
const int MAP_SIZE_Y = 333;

struct PointData {
    ObjectData object_data;
};

extern PointData local_map[MAP_SIZE_X][MAP_SIZE_Y];
extern bool is_exploration_active;

void exploration_init();
void exploration_run();
void exploration_start();
void exploration_request_scan();
void exploration_add_obstacle(int x, int y, ObjectData type);
void exploration_reset();

#endif // EXPLORATION_H
