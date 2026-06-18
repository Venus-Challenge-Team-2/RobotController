#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <vector>
#include <utility>

enum ObjectData {
    NO_OBJECT = 0,
    SMALL_CUBE = 1,
    BIG_CUBE = 2,
    MOUNTAIN = 3,
    HOLE = 4,
    RED_CUBE = 10,
    BLACK_CUBE = 11,
    BLUE_CUBE = 12,
    GREEN_CUBE = 13,
    WHITE_CUBE = 14
};

const int MAP_SIZE_X = 100;
const int MAP_SIZE_Y = 100;

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
void exploration_generate_hole_border();
void exploration_print_map();

#endif // EXPLORATION_H
