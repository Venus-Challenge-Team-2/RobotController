#include "exploration.h"
#include "mqtt.h"
#include <queue>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <sstream>

PointData local_map[MAP_SIZE_X][MAP_SIZE_Y];
bool is_exploration_active = false;

static std::vector<std::pair<int, int>> exploration_points;
static std::vector<std::pair<int, int>> current_path;
static bool is_waiting_for_scan = false;
static std::pair<int, int> last_target = {-1, -1};
static std::pair<int, int> current_waypoint = {-1, -1};

extern bool scanning;

void exploration_init() {
    for (int x = 0; x < MAP_SIZE_X; x++) {
        for (int y = 0; y < MAP_SIZE_Y; y++) {
            local_map[x][y].object_data = NO_OBJECT;
        }
    }
}

void exploration_add_obstacle(int x, int y, ObjectData type) {
    if (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y) {
        if (local_map[x][y].object_data != type) {
            local_map[x][y].object_data = type;
        }
    }
}

void exploration_request_scan() {
    is_waiting_for_scan = true;
    current_path.clear();
    current_waypoint = {-1, -1};
}

static std::vector<std::pair<int, int>> findEnclosedArea() {
    bool isOutside[MAP_SIZE_X][MAP_SIZE_Y] = {false};
    std::queue<std::pair<int, int>> q;

    for (int x = 0; x < MAP_SIZE_X; x++) {
        if (local_map[x][0].object_data != HOLE) { isOutside[x][0] = true; q.push({x, 0}); }
        if (local_map[x][MAP_SIZE_Y - 1].object_data != HOLE) { isOutside[x][MAP_SIZE_Y - 1] = true; q.push({x, MAP_SIZE_Y - 1}); }
    }
    for (int y = 0; y < MAP_SIZE_Y; y++) {
        if (local_map[0][y].object_data != HOLE) { isOutside[0][y] = true; q.push({0, y}); }
        if (local_map[MAP_SIZE_X - 1][y].object_data != HOLE) { isOutside[MAP_SIZE_X - 1][y] = true; q.push({MAP_SIZE_X - 1, y}); }
    }

    while (!q.empty()) {
        std::pair<int, int> curr = q.front();
        q.pop();
        int dx[] = {-1, 1, 0, 0};
        int dy[] = {0, 0, -1, 1};
        for (int i = 0; i < 4; i++) {
            int nx = curr.first + dx[i];
            int ny = curr.second + dy[i];
            if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y && !isOutside[nx][ny] && local_map[nx][ny].object_data != HOLE) {
                isOutside[nx][ny] = true;
                q.push({nx, ny});
            }
        }
    }

    std::vector<std::vector<std::pair<int, int>>> components;
    bool visited[MAP_SIZE_X][MAP_SIZE_Y] = {false};

    for (int x = 0; x < MAP_SIZE_X; x++) {
        for (int y = 0; y < MAP_SIZE_Y; y++) {
            if (!isOutside[x][y] && local_map[x][y].object_data != HOLE && !visited[x][y]) {
                std::vector<std::pair<int, int>> component;
                std::queue<std::pair<int, int>> cq;
                visited[x][y] = true;
                cq.push({x, y});
                component.push_back({x, y});

                while (!cq.empty()) {
                    std::pair<int, int> curr = cq.front();
                    cq.pop();
                    int dx[] = {-1, 1, 0, 0};
                    int dy[] = {0, 0, -1, 1};
                    for (int i = 0; i < 4; i++) {
                        int nx = curr.first + dx[i];
                        int ny = curr.second + dy[i];
                        if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y &&
                            !isOutside[nx][ny] && local_map[nx][ny].object_data != HOLE && !visited[nx][ny]) {
                            visited[nx][ny] = true;
                            cq.push({nx, ny});
                            component.push_back({nx, ny});
                        }
                    }
                }
                components.push_back(component);
            }
        }
    }

    size_t max_size = 0;
    int max_idx = -1;
    for (size_t i = 0; i < components.size(); i++) {
        if (components[i].size() > max_size) {
            max_size = components[i].size();
            max_idx = i;
        }
    }

    if (max_idx != -1) return components[max_idx];
    return {};
}

static std::vector<std::pair<int, int>> planScanPoints(const std::vector<std::pair<int, int>>& area) {
    if (area.empty()) return {};
    int minX = MAP_SIZE_X, maxX = 0, minY = MAP_SIZE_Y, maxY = 0;
    for (const auto& p : area) {
        if (p.first < minX) minX = p.first;
        if (p.first > maxX) maxX = p.first;
        if (p.second < minY) minY = p.second;
        if (p.second > maxY) maxY = p.second;
    }

    std::vector<std::pair<int, int>> points;
    // Step size 10 grid units = 30cm
    for (int x = minX + 5; x <= maxX; x += 10) {
        for (int y = minY + 5; y <= maxY; y += 10) {
            // Find the point in the enclosed area that is closest to (x, y) AND is NO_OBJECT
            float min_d = 1e9;
            std::pair<int, int> best_p = {-1, -1};
            for (const auto& p : area) {
                if (local_map[p.first][p.second].object_data == NO_OBJECT) {
                    float d = std::sqrt(std::pow(p.first - x, 2) + std::pow(p.second - y, 2));
                    if (d < min_d) {
                        min_d = d;
                        best_p = p;
                    }
                }
            }
            if (best_p.first != -1) {
                bool already_in = false;
                for (const auto& ep : points) {
                    if (ep == best_p) { already_in = true; break; }
                }
                if (!already_in) points.push_back(best_p);
            }
        }
    }
    return points;
}

static bool isSafe(int x, int y, int buffer) {
    if (x < 0 || x >= MAP_SIZE_X || y < 0 || y >= MAP_SIZE_Y) return false;
    for (int dx = -buffer; dx <= buffer; dx++) {
        for (int dy = -buffer; dy <= buffer; dy++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < MAP_SIZE_X && ny >= 0 && ny < MAP_SIZE_Y) {
                if (local_map[nx][ny].object_data != NO_OBJECT) return false;
            }
        }
    }
    return true;
}

struct Node {
    int x, y;
    double g, h;
    double f() const { return g + h; }
    bool operator>(const Node& other) const { return f() > other.f(); }
};

static double heuristic(int x, int y, int ex, int ey) {
    int dx = std::abs(x - ex);
    int dy = std::abs(y - ey);
    return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

static bool hasLineOfSight(std::pair<int, int> p1, std::pair<int, int> p2, int buffer) {
    double x = p1.first;
    double y = p1.second;
    double dx = p2.first - p1.first;
    double dy = p2.second - p1.second;
    double distance = std::sqrt(dx * dx + dy * dy);
    if (distance == 0) return true;
    double stepX = dx / distance;
    double stepY = dy / distance;
    for (int i = 1; i <= (int)distance; i++) {
        x += stepX;
        y += stepY;
        if (!isSafe((int)x, (int)y, buffer)) return false;
    }
    return isSafe(p2.first, p2.second, buffer);
}

static std::vector<std::pair<int, int>> simplifyPath(const std::vector<std::pair<int, int>>& path, int buffer) {
    if (path.size() <= 2) return path;
    std::vector<std::pair<int, int>> simplified;
    simplified.push_back(path[0]);
    size_t currentIdx = 0;
    while (currentIdx < path.size() - 1) {
        size_t nextIdx = path.size() - 1;
        while (nextIdx > currentIdx + 1) {
            if (hasLineOfSight(path[currentIdx], path[nextIdx], buffer)) break;
            nextIdx--;
        }
        simplified.push_back(path[nextIdx]);
        currentIdx = nextIdx;
    }
    return simplified;
}

static std::vector<std::pair<int, int>> findPathWithBuffer(int sx, int sy, int ex, int ey, int buffer) {
    if (sx == ex && sy == ey) return {};
    // If goal is unsafe, we can't go there
    if (!isSafe(ex, ey, buffer)) return {};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    // Allow start to be unsafe, so we can escape obstacles
    openSet.push({sx, sy, 0.0, heuristic(sx, sy, ex, ey)});

    std::pair<int, int> parent[MAP_SIZE_X][MAP_SIZE_Y];
    for(int i=0; i<MAP_SIZE_X; i++) for(int j=0; j<MAP_SIZE_Y; j++) parent[i][j] = {-1, -1};

    double gScore[MAP_SIZE_X][MAP_SIZE_Y];
    for(int i=0; i<MAP_SIZE_X; i++) for(int j=0; j<MAP_SIZE_Y; j++) gScore[i][j] = 1e9;
    gScore[sx][sy] = 0.0;

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.x == ex && current.y == ey) {
            std::vector<std::pair<int, int>> path;
            std::pair<int, int> p = {ex, ey};
            while (p.first != -1) {
                path.push_back(p);
                if (p.first == sx && p.second == sy) break;
                p = parent[p.first][p.second];
            }
            std::reverse(path.begin(), path.end());
            return simplifyPath(path, buffer);
        }

        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                int nx = current.x + dx;
                int ny = current.y + dy
;
                if (isSafe(nx, ny, buffer)) {
                    double moveCost = (std::abs(dx) + std::abs(dy) == 2) ? std::sqrt(2.0) : 1.0;
                    double tentativeG = gScore[current.x][current.y] + moveCost;
                    if (tentativeG < gScore[nx][ny]) {
                        parent[nx][ny] = {current.x, current.y};
                        gScore[nx][ny] = tentativeG;
                        openSet.push({nx, ny, tentativeG, heuristic(nx, ny, ex, ey)});
                    }
                }
            }
        }
    }
    return {};
}

void exploration_start() {
    std::cout << "Goal exploration start" << std::endl; 
    
    if (is_exploration_active) return;
    std::cout << "Goal exploration not active" << std::endl; 
    
    std::vector<std::pair<int, int>> enclosed = findEnclosedArea();
    if (enclosed.size() >= 300) {
        printf("Goal continue enclosed size good");
        fflush(stdout);
        exploration_points = planScanPoints(enclosed);
        if (!exploration_points.empty()) {
            std::cout << "Goal points not empty" << std::endl; 
            
            is_exploration_active = true;

            std::cout << "Exploration started locally! Points: " << exploration_points.size() << "\n" << std::endl; 
            for (const auto& p : exploration_points) {
                printf("Goal point: (%d, %d)\n", p.first, p.second);
                fflush(stdout);
            }
        } else {
            printf("Exploration failed to start: no points planned.\n");
        }
    } else {
        printf("Exploration failed to start: enclosed area too small (%zu).\n", enclosed.size());
    }
}

void exploration_run() {
    if (!is_exploration_active) return;
    if (mqtt_is_retreating()) return;
    if (!mqtt_is_idle()) return; // Wait for previous command to finish

    int cx = (int)robot_x;
    int cy = (int)robot_y;

    // Check if we reached the current waypoint
    if (current_waypoint.first != -1) {
        int dx = cx - current_waypoint.first;
        int dy = cy - current_waypoint.second;
        // If we are close enough to the waypoint, clear it
        if (dx * dx + dy * dy <= 4) {
            current_waypoint = {-1, -1};
        } else {
            // We were interrupted or missed it, recalculate path to target
            current_path.clear();
            current_waypoint = {-1, -1};
            // Fall through to plan path to last_target again
        }
    }

    if (is_waiting_for_scan) {
        printf("Exploration: Triggering scan at (%d, %d)\n", cx, cy);
        scanning = true; // Trigger scan in main loop
        is_waiting_for_scan = false;
        // next call will proceed because mqtt_is_idle() will be false while scanning
        return;
    }

    if (!current_path.empty()) {
        std::pair<int, int> next = current_path[0];
        current_path.erase(current_path.begin());
        current_waypoint = next;
        printf("Exploration: Moving to waypoint (%d, %d)\n", next.first, next.second);
        mqtt_set_target(next.first, next.second);

        // If this was the last waypoint in the path to the target, wait for scan after arrival
        if (current_path.empty()) {
            is_waiting_for_scan = true;
        }
    } else {
        // No current path, find next target
        std::pair<int, int> target = {-1, -1};

        // If we have a last target we haven't reached yet
        if (last_target.first != -1 && (std::abs(cx - last_target.first) > 2 || std::abs(cy - last_target.second) > 2)) {
            target = last_target;
        } else if (!exploration_points.empty()) {
            target = exploration_points[0];
            exploration_points.erase(exploration_points.begin());
            last_target = target;
            printf("Exploration: New target point (%d, %d)\n", target.first, target.second);
        }

        if (target.first != -1) {
            std::vector<std::pair<int, int>> path = findPathWithBuffer(cx, cy, target.first, target.second, 1);
            if (!path.empty()) {
                current_path = path;
                // Recursive-ish call to start moving immediately
                exploration_run();
            } else {
                printf("Exploration: Could not find path to (%d, %d), skipping point.\n", target.first, target.second);
                last_target = {-1, -1};
                // Recursive call to find next point
                exploration_run();
            }
        } else {
            is_exploration_active = false;
            printf("Exploration finished: All points visited.\n");
        }
    }
}

void exploration_reset() {
    exploration_points.clear();
    current_path.clear();
    is_exploration_active = false;
    is_waiting_for_scan = false;
    last_target = {-1, -1};
    current_waypoint = {-1, -1};
}

void exploration_generate_hole_border() {
    int halfMapX = MAP_SIZE_X / 2;
    int halfMapY = MAP_SIZE_Y / 2;
    int borderRadius = 15;

    int minX = std::max(halfMapX - borderRadius, 0);
    int maxX = std::min(halfMapX + borderRadius, MAP_SIZE_X - 1);
    int minY = std::max(halfMapY - borderRadius, 0);
    int maxY = std::min(halfMapY + borderRadius, MAP_SIZE_Y - 1);

    for (int gridX = 0; gridX < MAP_SIZE_X; gridX++) {
        for (int gridY = 0; gridY < MAP_SIZE_Y; gridY++) {
            bool isXEdge = (gridX == minX || gridX == maxX) && (gridY >= minY && gridY <= maxY);
            bool isYEdge = (gridY == minY || gridY == maxY) && (gridX >= minX && gridX <= maxX);

            if (isXEdge || isYEdge) {
                local_map[gridX][gridY].object_data = HOLE;
            }
        }
    }
}

void exploration_print_map() {
    int minX = MAP_SIZE_X, maxX = 0, minY = MAP_SIZE_Y, maxY = 0;
    bool found_hole = false;

    // Find the bounding box of holes
    for (int x = 0; x < MAP_SIZE_X; x++) {
        for (int y = 0; y < MAP_SIZE_Y; y++) {
            if (local_map[x][y].object_data == HOLE) {
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
                found_hole = true;
            }
        }
    }

    if (!found_hole) {
        // If no holes found, maybe print a default area around center
        minX = MAP_SIZE_X / 2 - 20;
        maxX = MAP_SIZE_X / 2 + 20;
        minY = MAP_SIZE_Y / 2 - 20;
        maxY = MAP_SIZE_Y / 2 + 20;
    }

    // Clear terminal
    printf("\033[H\033[J");

    // Print map (Y decreases downwards in terminal output for intuitive view)
    for (int y = maxY; y >= minY; y--) {
        for (int x = minX; x <= maxX; x++) {
            char c = '.';
            switch (local_map[x][y].object_data) {
                case HOLE:       c = 'H'; break;
                case MOUNTAIN:   c = 'W'; break;
                case RED_CUBE:   c = 'R'; break;
                case BLACK_CUBE: c = 'K'; break; // 'K' for Black
                case BLUE_CUBE:  c = 'B'; break;
                case GREEN_CUBE: c = 'G'; break;
                case WHITE_CUBE: c = 'W'; break;
                default:         c = '.'; break;
            }
            printf("%c", c);
        }
        printf("\n");
    }
    fflush(stdout);
}

