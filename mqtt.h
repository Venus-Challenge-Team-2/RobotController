#include <string>

#ifndef MQTT_H
#define MQTT_H

void mqtt_init();
void mqtt_read();
void mqtt_update_position();
void mqtt_navigation_control();
int mqtt_needs_update();
void mqtt_send_coords();
void mqtt_destroy();

void uart_send_string(const std::string& str);

extern float robot_x;
extern float robot_y;
extern float robot_angle;

#endif
