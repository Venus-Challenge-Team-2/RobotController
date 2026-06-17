#include <string>

#ifndef MQTT_H
#define MQTT_H

void mqtt_init();
void mqtt_read();
void mqtt_update_position();
void mqtt_navigation_control();
void mqtt_cancel_navigation();
int mqtt_needs_update();
void mqtt_send_coords();
bool mqtt_is_idle();
bool mqtt_is_retreating();
void mqtt_send_idle_msg();
void mqtt_send_work_msg();
void mqtt_destroy();

void mqtt_send_hole();
void mqtt_send_wall(uint32_t dist_mm);
void mqtt_evasive_move_back();

void uart_send_string(const std::string& str);
void set_stepper_command(int16_t left, int16_t right);

extern float robot_x;
extern float robot_y;
extern float robot_angle;
extern float robot_caster_phi;

extern const float steps_cm;
extern const float steps_rad;

#endif
