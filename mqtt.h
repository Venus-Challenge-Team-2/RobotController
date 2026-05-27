#ifndef MQTT_H
#define MQTT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * mqtt.h — MQTT/UART command interface for the camera project.
 *
 * Provides stepper-motor navigation driven by commands received over
 * UART0 (wired to the host-side MQTT bridge on IO_AR0/IO_AR1).
 *
 * Call order every loop iteration:
 *   1. mqtt_read()          – drain UART RX buffer, parse any new command
 *   2. mqtt_update_position() – integrate stepper ticks into x/y/angle
 *   3. mqtt_navigation_control() – issue next stepper command toward target
 *   4. if (mqtt_needs_update()) mqtt_send_coords(); – publish position via UART TX
 */

/* Initialise UART0, its switchbox pins, and the stepper motor driver.
   Must be called once after pynq_init(). */
void mqtt_init(void);

/* Drain UART0 RX, parse a complete message, and update target_x/target_y. */
void mqtt_read(void);

/* Integrate completed stepper ticks into the internal x/y/angle estimate. */
void mqtt_update_position(void);

/* Issue the next stepper command (rotate or translate) toward the target. */
void mqtt_navigation_control(void);

/* Returns non-zero when the position has changed since the last send. */
int mqtt_needs_update(void);

/* Serialise x/y/angle and send it over UART0 TX (clears the update flag). */
void mqtt_send_coords(void);

/* Tear down the stepper driver (call before pynq_destroy). */
void mqtt_destroy(void);

#ifdef __cplusplus
}
#endif

#endif /* MQTT_H */
