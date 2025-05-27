/**
 * @file ultrasonic_sensor.h
 * @brief HC-SR04 ultrasonic sensor driver header
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/atomic.h>

// HC-SR04 distance measurement range in mm (2cm to 400cm)
#define HC_SR04_MIN_DISTANCE_MM 20
#define HC_SR04_MAX_DISTANCE_MM 4000

/**
 * @brief Initialize the HC-SR04 ultrasonic sensor
 *
 * @param trigger_dev GPIO device for trigger pin
 * @param trigger_pin Trigger pin number
 * @param echo_dev GPIO device for echo pin
 * @param echo_pin Echo pin number
 * @return 0 on success, negative errno code on failure
 */
int hc_sr04_init(const struct device *trigger_dev, uint8_t trigger_pin,
                 const struct device *echo_dev, uint8_t echo_pin);

/**
 * @brief Measure distance using HC-SR04 ultrasonic sensor
 *
 * @param trigger_dev GPIO device for trigger pin
 * @param trigger_pin Trigger pin number
 * @param echo_dev GPIO device for echo pin
 * @param echo_pin Echo pin number
 * @param distance_mm Pointer to store the measured distance in millimeters
 * @return 0 on success, negative errno code on failure
 */
int hc_sr04_measure(const struct device *trigger_dev, uint8_t trigger_pin,
                    const struct device *echo_dev, uint8_t echo_pin,
                    uint16_t *distance_mm);

/**
 * @brief Start the ultrasonic sensor thread
 *
 * @param trigger_dev GPIO device for trigger pin
 * @param trigger_pin Trigger pin number
 * @param echo_dev GPIO device for echo pin
 * @param echo_pin Echo pin number
 * @param system_ready Atomic variable to indicate sensor readiness
 * @param last_distance_mm Atomic variable to store the last measured distance
 * @return 0 on success, negative errno code on failure
 */
int ultrasonic_thread_start(const struct device *trigger_dev, uint8_t trigger_pin,
                            const struct device *echo_dev, uint8_t echo_pin,
                            atomic_t *system_ready, atomic_t *last_distance_mm);

/**
 * @brief Get the last measured distance
 *
 * @return Last measured distance in millimeters
 */
uint16_t get_last_distance(void);

#endif /* ULTRASONIC_SENSOR_H */ 