/**
 * @file ultrasonic_sensor.c
 * @brief HC-SR04 ultrasonic sensor driver implementation
 */

#include "ultrasonic_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>


#define STACK_SIZE         4096*2 
#define SENSOR_THREAD_PRIO 5

/* HC-SR04 specific constants */
#define HC_SR04_TRIGGER_PULSE_US   10    
#define HC_SR04_ECHO_TIMEOUT_MS    200   
#define HC_SR04_SPEED_OF_SOUND     343  


static atomic_t *g_last_distance_mm;
static atomic_t *g_system_ready;
static const struct device *g_trigger_dev;
static const struct device *g_echo_dev;
static uint8_t g_trigger_pin;
static uint8_t g_echo_pin;


K_THREAD_STACK_DEFINE(ultra_stack, STACK_SIZE);
static struct k_thread ultra_thread_data;


int hc_sr04_init(const struct device *trigger_dev, uint8_t trigger_pin,
                const struct device *echo_dev, uint8_t echo_pin)
{
    int ret;

    printk("Starting HC-SR04 initialization\n");

    if (!device_is_ready(trigger_dev)) {
        printk("ERROR: Trigger GPIO device not ready\n");
        return -ENODEV;
    }

    if (!device_is_ready(echo_dev)) {
        printk("ERROR: Echo GPIO device not ready\n");
        return -ENODEV;
    }


    ret = gpio_pin_configure(trigger_dev, trigger_pin, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    if (ret < 0) {
        printk("ERROR: Failed to configure trigger pin: %d\n", ret);
        return ret;
    }


    ret = gpio_pin_set(trigger_dev, trigger_pin, 0);
    if (ret < 0) {
        printk("ERROR: Failed to set trigger pin low: %d\n", ret);
        return ret;
    }


    ret = gpio_pin_configure(echo_dev, echo_pin, GPIO_INPUT | GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH);
    if (ret < 0) {
        printk("ERROR: Failed to configure echo pin: %d\n", ret);
        return ret;
    }


    int echo_val = gpio_pin_get(echo_dev, echo_pin);
    if (echo_val < 0) {
        printk("ERROR: Failed to read echo pin: %d\n", echo_val);
        return echo_val;
    }
    if (echo_val != 0) {
        printk("WARNING: Echo pin not in expected low state\n");

    }

    printk("HC-SR04 initialized successfully\n");
    return 0;
}


int hc_sr04_measure(const struct device *trigger_dev, uint8_t trigger_pin,
                   const struct device *echo_dev, uint8_t echo_pin,
                   uint16_t *distance_mm)
{
    uint32_t start_time, echo_start, echo_end, echo_duration;
    uint32_t timeout_cycles = k_ms_to_cyc_ceil32(HC_SR04_ECHO_TIMEOUT_MS);
    int echo_val;


    uint16_t readings[5];
    int valid_readings = 0;


    for (int i = 0; i < 5; i++) {

        gpio_pin_set(trigger_dev, trigger_pin, 0);
        k_busy_wait(5); 


        gpio_pin_set(trigger_dev, trigger_pin, 1);
        k_busy_wait(HC_SR04_TRIGGER_PULSE_US);
        gpio_pin_set(trigger_dev, trigger_pin, 0);


        k_busy_wait(10); 


        start_time = k_cycle_get_32();
        do {
            echo_val = gpio_pin_get(echo_dev, echo_pin);
            if (echo_val < 0) {
                continue; 
            }

            if (k_cycle_get_32() - start_time > timeout_cycles) {
                break;  
            }
        } while (echo_val == 0);

        if (echo_val <= 0) {
            continue;  
        }

        echo_start = k_cycle_get_32();


        do {
            echo_val = gpio_pin_get(echo_dev, echo_pin);
            if (echo_val < 0) {
                break;  
            }

            if (k_cycle_get_32() - echo_start > timeout_cycles) {
                readings[valid_readings] = HC_SR04_MAX_DISTANCE_MM;  
                valid_readings++;
                break; 
            }
        } while (echo_val == 1);

        if (echo_val < 0) {
            continue; 
        }

        if (echo_val == 0) {
            echo_end = k_cycle_get_32();


            echo_duration = k_cyc_to_us_floor32(echo_end - echo_start);


            if (echo_duration < 100) {  
                continue; 
            }


            readings[valid_readings] = (echo_duration * HC_SR04_SPEED_OF_SOUND) / 2000;


            if (readings[valid_readings] < HC_SR04_MIN_DISTANCE_MM ||
                readings[valid_readings] > HC_SR04_MAX_DISTANCE_MM) {


                if (readings[valid_readings] > HC_SR04_MAX_DISTANCE_MM) {
                    readings[valid_readings] = HC_SR04_MAX_DISTANCE_MM;
                } else if (readings[valid_readings] < HC_SR04_MIN_DISTANCE_MM) {
                    readings[valid_readings] = HC_SR04_MIN_DISTANCE_MM;
                }
            }

            valid_readings++;
        }


        if (i < 4) {
            k_msleep(50); 
        }
    }

    if (valid_readings == 0) {
        return -EIO;
    }


    if (valid_readings > 1) {
        for (int i = 1; i < valid_readings; i++) {
            uint16_t key = readings[i];
            int j = i - 1;

            while (j >= 0 && readings[j] > key) {
                readings[j + 1] = readings[j];
                j--;
            }
            readings[j + 1] = key;
        }
    }


    if (valid_readings >= 5) {

        *distance_mm = readings[2];
    } else if (valid_readings >= 3) {

        *distance_mm = readings[valid_readings / 2];
    } else if (valid_readings == 2) {

        *distance_mm = (readings[0] + readings[1]) / 2;
    } else {

        *distance_mm = readings[0];
    }

    return 0;
}


static void ultra_thread(void *p1, void *p2, void *p3)
{
    uint16_t mm;
    int rc;
    uint32_t iteration = 0;
    uint32_t success_count = 0;
    uint32_t error_count = 0;


    const struct device *trigger_dev = g_trigger_dev;
    const struct device *echo_dev = g_echo_dev;
    uint8_t trigger_pin = g_trigger_pin;
    uint8_t echo_pin = g_echo_pin;

    printk("Ultrasonic thread started\n");
    printk("Trigger GPIO: %s pin %d\n", trigger_dev->name, trigger_pin);
    printk("Echo GPIO: %s pin %d\n", echo_dev->name, echo_pin);


    int retry = 0;
    int max_retries = 5;

    while (retry < max_retries) {
        rc = hc_sr04_init(trigger_dev, trigger_pin, echo_dev, echo_pin);
        if (rc == 0) {
            break;
        }
        

        k_msleep(100 * (retry + 1));
        retry++;
    }

    if (rc != 0) {
        printk("CRITICAL ERROR: Failed to initialize HC-SR04\n");

        atomic_set(g_system_ready, 1); 
        return; 
    }

    printk("HC-SR04 sensor ready\n");


    k_msleep(1000);


    atomic_set(g_system_ready, 1);

    while (1) {
        iteration++;


        int measure_attempt = 0;
        int max_measure_attempts = 5;
        bool success = false;

        while (measure_attempt < max_measure_attempts && !success) {
            rc = hc_sr04_measure(trigger_dev, trigger_pin, echo_dev, echo_pin, &mm);

            if (rc == 0) {
                success = true;
                success_count++;


                atomic_set(g_last_distance_mm, mm);
            } else {
                measure_attempt++;
                error_count++;

                if (measure_attempt < max_measure_attempts) {

                    k_msleep(50 * measure_attempt);
                }
            }
        }

        if (!success) {
            if (error_count > 10 && success_count == 0) {
                printk("Too many consecutive errors, reinitializing ultrasonic sensor\n");
                hc_sr04_init(trigger_dev, trigger_pin, echo_dev, echo_pin);
                k_msleep(500);
                error_count = 0;  
            }
        }


        if (success) {
            k_msleep(1000); 
        } else {
            k_msleep(2000); 
        }
    }
}


int ultrasonic_thread_start(const struct device *trigger_dev, uint8_t trigger_pin,
                           const struct device *echo_dev, uint8_t echo_pin,
                           atomic_t *system_ready, atomic_t *last_distance_mm)
{

    g_trigger_dev = trigger_dev;
    g_echo_dev = echo_dev;
    g_trigger_pin = trigger_pin;
    g_echo_pin = echo_pin;
    g_system_ready = system_ready;
    g_last_distance_mm = last_distance_mm;


    k_tid_t tid = k_thread_create(&ultra_thread_data, ultra_stack,
                                 STACK_SIZE,
                                 ultra_thread, NULL, NULL, NULL,
                                 SENSOR_THREAD_PRIO, 0, K_MSEC(500)); 

    if (tid == NULL) {
        printk("ERROR: Failed to create ultrasonic sensor thread\n");
        return -ENOMEM;
    }

    return 0;
}


uint16_t get_last_distance(void)
{
    if (g_last_distance_mm == NULL) {
        return 0;
    }
    return (uint16_t)atomic_get(g_last_distance_mm);
} 