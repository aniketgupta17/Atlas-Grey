/**
 * @file ultrasonic_sensor.c
 * @brief HC-SR04 ultrasonic sensor driver implementation
 */

#include "ultrasonic_sensor.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

/* Stack size and priority for the thread */
#define STACK_SIZE         4096*2  /* Doubled stack size for sensor */
#define SENSOR_THREAD_PRIO 5

/* HC-SR04 specific constants */
#define HC_SR04_TRIGGER_PULSE_US   10    /* Trigger pulse duration in microseconds */
#define HC_SR04_ECHO_TIMEOUT_MS    200   /* Increased timeout for echo */
#define HC_SR04_SPEED_OF_SOUND     343   /* Speed of sound in m/s */

/* Thread state variables */
static atomic_t *g_last_distance_mm;
static atomic_t *g_system_ready;
static const struct device *g_trigger_dev;
static const struct device *g_echo_dev;
static uint8_t g_trigger_pin;
static uint8_t g_echo_pin;

/* Thread stack */
K_THREAD_STACK_DEFINE(ultra_stack, STACK_SIZE);
static struct k_thread ultra_thread_data;

/* Improved HC-SR04 initialization with more robust error handling */
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

    /* Configure trigger pin as output with initial low level */
    ret = gpio_pin_configure(trigger_dev, trigger_pin, GPIO_OUTPUT_INACTIVE | GPIO_ACTIVE_HIGH);
    if (ret < 0) {
        printk("ERROR: Failed to configure trigger pin: %d\n", ret);
        return ret;
    }

    /* Make sure trigger pin is low */
    ret = gpio_pin_set(trigger_dev, trigger_pin, 0);
    if (ret < 0) {
        printk("ERROR: Failed to set trigger pin low: %d\n", ret);
        return ret;
    }

    /* Configure echo pin as input with pull-down to ensure stable low state */
    ret = gpio_pin_configure(echo_dev, echo_pin, GPIO_INPUT | GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH);
    if (ret < 0) {
        printk("ERROR: Failed to configure echo pin: %d\n", ret);
        return ret;
    }

    /* Verify echo pin state is low initially */
    int echo_val = gpio_pin_get(echo_dev, echo_pin);
    if (echo_val < 0) {
        printk("ERROR: Failed to read echo pin: %d\n", echo_val);
        return echo_val;
    }
    if (echo_val != 0) {
        printk("WARNING: Echo pin not in expected low state\n");
        /* Don't return error, we'll try anyway */
    }

    printk("HC-SR04 initialized successfully\n");
    return 0;
}

/* Improved HC-SR04 measurement function with robust timing and filtering */
int hc_sr04_measure(const struct device *trigger_dev, uint8_t trigger_pin,
                   const struct device *echo_dev, uint8_t echo_pin,
                   uint16_t *distance_mm)
{
    uint32_t start_time, echo_start, echo_end, echo_duration;
    uint32_t timeout_cycles = k_ms_to_cyc_ceil32(HC_SR04_ECHO_TIMEOUT_MS);
    int echo_val;

    /* Use median filtering - take 5 readings and use the median */
    uint16_t readings[5];
    int valid_readings = 0;

    /* Take up to 5 measurements for better reliability */
    for (int i = 0; i < 5; i++) {
        /* Ensure trigger is low before starting */
        gpio_pin_set(trigger_dev, trigger_pin, 0);
        k_busy_wait(5);  /* 5us delay to ensure pin is stable */

        /* Generate 10us trigger pulse */
        gpio_pin_set(trigger_dev, trigger_pin, 1);
        k_busy_wait(HC_SR04_TRIGGER_PULSE_US);
        gpio_pin_set(trigger_dev, trigger_pin, 0);

        /* Wait briefly for the sensor to process the trigger */
        k_busy_wait(10);  /* Increased from 5 to 10 us */

        /* Wait for echo pin to go high (pulse start) with timeout */
        start_time = k_cycle_get_32();
        do {
            echo_val = gpio_pin_get(echo_dev, echo_pin);
            if (echo_val < 0) {
                continue;  /* Try next reading */
            }

            if (k_cycle_get_32() - start_time > timeout_cycles) {
                break;  /* Try next reading */
            }
        } while (echo_val == 0);

        if (echo_val <= 0) {
            continue;  /* Skip to next reading attempt */
        }

        echo_start = k_cycle_get_32();

        /* Wait for echo pin to go low (pulse end) with timeout */
        do {
            echo_val = gpio_pin_get(echo_dev, echo_pin);
            if (echo_val < 0) {
                break;  /* Try next reading */
            }

            if (k_cycle_get_32() - echo_start > timeout_cycles) {
                readings[valid_readings] = HC_SR04_MAX_DISTANCE_MM;  /* Use max distance on timeout */
                valid_readings++;
                break;  /* Record timeout as max distance */
            }
        } while (echo_val == 1);

        if (echo_val < 0) {
            continue;  /* Skip to next reading attempt on error */
        }

        if (echo_val == 0) {
            echo_end = k_cycle_get_32();

            /* Calculate echo duration in microseconds */
            echo_duration = k_cyc_to_us_floor32(echo_end - echo_start);

            /* Check for very short pulses which could be noise */
            if (echo_duration < 100) {  /* Less than 100µs is likely noise */
                continue;  /* Skip this reading */
            }

            /* Calculate distance using the speed of sound formula */
            readings[valid_readings] = (echo_duration * HC_SR04_SPEED_OF_SOUND) / 2000;

            /* Validate range - HC-SR04 typically works from 2cm to 400cm */
            if (readings[valid_readings] < HC_SR04_MIN_DISTANCE_MM ||
                readings[valid_readings] > HC_SR04_MAX_DISTANCE_MM) {

                /* Clamp to valid range */
                if (readings[valid_readings] > HC_SR04_MAX_DISTANCE_MM) {
                    readings[valid_readings] = HC_SR04_MAX_DISTANCE_MM;
                } else if (readings[valid_readings] < HC_SR04_MIN_DISTANCE_MM) {
                    readings[valid_readings] = HC_SR04_MIN_DISTANCE_MM;
                }
            }

            valid_readings++;
        }

        /* Wait longer between readings */
        if (i < 4) {
            k_msleep(50);  /* Increased from 10ms to 50ms between individual readings */
        }
    }

    if (valid_readings == 0) {
        return -EIO;
    }

    /* Sort readings (simple insertion sort for small array) */
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

    /* Select measurement based on number of valid readings */
    if (valid_readings >= 5) {
        /* Use median of 5 readings */
        *distance_mm = readings[2];
    } else if (valid_readings >= 3) {
        /* Use median of 3+ readings */
        *distance_mm = readings[valid_readings / 2];
    } else if (valid_readings == 2) {
        /* Average of two readings */
        *distance_mm = (readings[0] + readings[1]) / 2;
    } else {
        /* Only one reading */
        *distance_mm = readings[0];
    }

    return 0;
}

/* Ultrasonic sensor thread function */
static void ultra_thread(void *p1, void *p2, void *p3)
{
    uint16_t mm;
    int rc;
    uint32_t iteration = 0;
    uint32_t success_count = 0;
    uint32_t error_count = 0;

    /* Access global thread variables */
    const struct device *trigger_dev = g_trigger_dev;
    const struct device *echo_dev = g_echo_dev;
    uint8_t trigger_pin = g_trigger_pin;
    uint8_t echo_pin = g_echo_pin;

    printk("Ultrasonic thread started\n");
    printk("Trigger GPIO: %s pin %d\n", trigger_dev->name, trigger_pin);
    printk("Echo GPIO: %s pin %d\n", echo_dev->name, echo_pin);

    /* Initialize HC-SR04 with progressive backoff retry */
    int retry = 0;
    int max_retries = 5;

    while (retry < max_retries) {
        rc = hc_sr04_init(trigger_dev, trigger_pin, echo_dev, echo_pin);
        if (rc == 0) {
            break;
        }
        
        /* Progressive backoff */
        k_msleep(100 * (retry + 1));
        retry++;
    }

    if (rc != 0) {
        printk("CRITICAL ERROR: Failed to initialize HC-SR04\n");
        /* Signal main thread that sensor is ready (even though it failed) */
        atomic_set(g_system_ready, 1); // Still signal readiness for main thread
        return; // Exit thread if init fails
    }

    printk("HC-SR04 sensor ready\n");

    /* Allow system to stabilize more */
    k_msleep(1000);

    /* Signal that sensor is ready */
    atomic_set(g_system_ready, 1);

    while (1) {
        iteration++;

        /* Measure distance with progressive backoff retry on error */
        int measure_attempt = 0;
        int max_measure_attempts = 5;
        bool success = false;

        while (measure_attempt < max_measure_attempts && !success) {
            rc = hc_sr04_measure(trigger_dev, trigger_pin, echo_dev, echo_pin, &mm);

            if (rc == 0) {
                success = true;
                success_count++;

                /* Update shared state */
                atomic_set(g_last_distance_mm, mm);
            } else {
                measure_attempt++;
                error_count++;

                if (measure_attempt < max_measure_attempts) {
                    /* Progressive backoff for retries */
                    k_msleep(50 * measure_attempt);
                }
            }
        }

        if (!success) {
            if (error_count > 10 && success_count == 0) {
                printk("Too many consecutive errors, reinitializing ultrasonic sensor\n");
                hc_sr04_init(trigger_dev, trigger_pin, echo_dev, echo_pin);
                k_msleep(500);
                error_count = 0;  /* Reset error count after reinitialization */
            }
        }

        /* Wait before next measurement cycle - progressive adjustment based on success */
        if (success) {
            k_msleep(1000);  /* Measurement interval of 1000ms to match vibration sensor */
        } else {
            k_msleep(2000);  /* Longer wait after failures to allow system to recover */
        }
    }
}

/* Start the ultrasonic sensor thread */
int ultrasonic_thread_start(const struct device *trigger_dev, uint8_t trigger_pin,
                           const struct device *echo_dev, uint8_t echo_pin,
                           atomic_t *system_ready, atomic_t *last_distance_mm)
{
    /* Store parameters in global variables for thread access */
    g_trigger_dev = trigger_dev;
    g_echo_dev = echo_dev;
    g_trigger_pin = trigger_pin;
    g_echo_pin = echo_pin;
    g_system_ready = system_ready;
    g_last_distance_mm = last_distance_mm;

    /* Create thread with k_thread_create instead of using K_THREAD_DEFINE macro */
    k_tid_t tid = k_thread_create(&ultra_thread_data, ultra_stack,
                                 STACK_SIZE,
                                 ultra_thread, NULL, NULL, NULL,
                                 SENSOR_THREAD_PRIO, 0, K_MSEC(500));  /* Start with 500ms delay for synchronization */

    if (tid == NULL) {
        printk("ERROR: Failed to create ultrasonic sensor thread\n");
        return -ENOMEM;
    }

    return 0;
}

/* Get the last measured distance */
uint16_t get_last_distance(void)
{
    if (g_last_distance_mm == NULL) {
        return 0;
    }
    return (uint16_t)atomic_get(g_last_distance_mm);
} 