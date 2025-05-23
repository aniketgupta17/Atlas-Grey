#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/rtc.h>
#include <stdio.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <string.h>
#include "rtc.h"

#define VIBRATION_THRESHOLD 0.15
#define SAMPLING_INTERVAL_MS 1000
#define MAX_VIBRATION_EVENTS 50
#define GRAVITY 9.81

static int vibration_event_count = 0;
static int vibration_event_index = 0;

typedef struct {
    struct rtc_time rtc_time;
    bool rtc_valid;
    double magnitude;
    double deviation;
} vibration_event_t;


static vibration_event_t vibration_events[MAX_VIBRATION_EVENTS];

static int print_samples = 0;
static int lsm6dsl_trig_cnt = 0;
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;

void format_rtc_timestamp(const struct rtc_time *tm, char *buffer, size_t len) {
    if (tm) {
        snprintf(buffer, len, "%04d-%02d-%02d %02d:%02d:%02d",
                 tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
                 tm->tm_hour, tm->tm_min, tm->tm_sec);
    } else {
        snprintf(buffer, len, "N/A");
    }
}

#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(const struct device *dev,
                                    const struct sensor_trigger *trig) {
    struct sensor_value accel_x, accel_y, accel_z;
    lsm6dsl_trig_cnt++;

    if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ) < 0) return;

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

    double x = sensor_value_to_double(&accel_x);
    double y = sensor_value_to_double(&accel_y);
    double z = sensor_value_to_double(&accel_z);
    double magnitude = sqrt(x * x + y * y + z * z);
    double deviation = fabs(magnitude - GRAVITY);

    if (deviation > VIBRATION_THRESHOLD) {
        int i = vibration_event_index % MAX_VIBRATION_EVENTS;

        vibration_events[i].magnitude = magnitude;
        vibration_events[i].deviation = deviation;

        if (rtc_get_time(rtc, &vibration_events[i].rtc_time) == 0) {
            vibration_events[i].rtc_valid = true;
        } else {
            vibration_events[i].rtc_valid = false;
        }

        vibration_event_index++;
        if (vibration_event_count < MAX_VIBRATION_EVENTS) {
            vibration_event_count++;
        }

        printk("⚠️ Strong vibration! Magnitude: %.4f | Δ=%.4f\n", magnitude, deviation);
    }

    if (print_samples) {
        print_samples = 0;
        accel_x_out = accel_x;
        accel_y_out = accel_y;
        accel_z_out = accel_z;
    }
}
#endif

int main(void) {
    int loop_cnt = 0;

    if (!device_is_ready(rtc)) {
        printk("RTC device not ready!\n");
        return 0;
    }

    if (set_date_time(rtc) < 0) {
        printk("RTC time initialization failed\n");
    } else {
        printk("RTC time set successfully\n");
    }

    const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    if (!device_is_ready(lsm6dsl_dev)) {
        printk("Sensor: device not ready.\n");
        return 0;
    }

    struct sensor_value odr_attr = { .val1 = 1, .val2 = 0 };
    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
        printk("Failed to set ODR.\n");
        return 0;
    }

#ifdef CONFIG_LSM6DSL_TRIGGER
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_DATA_READY,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };
    if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
        printk("Trigger set failed.\n");
        return 0;
    }
#endif

    while (1) {
        printk("\033[2J\033[H"); // Clear terminal
        printk("Suspension Bridge Vibration Monitor\n\n");

        double x = sensor_value_to_double(&accel_x_out);
        double y = sensor_value_to_double(&accel_y_out);
        double z = sensor_value_to_double(&accel_z_out);
        double magnitude = sqrt(x * x + y * y + z * z);
        double deviation = fabs(magnitude - GRAVITY);

        printk("Accel X: %.4f | Y: %.4f | Z: %.4f\n", x, y, z);
        printk("Magnitude: %.4f | Deviation from gravity: %.4f\n\n", magnitude, deviation);

        printk("Logged Vibrations (max %d):\n", MAX_VIBRATION_EVENTS);
        for (int i = 0; i < vibration_event_count; i++) {
            char ts[64];
            if (vibration_events[i].rtc_valid) {
                format_rtc_timestamp(&vibration_events[i].rtc_time, ts, sizeof(ts));
            } else {
                strcpy(ts, "N/A");
            }

            printk("Event %d: %.4f m/s² | Δ=%.4f at %s\n",
                   i + 1,
                   vibration_events[i].magnitude,
                   vibration_events[i].deviation,
                   ts);
        }

        printk("\nLoop: %d | Trigger count: %d\n", ++loop_cnt, lsm6dsl_trig_cnt);

        print_samples = 1;
        k_sleep(K_MSEC(SAMPLING_INTERVAL_MS));
    }
}
