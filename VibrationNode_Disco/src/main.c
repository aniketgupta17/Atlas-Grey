#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rtc.h"

#define DEVICE_NAME "vibrationsensor"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define MAX_VIBRATION_EVENTS 10
#define GRAVITY 9.81
#define VIBRATION_THRESHOLD 0.2
#define RMS_THRESHOLD 9.8
#define SAMPLING_INTERVAL_MS 1000

typedef struct {
    struct rtc_time rtc_time;
    bool rtc_valid;
    double magnitude;
    double deviation;
} vibration_event_t;

static vibration_event_t vibration_events[MAX_VIBRATION_EVENTS];
static int vibration_event_count = 0;      // number of valid entries
static int vibration_event_index = 0;      // circular buffer write index

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;

// UUIDs
static struct bt_uuid_128 custom_svc_uuid = BT_UUID_INIT_128(
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);
static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22);


static double last_rms = 0.0;
static char last_msg[32];

double calculate_rms(int count) {
    if (count == 0) {
        return 0.0;
    }

    double sum_squares = 0.0;
    for (int i = 0; i < count; i++) {
        sum_squares += pow(vibration_events[i].magnitude, 2);
    }
    return sqrt(sum_squares / count);
}


static ssize_t read_ble_message(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    printk("BLE read: %s\n", last_msg);
    return bt_gatt_attr_read(conn, attr, buf, len, offset, last_msg, strlen(last_msg) + 1);
}


BT_GATT_SERVICE_DEFINE(hello_svc,
    BT_GATT_PRIMARY_SERVICE(&custom_svc_uuid),
    BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_ble_message, NULL, NULL),
);

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
        0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Connected: %s (err=%u)\n", addr_str, err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Disconnected: %s (reason=%u)\n", addr_str, reason);

    int adv_err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (adv_err) {
        printk("Restart advertising failed (err %d)\n", adv_err);
    } else {
        printk("Advertising restarted\n");
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    bt_conn_cb_register(&conn_callbacks);

    int adv_err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (adv_err) {
        printk("Advertising start failed (err %d)\n", adv_err);
    } else {
        printk("Advertising started as \"%s\"\n", DEVICE_NAME);
    }
}

#ifdef CONFIG_LSM6DSL_TRIGGER
static void vibration_trigger_handler(const struct device *dev,
                                      const struct sensor_trigger *trig)
{
    struct sensor_value x, y, z;
    if (sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        return;
    }

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &x);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &y);
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &z);

    double dx = sensor_value_to_double(&x);
    double dy = sensor_value_to_double(&y);
    double dz = sensor_value_to_double(&z);
    double mag = sqrt(dx * dx + dy * dy + dz * dz);
    double deviation = fabs(mag - GRAVITY);

    if (deviation > VIBRATION_THRESHOLD) {
        int i = vibration_event_index % MAX_VIBRATION_EVENTS;
        vibration_events[i].magnitude = mag;
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

        printk("Vibration detected: Mag=%.4f Δ=%.4f\n", mag, deviation);
    }

        if (vibration_event_count == MAX_VIBRATION_EVENTS) {
            // Only update last_rms after FULL batch collected
            last_rms = calculate_rms(MAX_VIBRATION_EVENTS);

            // Get timestamp from the latest entry
            struct rtc_time *time_info = &vibration_events[(vibration_event_index - 1 + MAX_VIBRATION_EVENTS) % MAX_VIBRATION_EVENTS].rtc_time;

            if (vibration_events[(vibration_event_index - 1 + MAX_VIBRATION_EVENTS) % MAX_VIBRATION_EVENTS].rtc_valid) {
                snprintf(last_msg, sizeof(last_msg), "%.2f@%02d:%02d:%02d", 
                        last_rms,
                        time_info->tm_hour,
                        time_info->tm_min,
                        time_info->tm_sec);
            } else {
                snprintf(last_msg, sizeof(last_msg), "%.2f@NoTime", last_rms);
            }


            printk("STDDEV of last 10 vibrations: %.4f\n", last_rms);
            printk("Logs and STDDEV reset after 10 readings.\n");

            // Reset count & index for next batch
            vibration_event_count = 0;
            vibration_event_index = 0;
            memset(vibration_events, 0, sizeof(vibration_events));
        }

    accel_x_out = x;
    accel_y_out = y;
    accel_z_out = z;
}
#endif
#define BUFFER_SIZE 10

int main(void)
{
    printk("Starting BLE + Vibration Sensor\n");

    if (bt_enable(bt_ready) != 0) {
        printk("BLE enable failed\n");
        return 0;
    }

    if (!device_is_ready(rtc)) {
        printk("RTC not ready\n");
        return 0;
    }

    set_date_time(rtc);

    const struct device *sensor_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
    if (!device_is_ready(sensor_dev)) {
        printk("Accelerometer not ready\n");
        return 0;
    }

    struct sensor_value odr = { .val1 = 1 };
    sensor_attr_set(sensor_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

    while (1) {
        if (sensor_sample_fetch_chan(sensor_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
            printk("Sample fetch failed\n");
            continue;
        }

        sensor_channel_get(sensor_dev, SENSOR_CHAN_ACCEL_X, &accel_x_out);
        sensor_channel_get(sensor_dev, SENSOR_CHAN_ACCEL_Y, &accel_y_out);
        sensor_channel_get(sensor_dev, SENSOR_CHAN_ACCEL_Z, &accel_z_out);

        double dx = sensor_value_to_double(&accel_x_out);
        double dy = sensor_value_to_double(&accel_y_out);
        double dz = sensor_value_to_double(&accel_z_out);

        double mag = sqrt(dx * dx + dy * dy + dz * dz);
        double deviation = fabs(mag - GRAVITY);

        int i = vibration_event_index % MAX_VIBRATION_EVENTS;
        vibration_events[i].magnitude = mag;
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

        if (deviation > VIBRATION_THRESHOLD) {
            printk("Vibration detected: Mag=%.4f Δ=%.4f\n", mag, deviation);
        }

        if (vibration_event_count == MAX_VIBRATION_EVENTS) {
            last_rms = calculate_rms(MAX_VIBRATION_EVENTS);
            struct rtc_time *time_info = &vibration_events[(vibration_event_index - 1 + MAX_VIBRATION_EVENTS) % MAX_VIBRATION_EVENTS].rtc_time;

            if (vibration_events[(vibration_event_index - 1 + MAX_VIBRATION_EVENTS) % MAX_VIBRATION_EVENTS].rtc_valid) {
                snprintf(last_msg, sizeof(last_msg), "%.2f@%02d:%02d:%02d", 
                        last_rms,
                        time_info->tm_hour,
                        time_info->tm_min,
                        time_info->tm_sec);
            } else {
                snprintf(last_msg, sizeof(last_msg), "%.4f@NoTime", last_rms);
            }

            printk("RMS of last %d samples: %.4f\n", MAX_VIBRATION_EVENTS, last_rms);

            vibration_event_count = 0;
            vibration_event_index = 0;
            memset(vibration_events, 0, sizeof(vibration_events));
        }

        k_sleep(K_MSEC(SAMPLING_INTERVAL_MS));  
    }

    return 0;
}
