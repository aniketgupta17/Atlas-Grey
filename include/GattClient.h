#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

int gatt_sensor_ble_client_init(void);
