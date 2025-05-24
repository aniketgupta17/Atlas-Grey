#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "GattServer.h"
#include "GattClient.h"

void switch_to_server(void)
{
    printk("Read successful. Switching to server role...\n");


    int err = gatt_sensor_ble_server_init();
    if (err) {
        printk("Server init failed! (err %d)\n", err);
    } else {
        printk("Switched to server role successfully.\n");
    }
}

void main(void)
{
    printk("Booting: Main application\n");

    int err = gatt_sensor_ble_client_init(switch_to_server);
    if (err) {
        printk("BLE Client init failed! (err %d)\n", err);
        return;
    }

    printk("BLE Client initialized. Waiting for read...\n");
}
