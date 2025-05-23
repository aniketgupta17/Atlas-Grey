// #include <zephyr/kernel.h>
// #include <zephyr/bluetooth/bluetooth.h>
// #include <zephyr/bluetooth/gatt.h>
// #include <zephyr/bluetooth/conn.h>
// #include <zephyr/bluetooth/hci.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/sys/byteorder.h>

// // Function prototype for device_found
// static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);

// static struct bt_conn *default_conn;
// static struct bt_gatt_discover_params discover_params;
// static struct bt_gatt_read_params read_params;
// static bool is_scanning = false;
// static uint8_t connection_retries = 0;
// static uint8_t discovery_retries = 0;
// #define MAX_RETRIES 3
// #define MAX_DISCOVERY_RETRIES 2

// static struct bt_uuid_128 target_uuid = BT_UUID_INIT_128(
//     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
//     0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);

// static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
//     0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
//     0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22);

// static uint16_t char_handle;

// static uint8_t read_func(struct bt_conn *conn, uint8_t err,
//                          struct bt_gatt_read_params *params,
//                          const void *data, uint16_t length)
// {
//     if (err) {
//         printk("Read failed with error %u\n", err);
//         return BT_GATT_ITER_STOP;
//     }
//     if (!data || length == 0) {
//         printk("Read completed with no data (length=%u)\n", length);
//         return BT_GATT_ITER_STOP;
//     }

// if (length < 128 && memchr(data, '\0', length) == NULL) {
//     printk("Read data successful: length=%u, data=%.*s\n", length, length, (const char *)data);
// } else {
//     printk("Read data successful: length=%u, data (hex)=", length);
//     for (uint16_t i = 0; i < length; i++) {
//         printk("%02x ", ((uint8_t *)data)[i]);
//     }
//     printk("\n");
// }

//     int discon_err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
//     if (discon_err) {
//         printk("Failed to disconnect (err %d)\n", discon_err);
//     } else {
//         printk("Disconnected after successful read\n");
//     }

//     return BT_GATT_ITER_STOP;
// }
// static uint8_t discover_func(struct bt_conn *conn,
//                              const struct bt_gatt_attr *attr,
//                              struct bt_gatt_discover_params *params)
// {
//     if (!attr) {
//         printk("Discover complete, no more attributes found\n");
//         discovery_retries++;
//         if (discovery_retries < MAX_DISCOVERY_RETRIES && default_conn) {
//             printk("Retrying discovery (attempt %d/%d)\n", discovery_retries + 1, MAX_DISCOVERY_RETRIES);
//             discover_params.start_handle = 0x0001;
//             int err_disc = bt_gatt_discover(default_conn, &discover_params);
//             if (err_disc) {
//                 printk("Retry discovery failed (err %d)\n", err_disc);
//             }
//             return BT_GATT_ITER_STOP;
//         }
//         return BT_GATT_ITER_STOP;
//     }

//     printk("Discovered attribute handle %u\n", attr->handle);

//     if (bt_uuid_cmp(params->uuid, &custom_char_uuid.uuid) == 0) {
//         // Extract characteristic value handle from attr->user_data
//         struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
//         char_handle = chrc->value_handle;  // <-- This is the handle to read from

//         printk("Characteristic value handle: %u\n", char_handle);

//         read_params.func = read_func;
//         read_params.handle_count = 1;
//         read_params.single.handle = char_handle;
//         read_params.single.offset = 0;

//         int err = bt_gatt_read(conn, &read_params);
//         if (err) {
//             printk("Failed to initiate GATT read (err %d)\n", err);
//         } else {
//             printk("GATT read initiated for handle %u\n", char_handle);
//         }
//         return BT_GATT_ITER_STOP;
//     }

//     return BT_GATT_ITER_CONTINUE;
// }


// static void connected(struct bt_conn *conn, uint8_t err)
// {
//     if (err) {
//         printk("Connection failed (err %u)\n", err);
//         connection_retries++;
//         if (connection_retries < MAX_RETRIES && is_scanning) {
//             printk("Retrying connection (attempt %d/%d)\n", connection_retries + 1, MAX_RETRIES);
//             return;
//         } else {
//             printk("Max connection retries reached or scanning stopped\n");
//             is_scanning = false;
//         }
//         return;
//     }
//     printk("Connected\n");
//     connection_retries = 0;
//     discovery_retries = 0;
//     default_conn = bt_conn_ref(conn);

//     // Add delay to stabilize connection before discovery
//     k_msleep(100);

//     discover_params.uuid = &custom_char_uuid.uuid;
//     discover_params.func = discover_func;
//     discover_params.start_handle = 0x0001;
//     discover_params.end_handle = 0xffff;
//     discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    

//     int err_disc = bt_gatt_discover(default_conn, &discover_params);
//     if (err_disc) {
//         printk("Discover failed (err %d)\n", err_disc);
//     } else {
//         printk("GATT discovery initiated\n");
//     }
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
//     printk("Disconnected (reason %u)\n", reason);
//     if (default_conn) {
//         bt_conn_unref(default_conn);
//         default_conn = NULL;
//     }
//     if (connection_retries < MAX_RETRIES && !is_scanning) {
//         int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
//         if (scan_err) {
//             printk("Scanning failed to start (err %d)\n", scan_err);
//         } else {
//             printk("Resumed scanning after disconnection\n");
//             is_scanning = true;
//         }
//     }
// }

// static struct bt_conn_cb conn_callbacks = {
//     .connected = connected,
//     .disconnected = disconnected,
// };

// static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid)
// {
//     struct bt_data data;
//     struct net_buf_simple_state state;

//     net_buf_simple_save(ad, &state);

//     while (ad->len > 1) {
//         uint8_t len = net_buf_simple_pull_u8(ad);
//         uint8_t type = net_buf_simple_pull_u8(ad);

//         if (len == 0 || len > ad->len + 1) {
//             break;
//         }

//         data.type = type;
//         data.data_len = len - 1;
//         data.data = ad->data;

//         if (type == BT_DATA_UUID128_ALL && data.data_len == 16) {
//             uint8_t uuid_val[16];
//             memcpy(uuid_val, data.data, 16);
//             struct bt_uuid_128 adv_uuid = BT_UUID_INIT_128(0);
//             bt_uuid_create(&adv_uuid.uuid, uuid_val, 16);
//             if (bt_uuid_cmp(uuid, &adv_uuid.uuid) == 0) {
//                 net_buf_simple_restore(ad, &state);
//                 return true;
//             }
//         }

//         net_buf_simple_pull(ad, len - 1);
//     }

//     net_buf_simple_restore(ad, &state);
//     return false;
// }

// static void device_found(const bt_addr_le_t *addr, int8_t rssi,
//                          uint8_t type, struct net_buf_simple *ad)
// {
//     char addr_str[BT_ADDR_LE_STR_LEN];
//     bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

//     if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
//         if (adv_has_uuid(ad, &target_uuid.uuid)) {
//             printk("Found device with target service UUID: %s (RSSI %d)\n", addr_str, rssi);

//             if (is_scanning) {
//                 int err = bt_le_scan_stop();
//                 if (err) {
//                     printk("Failed to stop scanning (err %d)\n", err);
//                     return;
//                 }
//                 is_scanning = false;
//                 printk("Scanning stopped\n");
//             }

//             static const struct bt_le_conn_param conn_params = {
//                 .interval_min = 24, // 30ms
//                 .interval_max = 40, // 50ms
//                 .latency = 0,
//                 .timeout = 1000, // 10s
//             };

//             int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_params, &default_conn);
//             if (err) {
//                 printk("Create connection failed (err %d)\n", err);
//                 connection_retries++;
//                 if (connection_retries < MAX_RETRIES) {
//                     printk("Retrying connection (attempt %d/%d)\n", connection_retries + 1, MAX_RETRIES);
//                     err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
//                     if (err) {
//                         printk("Scanning failed to start (err %d)\n", err);
//                     } else {
//                         is_scanning = true;
//                         printk("Scanning resumed for retry\n");
//                     }
//                 } else {
//                     printk("Max connection retries reached\n");
//                 }
//             } else {
//                 printk("Connection pending\n");
//             }
//         }
//     }
// }

// static void bt_ready(int err)
// {
//     if (err) {
//         printk("Bluetooth init failed (err %d)\n", err);
//         return;
//     }

//     printk("Bluetooth initialized\n");

//     bt_conn_cb_register(&conn_callbacks);

//     int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
//     if (scan_err) {
//         printk("Scanning failed to start (err %d)\n", scan_err);
//         return;
//     }
//     is_scanning = true;
//     printk("Scanning started\n");
// }

// void main(void)
// {
//     printk("Booting: BLE client\n");
//     int err = bt_enable(bt_ready);
//     if (err) {
//         printk("Bluetooth enable failed (err %d)\n", err);
//     }
// }

// main.c
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "GattServer.h" // Include your own header file first
#include "GattClient.h" // Include your own header file first

void main(void)
{
    printk("Booting: Main application\n");

    int err = gatt_sensor_ble_client_init();
    if (err) {
        printk("BLE initialization failed! (err %d)\n", err);
        return;
    }

    printk("BLE sensor module initialized successfully.\n");
    // Your main application logic can continue here,
    // or you can go into an infinite loop if the BLE events
    // are the primary focus and handled by callbacks.
    // For many Zephyr BLE apps, main just sets things up and then idles.
}