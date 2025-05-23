// vibrationsensor_ble.c
#include "GattServer.h" // Include your own header file first

// Internal includes (if not already in your header, but for Zephyr, they often are)
// #include <zephyr/kernel.h>
// #include <zephyr/bluetooth/bluetooth.h>
// ... (all original includes)

#define DEVICE_NAME "vibrationsensor"
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// 128-bit custom service UUID: 11111111-1111-1111-1111-111111111111
static struct bt_uuid_128 custom_svc_uuid = BT_UUID_INIT_128(
    0x11, 0x11, 0x11, 0x11,
    0x11, 0x11,
    0x11, 0x11,
    0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11);

// 128-bit custom characteristic UUID: 22222222-2222-2222-2222-222222222222
static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
    0x22, 0x22, 0x22, 0x22,
    0x22, 0x22,
    0x22, 0x22,
    0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22);

// Read callback: returns "Hello"
static ssize_t read_hello(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    const char *value = "Hello";
    printk("read_hello called\n");

    // Consider if you really want to stop advertising on every read.
    // In a real application, you might want to restart advertising in disconnected
    // or keep it running if you allow multiple connections or rapid re-connections.
    int err = bt_le_adv_stop();
    if (err) {
        printk("Failed to stop advertising (err %d)\n", err);
    }

    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value) + 1);
}

// GATT Service Definition
BT_GATT_SERVICE_DEFINE(hello_svc,
    BT_GATT_PRIMARY_SERVICE(&custom_svc_uuid),
    BT_GATT_CHARACTERISTIC(&custom_char_uuid.uuid, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_hello, NULL, NULL),
);

// Advertising Data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0x11, 0x11, 0x11, 0x11,
        0x11, 0x11,
        0x11, 0x11,
        0x11, 0x11,
        0x11, 0x11, 0x11, 0x11, 0x11, 0x11),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// Connection Callbacks
static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Connected to %s (err=%u)\n", addr_str, err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    printk("Disconnected from %s (reason=%u)\n", addr_str, reason);

    // After disconnection, you might want to restart advertising to allow new connections
    // For now, let's keep it as is, but this is a common place to restart advertising.
    int adv_err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (adv_err) {
        printk("Advertising failed to restart (err %d)\n", adv_err);
    } else {
        printk("Advertising restarted after disconnection.\n");
    }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};



// Bluetooth ready callback
static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized. Registering connection callbacks...\n");
    bt_conn_cb_register(&conn_callbacks);

    int adv_err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (adv_err) {
        printk("Advertising failed to start (err %d)\n", adv_err);
        return;
    }

    printk("Advertising as \"%s\" with custom UUID\n", DEVICE_NAME);
}

// Public initialization function
int gatt_sensor_ble_server_init(void)
{
    printk("Initializing BLE vibration sensor server...\n");

    int err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return err;
    }
    return 0; // Success
}