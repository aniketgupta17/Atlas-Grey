#include "GattClient.h"

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad);

static struct bt_conn *default_conn;
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_read_params read_params;
static bool is_scanning = false;
static uint8_t connection_retries = 0;
static uint8_t discovery_retries = 0;
static gatt_client_done_cb_t client_done_cb = NULL;

#define MAX_RETRIES 3
#define MAX_DISCOVERY_RETRIES 2
#define RETRY_BACKOFF_MS 500 

static struct bt_uuid_128 target_uuid = BT_UUID_INIT_128(
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11,
    0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11);

static struct bt_uuid_128 custom_char_uuid = BT_UUID_INIT_128(
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22);

static uint16_t char_handle;
static bool had_successful_read = true;
static bt_addr_le_t last_connected_addr;

static void reset_state(void)
{
    had_successful_read = false;
    connection_retries = 0;
    discovery_retries = 0;
    char_handle = 0;
    memset(&discover_params, 0, sizeof(discover_params));
    memset(&read_params, 0, sizeof(read_params));
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

static uint8_t read_func(struct bt_conn *conn, uint8_t err,
                         struct bt_gatt_read_params *params,
                         const void *data, uint16_t length)
{
    if (err) {
        printk("Read failed with error %u\n", err);
        goto disconnect;
    }

    if (!data || length == 0) {
        printk("Read completed with no data\n");
        goto disconnect;
    }

    char msg_buf[128];
    if (length >= sizeof(msg_buf)) length = sizeof(msg_buf) - 1;

    memcpy(msg_buf, data, length);
    msg_buf[length] = '\0';

    printk("Received string: %s\n", msg_buf);
    had_successful_read = true;

disconnect:
    int discon_err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if (discon_err) {
        printk("Failed to disconnect (err %d)\n", discon_err);
    } else {
        printk("Disconnected after successful read\n");
    }

    return BT_GATT_ITER_STOP;
}

static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("Discover complete, no more attributes found\n");
        discovery_retries++;
        if (discovery_retries < MAX_DISCOVERY_RETRIES && default_conn) {
            printk("Retrying discovery (attempt %d/%d)\n", discovery_retries + 1, MAX_DISCOVERY_RETRIES);
            discover_params.start_handle = 0x0001;
            int err_disc = bt_gatt_discover(default_conn, &discover_params);
            if (err_disc) {
                printk("Retry discovery failed (err %d)\n", err_disc);
            }
            return BT_GATT_ITER_STOP;
        }
        goto disconnect;
    }

    printk("Discovered attribute handle %u\n", attr->handle);

    if (bt_uuid_cmp(params->uuid, &custom_char_uuid.uuid) == 0) {
        struct bt_gatt_chrc *chrc = (struct bt_gatt_chrc *)attr->user_data;
        char_handle = chrc->value_handle;

        printk("Characteristic value handle: %u\n", char_handle);

        read_params.func = read_func;
        read_params.handle_count = 1;
        read_params.single.handle = char_handle;
        read_params.single.offset = 0;

        int err = bt_gatt_read(conn, &read_params);
        if (err) {
            printk("Failed to initiate GATT read (err %d)\n", err);
            goto disconnect;
        } else {
            printk("GATT read initiated for handle %u\n", char_handle);
        }
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;

disconnect:
    int discon_err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    if (discon_err) {
        printk("Failed to disconnect (err %d)\n", discon_err);
    } else {
        printk("Disconnected after discovery failure\n");
    }
    return BT_GATT_ITER_STOP;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("Connection failed (err %u)\n", err);
        connection_retries++;
        if (connection_retries < MAX_RETRIES && is_scanning) {
            printk("Retrying connection (attempt %d/%d) after %dms\n", connection_retries + 1, MAX_RETRIES, RETRY_BACKOFF_MS);
            k_msleep(RETRY_BACKOFF_MS); // Backoff before retry
            return;
        } else {
            printk("Max connection retries reached or scanning stopped\n");
            reset_state();
            is_scanning = false;
            k_msleep(RETRY_BACKOFF_MS); // Backoff before resuming scan
            int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
            if (scan_err) {
                printk("Scanning failed to start (err %d)\n", scan_err);
            } else {
                is_scanning = true;
            }
        }
        return;
    }

    printk("Connected\n");
    connection_retries = 0;
    discovery_retries = 0;
    default_conn = bt_conn_ref(conn);

    k_msleep(100); // Stabilize connection

    discover_params.uuid = &custom_char_uuid.uuid;
    discover_params.func = discover_func;
    discover_params.start_handle = 0x0001;
    discover_params.end_handle = 0xffff;
    discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err_disc = bt_gatt_discover(default_conn, &discover_params);
    if (err_disc) {
        printk("Discover failed (err %d)\n", err_disc);
        int discon_err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (discon_err) {
            printk("Failed to disconnect (err %d)\n", discon_err);
        } else {
            printk("Disconnected after discovery failure\n");
        }
    } else {
        printk("GATT discovery initiated\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    reset_state();

    if (had_successful_read) {
        printk("Resuming scan for new devices after %dms...\n", RETRY_BACKOFF_MS);
        k_msleep(RETRY_BACKOFF_MS); 
        int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
        if (scan_err) {
            printk("Scanning failed to start (err %d)\n", scan_err);
        } else {
            is_scanning = true;
        }
    }
            if (client_done_cb) {
            client_done_cb();
            client_done_cb = NULL;
        }
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid)
{
    struct bt_data data;
    struct net_buf_simple_state state;

    net_buf_simple_save(ad, &state);

    while (ad->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        uint8_t type = net_buf_simple_pull_u8(ad);

        if (len == 0 || len > ad->len + 1) break;

        data.type = type;
        data.data_len = len - 1;
        data.data = ad->data;

        if (type == BT_DATA_UUID128_ALL && data.data_len == 16) {
            struct bt_uuid_128 adv_uuid;
            bt_uuid_create(&adv_uuid.uuid, data.data, 16);
            if (bt_uuid_cmp(uuid, &adv_uuid.uuid) == 0) {
                net_buf_simple_restore(ad, &state);
                return true;
            }
        }

        net_buf_simple_pull(ad, len - 1);
    }

    net_buf_simple_restore(ad, &state);
    return false;
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi,
                         uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) return;

    if (!adv_has_uuid(ad, &target_uuid.uuid)) return;

    if (had_successful_read && bt_addr_le_cmp(addr, &last_connected_addr) == 0) {
        printk("Skipping previously read device: %s\n", addr_str);
        return;
    }

    printk("Found device with target UUID: %s (RSSI %d)\n", addr_str, rssi);
    bt_addr_le_copy(&last_connected_addr, addr);

    if (is_scanning) {
        int err = bt_le_scan_stop();
        if (err) {
            printk("Failed to stop scanning (err %d)\n", err);
            return;
        }
        is_scanning = false;
        printk("Scanning stopped\n");
    }

    static const struct bt_le_conn_param conn_params = {
        .interval_min = 24,
        .interval_max = 40,
        .latency = 0,
        .timeout = 1000,
    };

    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, &conn_params, &default_conn);
    if (err) {
        printk("Create connection failed (err %d)\n", err);
        connection_retries++;
        if (connection_retries < MAX_RETRIES) {
            printk("Retrying connection (attempt %d/%d) after %dms\n", connection_retries + 1, MAX_RETRIES, RETRY_BACKOFF_MS);
            k_msleep(RETRY_BACKOFF_MS); // Backoff before retry
            int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
            if (scan_err) {
                printk("Scanning failed to start (err %d)\n", scan_err);
            } else {
                is_scanning = true;
            }
        } else {
            printk("Max connection retries reached\n");
            reset_state();
            k_msleep(RETRY_BACKOFF_MS); // Backoff before resuming scan
            int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
            if (scan_err) {
                printk("Scanning failed to start (err %d)\n", scan_err);
            } else {
                is_scanning = true;
            }
        }
    } else {
        printk("Connection pending\n");
    }
}

static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");
    bt_conn_cb_register(&conn_callbacks);

    int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, device_found);
    if (scan_err) {
        printk("Scanning failed to start (err %d)\n", scan_err);
        return;
    }

    is_scanning = true;
    printk("Scanning started\n");

}


int gatt_sensor_ble_client_init(gatt_client_done_cb_t done_cb)
{
    printk("Booting: BLE client\n");

    client_done_cb = done_cb;  // Save the callback

    int err = bt_enable(bt_ready);  // bt_ready is called when BLE stack is up
    if (err) {
        printk("Bluetooth enable failed (err %d)\n", err);
        return err;
    }

    return 0;
}





