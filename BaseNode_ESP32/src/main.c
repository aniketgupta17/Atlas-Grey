/* main.c
 * BLE relay on ESP32 DevKit: receives packets from Disco GATT server,
 * logs via printk, and rebroadcasts via GATT continuously
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <zephyr/net/buf.h>        /* for net_buf_simple */

#include <string.h>
#include <zephyr/sys/util.h>

#define READ_INTERVAL_MS   1000
#define RELAY_QUEUE_SIZE   10
#define RELAY_MSG_SIZE     64

/* Forward declarations */
static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid);
static void start_scan(void);
static void start_advertising(void);

/* Relay message queue */
K_MSGQ_DEFINE(relay_msgq, RELAY_MSG_SIZE, RELAY_QUEUE_SIZE, 4);

/* Buffer for last received message */
static char last_msg[RELAY_MSG_SIZE] = "Waiting for data...";

/* UUIDs */
static struct bt_uuid_128 disco_svc_uuid  = BT_UUID_INIT_128(
    0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,
    0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11);
static struct bt_uuid_128 disco_char_uuid = BT_UUID_INIT_128(
    0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,
    0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22);
static struct bt_uuid_128 relay_svc_uuid   = BT_UUID_INIT_128(
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33);
static struct bt_uuid_128 relay_char_uuid  = BT_UUID_INIT_128(
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44);

/* GATT client context */
static struct bt_conn *default_conn;
static uint16_t disco_char_handle;
static struct k_work_delayable read_work;
static struct bt_gatt_read_params read_params_global;

/* Relay GATT service (notify) */
static void relay_ccc_cfg(const struct bt_gatt_attr *attr, uint16_t value) { }
BT_GATT_SERVICE_DEFINE(relay_svc,
    BT_GATT_PRIMARY_SERVICE(&relay_svc_uuid),
    BT_GATT_CHARACTERISTIC(&relay_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(relay_ccc_cfg, BT_GATT_PERM_READ),
);

/* --- Advertising for the relay service --- */
static void start_advertising(void)
{
    printk("[RELAY] start_advertising() called\n");
    struct bt_le_adv_param adv_params = BT_LE_ADV_PARAM_INIT(
        BT_LE_ADV_OPT_CONNECTABLE,
        BT_GAP_ADV_FAST_INT_MIN_2,
        BT_GAP_ADV_FAST_INT_MAX_2,
        NULL);

    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS,
                      BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
            0x33,0x33,0x33,0x33, 0x33,0x33,0x33,0x33,
            0x33,0x33,0x33,0x33, 0x33,0x33,0x33,0x33)
    };

    int err = bt_le_adv_start(&adv_params,
                              ad, ARRAY_SIZE(ad),
                              NULL, 0);
    printk("[RELAY] bt_le_adv_start returned %d\n", err);
}

/* --- Simple UUID parser in advertisement packets --- */
static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid)
{
    struct bt_data data;
    struct net_buf_simple_state state;
    net_buf_simple_save(ad, &state);

    while (ad->len > 1) {
        uint8_t len  = net_buf_simple_pull_u8(ad);
        uint8_t type = net_buf_simple_pull_u8(ad);

        if (len < 1 || len - 1 > ad->len) {
            break;
        }
        data.type     = type;
        data.data_len = len - 1;
        data.data     = ad->data;

        if ((type == BT_DATA_UUID128_ALL ||
             type == BT_DATA_UUID128_SOME) &&
            data.data_len == 16) {
            struct bt_uuid_128 adv_uuid;
            bt_uuid_create(&adv_uuid.uuid, data.data, 16);
            if (!bt_uuid_cmp(uuid, &adv_uuid.uuid)) {
                net_buf_simple_restore(ad, &state);
                return true;
            }
        }
        net_buf_simple_pull(ad, len - 1);
    }

    net_buf_simple_restore(ad, &state);
    return false;
}

/* --- Relay thread: pop from queue and send notifications --- */
void relay_thread(void)
{
    char msg[RELAY_MSG_SIZE];
    while (1) {
        k_msgq_get(&relay_msgq, msg, K_FOREVER);
        printk("[RELAY] Notified: %s\n", msg);
        bt_gatt_notify(NULL, &relay_svc.attrs[1],
                       msg, strlen(msg));
    }
}
K_THREAD_DEFINE(relay_tid, 1024, relay_thread, NULL, NULL, NULL, 5, 0, 0);

/* --- GATT read callback --- */
static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
                       struct bt_gatt_read_params *p,
                       const void *data, uint16_t length)
{
    printk("[CLIENT] read_cb err=%u length=%u\n", err, length);
    if (!err && data && length) {
        char buf[RELAY_MSG_SIZE];
        size_t ml = MIN(length, RELAY_MSG_SIZE - 1);
        memcpy(buf, data, ml);
        buf[ml] = '\0';
        printk("[CLIENT] Read: %s\n", buf);
        k_msgq_put(&relay_msgq, buf, K_NO_WAIT);
        strncpy(last_msg, buf, RELAY_MSG_SIZE);
    }
    /* schedule next read */
    k_work_schedule(&read_work, K_MSEC(READ_INTERVAL_MS));
    return BT_GATT_ITER_STOP;
}

/* --- do_read: issue the GATT read --- */
static void do_read(struct k_work *work)
{
    printk("[CLIENT] scheduling bt_gatt_read\n");
    memset(&read_params_global, 0, sizeof(read_params_global));
    read_params_global.func         = read_cb;
    read_params_global.handle_count = 1;
    read_params_global.single.handle = disco_char_handle;
    read_params_global.single.offset = 0;
    int err = bt_gatt_read(default_conn, &read_params_global);
    printk("[CLIENT] bt_gatt_read returned %d\n", err);
}

/* --- GATT discovery callback --- */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("[CLIENT] Discovery complete\n");
        return BT_GATT_ITER_STOP;
    }
    struct bt_gatt_chrc *chrc = (void *)attr->user_data;
    if (!bt_uuid_cmp(params->uuid, &disco_char_uuid.uuid)) {
        disco_char_handle = chrc->value_handle;
        printk("[CLIENT] Found Disco char handle %u\n", disco_char_handle);
        k_work_init_delayable(&read_work, do_read);
        k_work_schedule(&read_work, K_NO_WAIT);
        return BT_GATT_ITER_STOP;
    }
    return BT_GATT_ITER_CONTINUE;
}

/* --- Connection callbacks --- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    printk("[CLIENT] connected(err=%u)\n", err);
    if (err) { return; }

    default_conn = bt_conn_ref(conn);

    static struct bt_gatt_discover_params discov = {0};
    discov.uuid        = &disco_char_uuid.uuid;
    discov.func        = discover_func;
    discov.start_handle = 0x0001;
    discov.end_handle   = 0xffff;
    discov.type        = BT_GATT_DISCOVER_CHARACTERISTIC;

    int d_err = bt_gatt_discover(default_conn, &discov);
    printk("[CLIENT] bt_gatt_discover returned %d\n", d_err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("[CLIENT] disconnected(reason=%u)\n", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    k_work_cancel_delayable(&read_work);
    start_scan();
}

static struct bt_conn_cb conn_callbacks = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* --- Scan callback: look for Disco service and connect --- */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *ad)
{
    printk("[CLIENT] scan_cb type=%u rssi=%d len=%u\n",
           adv_type, rssi, ad->len);
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND) {
        return;
    }
    if (!adv_has_uuid(ad, &disco_svc_uuid.uuid)) {
        return;
    }

    printk("[CLIENT] Found Disco, stopping scan\n");
    bt_le_scan_stop();

    bt_conn_le_create(addr,
                      BT_CONN_LE_CREATE_CONN,
                      BT_LE_CONN_PARAM_DEFAULT,
                      &default_conn);
}

/* --- Start scanning for Disco --- */
static void start_scan(void)
{
    printk("[CLIENT] Starting scan\n");
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    printk("[CLIENT] bt_le_scan_start returned %d\n", err);
}

/* --- Called when BT stack is ready --- */
static void bt_ready(int err)
{
    printk("[MAIN] bt_ready(err=%d)\n", err);
    if (err) {
        printk("[ERROR] Bluetooth init failed\n");
        return;
    }

    bt_conn_cb_register(&conn_callbacks);

    /* As a _client_, start scanning for Disco immediately */
    start_scan();

    /* As a _server_, start advertising our relay service */
    start_advertising();
}

int main(void)
{
    printk("Starting BLE client+relay on ESP32 DevKit\n");
    int err = bt_enable(bt_ready);
    printk("[MAIN] bt_enable returned %d\n", err);

    /* No display init needed */

    /* Main loop just sleeps; all events handled asynchronously */
    while (1) {
        k_sleep(K_SECONDS(1));
    }

    return 0;
}
