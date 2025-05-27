/*
 * main.c
 * BLE relay on M5Core2: receives packets from Disco GATT server,
 * displays on-screen, and rebroadcasts via GATT continuously
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <string.h>
#include <zephyr/sys/util.h>
#include <zephyr/net_buf.h>

#define READ_INTERVAL_MS 1000
#define RELAY_QUEUE_SIZE 10
#define RELAY_MSG_SIZE   64

/* Forward declarations */
static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid);
static void start_scan(void);

/* Queue for values to rebroadcast */
K_MSGQ_DEFINE(relay_msgq, RELAY_MSG_SIZE, RELAY_QUEUE_SIZE, 4);
static char display_buf[RELAY_MSG_SIZE] = "Waiting for data...";
static lv_obj_t *data_label;

/* UUIDs */
static struct bt_uuid_128 disco_svc_uuid = BT_UUID_INIT_128(
    0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,
    0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11);
static struct bt_uuid_128 disco_char_uuid = BT_UUID_INIT_128(
    0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,
    0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22);
static struct bt_uuid_128 relay_svc_uuid = BT_UUID_INIT_128(
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33);
static struct bt_uuid_128 relay_char_uuid = BT_UUID_INIT_128(
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44);

/* GATT context */
static struct bt_conn *default_conn;
static uint16_t char_handle;
static struct k_work_delayable read_work;
static struct bt_gatt_read_params read_params_global;

/* Relay service for notifying subscribers */
static void relay_ccc_cfg(const struct bt_gatt_attr *attr, uint16_t value) {}
BT_GATT_SERVICE_DEFINE(relay_svc,
    BT_GATT_PRIMARY_SERVICE(&relay_svc_uuid),
    BT_GATT_CHARACTERISTIC(&relay_char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(relay_ccc_cfg, BT_GATT_PERM_READ),
);

/* Advertisement parser implementation */
static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid) {
    struct bt_data data;
    struct net_buf_simple_state state;
    net_buf_simple_save(ad, &state);
    while (ad->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        uint8_t type = net_buf_simple_pull_u8(ad);
        if (len < 1 || len - 1 > ad->len) break;
        data.type = type;
        data.data_len = len - 1;
        data.data = ad->data;
        if ((type == BT_DATA_UUID128_ALL || type == BT_DATA_UUID128_SOME) && data.data_len == 16) {
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

/* Relay thread: send out notifications */
void relay_thread(void) {
    char msg[RELAY_MSG_SIZE];
    while (1) {
        k_msgq_get(&relay_msgq, msg, K_FOREVER);
        bt_gatt_notify(NULL, &relay_svc.attrs[1], msg, strlen(msg));
        printk("[RELAY] Notified: %s\n", msg);
    }
}
K_THREAD_DEFINE(relay_tid, 1024, relay_thread, NULL, NULL, NULL, 5, 0, 0);

/* Read callback: enqueue data, update display, schedule next read */
static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
                       struct bt_gatt_read_params *params,
                       const void *data, uint16_t length) {
    if (!err && data && length) {
        char buf[RELAY_MSG_SIZE];
        size_t ml = MIN(length, RELAY_MSG_SIZE - 1);
        memcpy(buf, data, ml);
        buf[ml] = '\0';
        printk("[CLIENT] Read: %s\n", buf);
        k_msgq_put(&relay_msgq, buf, K_NO_WAIT);
        strncpy(display_buf, buf, RELAY_MSG_SIZE);
    }
    /* Queue next read */
    k_work_schedule(&read_work, K_MSEC(READ_INTERVAL_MS));
    return BT_GATT_ITER_STOP;
}

/* Issue a GATT read */
static void do_read(struct k_work *work) {
    memset(&read_params_global, 0, sizeof(read_params_global));
    read_params_global.func = read_cb;
    read_params_global.handle_count = 1;
    read_params_global.single.handle = char_handle;
    read_params_global.single.offset = 0;
    int err = bt_gatt_read(default_conn, &read_params_global);
    printk("[CLIENT] bt_gatt_read err=%d\n", err);
}

/* Discover callback: grab characteristic handle, start reads */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params) {
    if (!attr) {
        printk("[CLIENT] Discovery complete\n");
        return BT_GATT_ITER_STOP;
    }
    struct bt_gatt_chrc *chrc = (void *)attr->user_data;
    if (!bt_uuid_cmp(params->uuid, &disco_char_uuid.uuid)) {
        char_handle = chrc->value_handle;
        printk("[CLIENT] Found char handle %u\n", char_handle);
        k_work_init_delayable(&read_work, do_read);
        k_work_schedule(&read_work, K_NO_WAIT);
        return BT_GATT_ITER_STOP;
    }
    return BT_GATT_ITER_CONTINUE;
}

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err) {
    printk("[CLIENT] connected(err=%u)\n", err);
    if (err) return;
    default_conn = bt_conn_ref(conn);
    static struct bt_gatt_discover_params discov = {0};
    discov.uuid = &disco_char_uuid.uuid;
    discov.func = discover_func;
    discov.start_handle = 0x0001;
    discov.end_handle = 0xffff;
    discov.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    bt_gatt_discover(default_conn, &discov);
}
static void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("[CLIENT] disconnected(r=%u)\n", reason);
    if (default_conn) { bt_conn_unref(default_conn); default_conn = NULL; }
    k_work_cancel_delayable(&read_work);
    start_scan();
}
static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

/* Scan callback: find service, connect */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *ad) {
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND) return;
    if (!adv_has_uuid(ad, &disco_svc_uuid.uuid)) return;
    printk("[CLIENT] Found Disco, stopping scan\n");
    bt_le_scan_stop();
    bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
                      BT_LE_CONN_PARAM_DEFAULT,
                      &default_conn);
}

/* Start scanning */
static void start_scan(void) {
    bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    printk("[CLIENT] Scanning...\n");
}

/* Bluetooth ready */
static void bt_ready(int err) {
    printk("[CLIENT] bt_ready(err=%d)\n", err);
    if (err) return;
    bt_conn_cb_register(&conn_callbacks);
    start_scan();
}

int main(void) {
    printk("Starting BLE client + relay\n");
    bt_enable(bt_ready);

    lv_init();
    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    display_blanking_off(disp);
    data_label = lv_label_create(lv_scr_act());
    lv_label_set_text(data_label, display_buf);
    lv_obj_align(data_label, LV_ALIGN_CENTER, 0, 0);

    while (1) {
        lv_label_set_text(data_label, display_buf);
        lv_timer_handler();
        k_sleep(K_MSEC(100));
    }
    return 0;
}