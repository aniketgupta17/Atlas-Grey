/* src/main.c
 * BLE central on M5Core2: scan → connect → read → display via LVGL
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net/buf.h>        /* for net_buf_simple */

#include <zephyr/drivers/display.h>
#include <lvgl.h>

#include <string.h>

#define READ_INTERVAL_MS   1000
#define DISPLAY_BUF_SIZE   64

/* Forward decls */
static bool adv_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid);
static void start_scan(void);

/* On-screen label buffer */
static char display_buf[DISPLAY_BUF_SIZE] = "Waiting for data...";
static lv_obj_t *data_label;

/* UUIDs from your Disco board */
static struct bt_uuid_128 disco_svc_uuid  = BT_UUID_INIT_128(
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33);
static struct bt_uuid_128 disco_char_uuid = BT_UUID_INIT_128(
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44);

/* GATT client state */
static struct bt_conn *default_conn;
static uint16_t   disco_char_handle;
static struct k_work_delayable read_work;
static struct bt_gatt_read_params read_params;

/* Simple adv parser to look for a 128-bit UUID */
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

/* Read callback: copy into display_buf and re-schedule */
static uint8_t read_cb(struct bt_conn *conn, uint8_t err,
                       struct bt_gatt_read_params *p,
                       const void *data, uint16_t length)
{
    printk("[CLIENT] read_cb err=%u length=%u\n", err, length);

    if (!err && data && length) {
        size_t ml = MIN(length, DISPLAY_BUF_SIZE - 1);
        memcpy(display_buf, data, ml);
        display_buf[ml] = '\0';
        printk("[CLIENT] Updated display: %s\n", display_buf);
    }

    /* queue next read in 1 s */
    k_work_schedule(&read_work, K_MSEC(READ_INTERVAL_MS));
    return BT_GATT_ITER_STOP;
}

/* Issue a GATT read */
static void do_read(struct k_work *work)
{
    memset(&read_params, 0, sizeof(read_params));
    read_params.func         = read_cb;
    read_params.handle_count = 1;
    read_params.single.handle = disco_char_handle;
    read_params.single.offset = 0;

    int err = bt_gatt_read(default_conn, &read_params);
    printk("[CLIENT] bt_gatt_read → %d\n", err);
}

/* Discovery callback: look for your characteristic UUID */
static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("[CLIENT] discover complete\n");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (void *)attr->user_data;
    if (!bt_uuid_cmp(params->uuid, &disco_char_uuid.uuid)) {
        disco_char_handle = chrc->value_handle;
        printk("[CLIENT] Found char handle=0x%04X\n", disco_char_handle);

        /* start periodic reads immediately */
        k_work_init_delayable(&read_work, do_read);
        k_work_schedule(&read_work, K_NO_WAIT);
        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    printk("[CLIENT] connected(err=%u)\n", err);
    if (err) {
        printk("[CLIENT] conn failed, rescan\n");
        start_scan();
        return;
    }

    default_conn = bt_conn_ref(conn);

    static struct bt_gatt_discover_params discov;
    discov.uuid        = &disco_char_uuid.uuid;
    discov.func        = discover_func;
    discov.start_handle = 1;
    discov.end_handle   = 0xffff;
    discov.type        = BT_GATT_DISCOVER_CHARACTERISTIC;

    int d_err = bt_gatt_discover(default_conn, &discov);
    printk("[CLIENT] bt_gatt_discover → %d\n", d_err);
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

/* Scan callback: look for your service UUID and connect */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *ad)
{
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND) {
        return;
    }
    if (!adv_has_uuid(ad, &disco_svc_uuid.uuid)) {
        return;
    }

    char str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, str, sizeof(str));
    printk("[SCAN] Found target %s (RSSI %d), stopping scan\n", str, rssi);

    bt_le_scan_stop();
    bt_conn_le_create(addr,
                      BT_CONN_LE_CREATE_CONN,
                      BT_LE_CONN_PARAM_DEFAULT,
                      &default_conn);
}

/* Start scanning */
static void start_scan(void)
{
    printk("[SCAN] start\n");
    int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    printk("[SCAN] bt_le_scan_start → %d\n", err);
}

/* BT stack ready */
static void bt_ready(int err)
{
    printk("[MAIN] bt_ready(err=%d)\n", err);
    if (err) {
        printk("[ERROR] Bluetooth init failed\n");
        return;
    }

    bt_conn_cb_register(&conn_callbacks);
    start_scan();
}

void main(void)
{
    printk("Starting M5Core2 BLE central\n");
    int err = bt_enable(bt_ready);
    printk("[MAIN] bt_enable → %d\n", err);

    /* LVGL display init */
    lv_init();
    const struct device *disp = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    display_blanking_off(disp);
    data_label = lv_label_create(lv_scr_act());
    lv_label_set_text(data_label, display_buf);
    lv_obj_align(data_label, LV_ALIGN_CENTER, 0, 0);

    /* Update loop */
    while (1) {
        lv_label_set_text(data_label, display_buf);
        lv_timer_handler();
        k_sleep(K_MSEC(100));
    }
}
