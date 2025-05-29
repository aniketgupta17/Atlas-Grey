#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net_buf.h>
#include <zephyr/drivers/display.h>
#include <lvgl.h>
#include <string.h>
#include <zephyr/sys/reboot.h>
#include <stdlib.h>
#include <ctype.h>
#define MAX_RETRIES 8
#define READ_INTERVAL_MS        2000
#define RETRY_DELAY_MS            50
#define RECONNECT_DELAY_MS      3000
#define RELAY_MSG_SIZE            64
#define RELAY_QUEUE_SIZE          10
volatile bool sink_subscribed = false;
static struct bt_conn *disco_conn = NULL;
static struct bt_conn *sink_conn;
static uint16_t        disco_char_handle;

static K_SEM_DEFINE(tx_sem, 1, 1);
K_MSGQ_DEFINE(relay_msgq, RELAY_MSG_SIZE, RELAY_QUEUE_SIZE, 4);

static struct k_work_delayable read_work, reconnect_work;

static struct bt_uuid_128 UUID_SVC_DISCO  = BT_UUID_INIT_128(
        0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,
        0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11);
static struct bt_uuid_128 UUID_CHAR_DISCO = BT_UUID_INIT_128(
        0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,
        0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22);
static struct bt_uuid_128 UUID_SVC_RELAY  = BT_UUID_INIT_128(
        0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
        0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33);
static struct bt_uuid_128 UUID_CHAR_RELAY = BT_UUID_INIT_128(
        0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,
        0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44);

static bool disco_connected = false;

typedef struct {
    float x[2];
    float P[2][2];
    float Q[2][2];
    float R[2][2];
} Kalman2D;

static void kalman2d_init(Kalman2D *kf, float x0_dist, float x0_vib) {
    kf->x[0] = x0_dist;
    kf->x[1] = x0_vib;
    kf->P[0][0] = 1.0f; kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f; kf->P[1][1] = 1.0f;
    kf->Q[0][0] = 0.01f; kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;  kf->Q[1][1] = 0.01f;
    kf->R[0][0] = 0.1f;  kf->R[0][1] = 0.0f;
    kf->R[1][0] = 0.0f;  kf->R[1][1] = 0.1f;
}
static void kalman2d_update_measurement_noise(Kalman2D *kf, float dist_rms) {
    if (dist_rms < 1000.0f) kf->R[0][0] = 0.05f;
    else kf->R[0][0] = 0.2f;
    kf->R[1][1] = 0.1f;
}
static void kalman2d_predict(Kalman2D *kf) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            kf->P[i][j] += kf->Q[i][j];
}
static int invert_2x2(float m[2][2], float inv[2][2]) {
    float det = m[0][0]*m[1][1] - m[0][1]*m[1][0];
    if (det == 0) return -1;
    float inv_det = 1.0f / det;
    inv[0][0] =  m[1][1] * inv_det;
    inv[0][1] = -m[0][1] * inv_det;
    inv[1][0] = -m[1][0] * inv_det;
    inv[1][1] =  m[0][0] * inv_det;
    return 0;
}
static void mat_mult_2x2(float a[2][2], float b[2][2], float out[2][2]) {
    for (int i=0; i<2; i++) for (int j=0; j<2; j++) {
        out[i][j] = 0.0f;
        for (int k=0; k<2; k++)
            out[i][j] += a[i][k]*b[k][j];
    }
}
static void mat_mult_2x2_2x1(float a[2][2], float b[2], float out[2]) {
    for (int i=0; i<2; i++) {
        out[i] = 0.0f;
        for (int j=0; j<2; j++) out[i] += a[i][j]*b[j];
    }
}
static void mat_sub_2x2(float a[2][2], float b[2][2], float out[2][2]) {
    for(int i=0; i<2; i++)
        for(int j=0; j<2; j++)
            out[i][j] = a[i][j] - b[i][j];
}
static void kalman2d_update(Kalman2D *kf, float z[2]) {
    float y[2] = { z[0] - kf->x[0], z[1] - kf->x[1] };
    float S[2][2];
    for(int i=0;i<2;i++) for(int j=0;j<2;j++)
        S[i][j] = kf->P[i][j] + kf->R[i][j];
    float invS[2][2];
    if (invert_2x2(S, invS) != 0) return;
    float K[2][2]; mat_mult_2x2(kf->P, invS, K);
    float Ky[2]; mat_mult_2x2_2x1(K, y, Ky);
    for(int i=0; i<2; i++) kf->x[i] += Ky[i];
    float I[2][2] = { {1,0}, {0,1} };
    float I_minus_K[2][2]; mat_sub_2x2(I, K, I_minus_K);
    float newP[2][2]; mat_mult_2x2(I_minus_K, kf->P, newP);
    for(int i=0; i<2; i++) for(int j=0; j<2; j++)
        kf->P[i][j] = newP[i][j];
}

static int pow10(int n) {
    int r = 1;
    for (int i = 0; i < n; i++) r *= 10;
    return r;
}
void float_to_str(float val, char *buf, int decimals) {
    int whole = (int)val;
    float frac = val - whole;
    if (frac < 0) frac = -frac;
    int frac_part = (int)(frac * pow10(decimals) + 0.5f);
    if (val < 0 && whole == 0) {
        sprintf(buf, "-0.%0*d", decimals, frac_part);
    } else {
        sprintf(buf, "%d.%0*d", whole, decimals, frac_part);
    }
}

static void float_to_str2(float v, char *dst)
{
    int scaled = (int)(v * 100 + (v < 0 ? -0.5f : 0.5f));
    sprintf(dst, "%d.%02d", scaled / 100, abs(scaled % 100));
}

static Kalman2D kf;
static bool kalman_initialized = false;


static void ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    printk("[RELAY] CCC changed: 0x%04x\n", value);
    sink_subscribed = (value == BT_GATT_CCC_NOTIFY);
}

BT_GATT_SERVICE_DEFINE(relay_svc,
    BT_GATT_PRIMARY_SERVICE(&UUID_SVC_RELAY),
    BT_GATT_CHARACTERISTIC(&UUID_CHAR_RELAY.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE, NULL, NULL, NULL),
    BT_GATT_CCC(ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void start_advertising(void)
{
    const struct bt_data ad[] = {
        BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
        BT_DATA_BYTES(BT_DATA_UUID128_ALL,
            0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
            0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33),
        BT_DATA(BT_DATA_NAME_COMPLETE, "M5Core2-Relay", 13)
    };

    static const struct bt_le_adv_param adv_conn = {
        .options      = BT_LE_ADV_OPT_CONNECTABLE,
        .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
        .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
    };

    int err = bt_le_adv_start(&adv_conn, ad, ARRAY_SIZE(ad), NULL, 0);
    printk("[RELAY] Advertising started (err=%d)\n", err);
}

static bool ad_has_uuid(struct net_buf_simple *ad, const struct bt_uuid *uuid)
{
    struct bt_data d; struct net_buf_simple_state s;
    net_buf_simple_save(ad,&s);
    while (ad->len > 1) {
        uint8_t len = net_buf_simple_pull_u8(ad);
        uint8_t type= net_buf_simple_pull_u8(ad);
        if (len < 1 || len-1 > ad->len) break;
        d.type = type; d.data_len = len-1; d.data = ad->data;
        if ((type==BT_DATA_UUID128_ALL||type==BT_DATA_UUID128_SOME)&&d.data_len==16){
            struct bt_uuid_128 u; bt_uuid_create(&u.uuid,d.data,16);
            if (!bt_uuid_cmp(uuid,&u.uuid)){ net_buf_simple_restore(ad,&s); return true; }
        }
        net_buf_simple_pull(ad,len-1);
    }
    net_buf_simple_restore(ad,&s); return false;
}


static void tx_done(struct bt_conn *c, void *u) { 
    printk("[RELAY-TX] Notification complete\n");
    k_sem_give(&tx_sem); 
}

void relay_thread(void)
{
    char msg[RELAY_MSG_SIZE];
    while (1) {
        k_msgq_get(&relay_msgq, msg, K_FOREVER);

        if (!sink_conn || !sink_subscribed) {
            printk("[RELAY-THREAD] No sink or not subscribed. Dropping message: \"%s\"\n", msg);
            continue;
        }
        printk("[RELAY-THREAD] Notifying: \"%s\"\n", msg);

        k_sem_take(&tx_sem, K_FOREVER);

        struct bt_gatt_notify_params p = {
            .attr = &relay_svc.attrs[1],
            .data = msg,
            .len  = strlen(msg),
            .func = tx_done,
        };

        int err = bt_gatt_notify_cb(sink_conn, &p);
        if (err == -ENOMEM) {
            printk("*** [RELAY-TX] ENOMEM - retrying (will try again soon) ***\n");
            k_sem_give(&tx_sem);
            k_sleep(K_MSEC(RETRY_DELAY_MS));
        } else if (err) {
            printk("*** [RELAY-TX] bt_gatt_notify_cb returned: %d ***\n", err);
            k_sem_give(&tx_sem);
        } else {
            printk("[RELAY-TX] Notification sent OK\n");
        }
    }
}
K_THREAD_DEFINE(relay_tid, 1024, relay_thread, NULL, NULL, NULL, 5, 0, 0);


static void schedule_read(struct k_work *w);

static int parse_sensor(const char *in, float *vib, float *dist)
{
    if (!in || !vib || !dist) return 0;
    const char *p = in;

    long v_whole = strtol(p, (char **)&p, 10);
    if (*p != '.') return 0;
    ++p;

    long v_frac = 0, v_scale = 1;
    int v_digits = 0;
    while (isdigit((unsigned char)*p) && v_digits < 3) {
        v_frac  = v_frac * 10 + (*p - '0');
        v_scale = v_scale * 10;
        ++p; ++v_digits;
    }
    if (*p != ',') return 0;
    ++p;

    long d_whole = strtol(p, (char **)&p, 10);
    if (*p != '.') return 0;
    ++p;

    long d_frac = 0, d_scale = 1;
    int d_digits = 0;
    while (isdigit((unsigned char)*p) && d_digits < 3) {
        d_frac  = d_frac * 10 + (*p - '0');
        d_scale = d_scale * 10;
        ++p; ++d_digits;
    }
    if (*p != '@') return 0;

    *vib  = v_whole + (float)v_frac / v_scale;
    *dist = d_whole + (float)d_frac / d_scale;
    return 1;
}

static void fp_printk(const char *tag, float val)
{
    int scaled = (int)(val * 100 + (val < 0 ? -0.5f : 0.5f));
    printk("%s%d.%02d", tag, scaled / 100, abs(scaled % 100));
}

static void log_parsed_values(float vib, float dist)
{
    printk("[RELAY-READ] Parsed: ");
    fp_printk("VIB=", vib);
    fp_printk(" DIST=", dist);
    printk("\n");
}

static uint8_t read_cb(struct bt_conn *c, uint8_t err,
                       struct bt_gatt_read_params *p,
                       const void *data, uint16_t len)
{
    if (!disco_connected) {
        printk("[RELAY-READ] Not connected to Disco, skipping data\n");
        return BT_GATT_ITER_STOP;
    }

    if (!err && data && len) {
        char buf[RELAY_MSG_SIZE];
        size_t n = MIN(len, sizeof(buf)-1);
        memcpy(buf, data, n); buf[n] = 0;

        float vib = 0, dist = 0;
        if (parse_sensor(buf, &vib, &dist)) {
            if (!kalman_initialized) {
                kalman2d_init(&kf, dist, vib);
                kalman_initialized = true;
            }

            kalman2d_update_measurement_noise(&kf, dist);
            kalman2d_predict(&kf);

            float z[2] = { dist, vib };
            kalman2d_update(&kf, z);

            char vib_s[8], dist_s[8];
            float_to_str2(kf.x[1], vib_s);
            float_to_str2(kf.x[0], dist_s);

            char out_msg[20];
            sprintf(out_msg, "%s,%s@%s",
                    vib_s, dist_s,
                    strchr(buf, '@') ? strchr(buf, '@') + 1 : "00:00:00");
            out_msg[sizeof(out_msg)-1] = '\0';
            printk("[RELAY-READ] Filtered string len=%d: \"%s\"\n", (int)strlen(out_msg), out_msg);

            k_msgq_put(&relay_msgq, out_msg, K_NO_WAIT);
        } else {
            printk("[RELAY-READ] Parse fail, relaying raw: \"%s\"\n", buf);
            k_msgq_put(&relay_msgq, buf, K_NO_WAIT);
        }
    } else {
        printk("[RELAY-READ] Read error or empty data (err=%u, len=%u)\n", err, len);
    }

    if (disco_connected)
        k_work_schedule(&read_work, K_MSEC(READ_INTERVAL_MS));
    return BT_GATT_ITER_STOP;
}



static void schedule_read(struct k_work *w)
{
    if (!disco_connected || !disco_conn) {
        printk("[RELAY-READ] No Disco connection, skipping read\n");
        return;
    }
    static struct bt_gatt_read_params params;
    params.func = read_cb;
    params.handle_count = 1;
    params.single.handle = disco_char_handle;
    params.single.offset = 0;
    printk("[RELAY-READ] Scheduling read from handle 0x%04x\n", disco_char_handle);
    bt_gatt_read(disco_conn, &params);
}


static uint8_t discover_cb(struct bt_conn *c, const struct bt_gatt_attr *a,
                           struct bt_gatt_discover_params *p)
{
    if (!a) return BT_GATT_ITER_STOP;
    struct bt_gatt_chrc *ch = (void *)a->user_data;
    disco_char_handle = ch->value_handle;
    printk("[RELAY] Disco characteristic discovered, handle=0x%04x\n", disco_char_handle);
    disco_connected = true;
    k_work_init_delayable(&read_work, schedule_read);
    k_work_schedule(&read_work, K_NO_WAIT);
    return BT_GATT_ITER_STOP;
}

static void disco_connect(struct bt_conn *c)
{
    static struct bt_gatt_discover_params dp = {0};
    dp.uuid = &UUID_CHAR_DISCO.uuid;
    dp.func = discover_cb;
    dp.start_handle = 0x0001;
    dp.end_handle   = 0xffff;
    dp.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    printk("[RELAY] Starting GATT discovery on Disco connection\n");
    bt_gatt_discover(c, &dp);
}


static void reconnect_fn(struct k_work *work);

static void connected(struct bt_conn *c, uint8_t err)
{
    if (err) { 
        printk("[RELAY] Connection failed, err=%u\n", err);
        return; 
    }

    struct bt_conn_info info;
    if (bt_conn_get_info(c, &info)) {
        printk("[ERR] cannot get conn info\n");
        return;
    }

    if (info.role == BT_CONN_ROLE_CENTRAL) {
        printk("[RELAY] Connected to Disco as CENTRAL\n");
        disco_conn = bt_conn_ref(c);
        disco_connected = true;
        disco_connect(c);
    } else {
        printk("[RELAY] Sink connected as PERIPHERAL\n");
        sink_conn = bt_conn_ref(c);
    }
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{
    printk("[RELAY] Disconnected (reason=%u)\n", reason);
    if (c == disco_conn) { 
        if (disco_conn) {
        bt_conn_unref(disco_conn); 
        disco_conn = NULL; 
        disco_connected = false;
        }
        printk("[RELAY] Scheduling reconnect in %d ms\n", RECONNECT_DELAY_MS);
        k_work_schedule(&reconnect_work, K_MSEC(RECONNECT_DELAY_MS));
    }
    if (c == sink_conn ) { 
        bt_conn_unref(sink_conn ); 
        sink_conn  = NULL; 
    }
}

static struct bt_conn_cb conn_cb = {
    .connected = connected,
    .disconnected = disconnected,
};

static void disconnect_if_match(struct bt_conn *conn, void *data)
{
    const bt_addr_le_t *target_addr = (const bt_addr_le_t *)data;
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 &&
        !bt_addr_le_cmp(&info.le.dst, target_addr)) {
        printk("[RELAY] Forcibly disconnecting stale conn\n");
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        bt_conn_unref(conn);
    }
}

static void force_delete_conn(const bt_addr_le_t *target_addr)
{
    bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_if_match, (void *)target_addr);
}


static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type, struct net_buf_simple *ad)
{
    if (type != BT_GAP_ADV_TYPE_ADV_IND) return;
    if (!ad_has_uuid(ad, &UUID_SVC_DISCO.uuid)) return;
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
    printk("[RELAY] Found Disco device at %s (RSSI %d)\n", addr_str, rssi);
    bt_le_scan_stop();

    force_delete_conn(addr);

    k_sleep(K_MSEC(200));

    int tries = 0, err = 0;
    for (tries = 0; tries < MAX_RETRIES; ++tries) {
        printk("[RELAY] Creating connection to Disco, try %d...\n", tries+1);
        err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &disco_conn);
        printk("[RELAY] bt_conn_le_create returned %d\n", err);
        if (err == 0) {
            break;
        }
        k_sleep(K_MSEC(350));
    }
    if (err != 0) {
        printk("[RELAY] Connection failed after %d retries, rebooting device...\n", MAX_RETRIES);
        k_sleep(K_MSEC(100));
        sys_reboot(SYS_REBOOT_COLD);
    }
}

static void start_scan(void)
{
    printk("[RELAY] Starting scan for Disco\n");
    bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
}


static void reconnect_fn(struct k_work *work)
{
    printk("[RELAY] Attempting to reconnect to Disco...\n");
    if (disco_conn) {
        bt_conn_unref(disco_conn);
        disco_conn = NULL;
    }
    k_sleep(K_MSEC(200));
    start_scan();
}


static void bt_ready(int err)
{
    printk("[RELAY] BT ready (err=%d)\n", err);
    bt_conn_cb_register(&conn_cb);
    start_scan();
    start_advertising();
    k_work_init_delayable(&reconnect_work, reconnect_fn);
}

int main(void)
{
    printk("M5Core2 relay - build OK\n");
    bt_enable(bt_ready);
    while (1) { k_sleep(K_SECONDS(1)); }
}
