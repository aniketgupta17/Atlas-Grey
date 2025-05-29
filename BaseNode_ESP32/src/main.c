#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/net/buf.h>
#include <string.h>
#include <zephyr/sys/reboot.h>

static struct bt_uuid_128 RELAY_SVC_UUID  = BT_UUID_INIT_128(
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33,
    0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33);
static struct bt_uuid_128 RELAY_CHAR_UUID = BT_UUID_INIT_128(
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44,
    0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x44);

#define MAX_PAYLOAD_LEN 64            
static struct bt_conn               *default_conn;
static uint16_t                      relay_ccc_handle;
static struct bt_gatt_subscribe_params sub_params;

/* Data reception statistics */
static uint32_t packet_count = 0;
static uint32_t total_bytes_received = 0;



static void print_hex_data(const uint8_t *data, uint16_t length)
{
    printk("[ESP-DEVKIT] Raw hex data (%u bytes): ", length);
    for (int i = 0; i < length; i++) {
        printk("%02X ", data[i]);
        if ((i + 1) % 16 == 0) {
            printk("\n[ESP-DEVKIT]                           ");
        }
    }
    printk("\n");
}

static void print_ascii_data(const uint8_t *data, uint16_t length)
{
    printk("[ESP-DEVKIT-Result] ASCII interpretation: '");
    for (int i = 0; i < length; i++) {
        if (data[i] >= 32 && data[i] <= 126) {
            printk("%c", data[i]);
        } else {
            printk(".");
        }
    }
    printk("'\n");
}

static void analyze_data_content(const uint8_t *data, uint16_t length)
{
    printk("[ESP-DEVKIT] Data analysis:\n");
    printk("[ESP-DEVKIT]   - Length: %u bytes\n", length);
    printk("[ESP-DEVKIT]   - First byte: 0x%02X (%u)\n", 
           length > 0 ? data[0] : 0, length > 0 ? data[0] : 0);
    
    if (length > 1) {
        printk("[ESP-DEVKIT]   - Last byte: 0x%02X (%u)\n", 
               data[length-1], data[length-1]);
    }
    
  
    bool is_string = true;
    for (int i = 0; i < length; i++) {
        if (data[i] == 0) {
            
            break;
        }
        if (data[i] < 32 || data[i] > 126) {
            is_string = false;
            break;
        }
    }
    
    printk("[ESP-DEVKIT]   - Appears to be text: %s\n", is_string ? "YES" : "NO");
    
    // Check for common patterns
    if (length >= 4) {
        uint32_t first_uint32 = (data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0];
        printk("[ESP-DEVKIT]   - First 4 bytes as uint32 (LE): %u (0x%08X)\n", 
               first_uint32, first_uint32);
    }
}

static bool adv_has_uuid(struct net_buf_simple *ad,
                         const struct bt_uuid   *uuid)
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


static uint8_t notify_cb(struct bt_conn               *conn,
                         struct bt_gatt_subscribe_params *params,
                         const void                    *data,
                         uint16_t                       length)
{
    if (!data) {
        printk("[ESP-DEVKIT] NOTIFICATIONS STOPPED\n");
        printk("[ESP-DEVKIT] Final statistics:\n");
        printk("[ESP-DEVKIT]   - Total packets received: %u\n", packet_count);
        printk("[ESP-DEVKIT]   - Total bytes received: %u\n", total_bytes_received);
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }

    packet_count++;
    total_bytes_received += length;

    printk("[ESP-DEVKIT] │ PACKET #%u RECEIVED (Length: %u bytes)     │\n", 
           packet_count, length);
    printk("[ESP-DEVKIT] │ Total bytes so far: %u                   │\n", 
           total_bytes_received);

    // Print raw hex data
    print_hex_data((const uint8_t*)data, length);
    
    // Print ASCII interpretation
    print_ascii_data((const uint8_t*)data, length);
    
    // Analyze data content
    analyze_data_content((const uint8_t*)data, length);
    
    // Print as null-terminated string (original behavior)
    char buf[MAX_PAYLOAD_LEN];
    size_t l = MIN(length, sizeof(buf) - 1);
    memcpy(buf, data, l);
    buf[l] = '\0';
    printk("[ESP-DEVKIT] As string: \"%s\"\n", buf);
    
    // Print timestamp
    int64_t uptime_ms = k_uptime_get();
    printk("[ESP-DEVKIT] Timestamp: %lld ms since boot\n", uptime_ms);
    
    printk("[ESP-DEVKIT] ─────────────────────────────────────────\n");

    return BT_GATT_ITER_CONTINUE;
}



static uint8_t discover_func(struct bt_conn *conn,
                             const struct bt_gatt_attr *attr,
                             struct bt_gatt_discover_params *params)
{
    if (!attr) {
        printk("[ESP-DEVKIT] Discovery complete\n");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = (void *)attr->user_data;

    if (!bt_uuid_cmp(params->uuid, &RELAY_CHAR_UUID.uuid)) {
        uint16_t value_handle = chrc->value_handle;
        printk("[ESP-DEVKIT] Found relay char handle = %u\n", value_handle);
        printk("[ESP-DEVKIT] Characteristic properties: 0x%02X\n", chrc->properties);
        
        // Check if notifications are supported
        if (chrc->properties & BT_GATT_CHRC_NOTIFY) {
            printk("[ESP-DEVKIT] Notifications supported\n");
        } else {
            printk("[ESP-DEVKIT] Notifications NOT supported!\n");
        }
        
        if (chrc->properties & BT_GATT_CHRC_INDICATE) {
            printk("[ESP-DEVKIT] Indications supported\n");
        }

        /* Finding CCCD by searching nearby handles */
        relay_ccc_handle = value_handle + 1;
        printk("[ESP-DEVKIT] Attempting CCCD at handle %u\n", relay_ccc_handle);

        sub_params.ccc_handle   = relay_ccc_handle;
        sub_params.value_handle = value_handle;
        sub_params.value        = BT_GATT_CCC_NOTIFY;
        sub_params.notify       = notify_cb;

        int err = bt_gatt_subscribe(conn, &sub_params);
        printk("[ESP-DEVKIT] bt_gatt_subscribe() = %d\n", err);
        
        if (err == 0) {
            printk("[ESP-DEVKIT] SUBSCRIPTION SUCCESSFUL!\n");
            printk("[ESP-DEVKIT] Now listening for notifications...\n");
            printk("[ESP-DEVKIT] Waiting for M5Core2 to send data...\n");
            printk("[ESP-DEVKIT] (Try triggering sensors on M5Core2)\n");
        } else {
            printk("[ESP-DEVKIT] SUBSCRIPTION FAILED! Error: %d\n", err);
            printk("[ESP-DEVKIT] Trying CCCD at handle %u...\n", value_handle + 2);
            
            // Try alternative CCCD location
            sub_params.ccc_handle = value_handle + 2;
            err = bt_gatt_subscribe(conn, &sub_params);
            printk("[ESP-DEVKIT] Second attempt result: %d\n", err);
        }

        return BT_GATT_ITER_STOP;
    }

    return BT_GATT_ITER_CONTINUE;
}

static void start_discovery(struct bt_conn *conn)
{
    static struct bt_gatt_discover_params disc_params = {0};

    printk("[ESP-DEVKIT] Starting GATT service discovery...\n");
    
    disc_params.uuid       = &RELAY_CHAR_UUID.uuid;
    disc_params.func       = discover_func;
    disc_params.start_handle = 0x0001;
    disc_params.end_handle   = 0xffff;
    disc_params.type       = BT_GATT_DISCOVER_CHARACTERISTIC;

    int err = bt_gatt_discover(conn, &disc_params);
    printk("[ESP-DEVKIT] bt_gatt_discover() = %d\n", err);
}


static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("[ESP-DEVKIT] Connection failed (err %u)\n", err);
        return;
    }
    
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));
    
    printk("[ESP-DEVKIT] Connected to %s\n", addr_str);
    printk("[ESP-DEVKIT] Connection parameters:\n");
    
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0) {
        printk("[ESP-DEVKIT]   - Interval: %u units (%.2f ms)\n", 
               info.le.interval, info.le.interval * 1.25);
        printk("[ESP-DEVKIT]   - Latency: %u\n", info.le.latency);
        printk("[ESP-DEVKIT]   - Timeout: %u units (%u ms)\n", 
               info.le.timeout, info.le.timeout * 100);
    }

    default_conn = bt_conn_ref(conn);
    

    packet_count = 0;
    total_bytes_received = 0;
    
    start_discovery(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("[ESP-DEVKIT] Disconnected (reason %u)\n", reason);
    printk("[ESP-DEVKIT] Session statistics:\n");
    printk("[ESP-DEVKIT]   - Packets received: %u\n", packet_count);
    printk("[ESP-DEVKIT]   - Total bytes: %u\n", total_bytes_received);
    
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    
    printk("[ESP-DEVKIT] Restarting scan in 2 seconds...\n");
    k_sleep(K_SECONDS(4));
}

static struct bt_conn_cb conn_cb = {
    .connected    = connected,
    .disconnected = disconnected,
};


static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t adv_type, struct net_buf_simple *ad)
{
    if (adv_type != BT_GAP_ADV_TYPE_ADV_IND &&
        adv_type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        return;
    }
    if (!adv_has_uuid(ad, &RELAY_SVC_UUID.uuid)) {
        return;
    }

    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    printk("[ESP-DEVKIT] M5Core2 Relay found!\n");
    printk("[ESP-DEVKIT]   - Address: %s\n", addr_str);
    printk("[ESP-DEVKIT]   - RSSI: %d dBm\n", rssi);
    printk("[ESP-DEVKIT]   - Advertisement type: %u\n", adv_type);
    printk("[ESP-DEVKIT] Stopping scan and connecting...\n");
    
    bt_le_scan_stop();

    int err = bt_conn_le_create(addr,
                                BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT,
                                &default_conn);
    printk("[ESP-DEVKIT] bt_conn_le_create() = %d\n", err);
}


static void bt_ready(int err)
{
    if (err) {
        printk("[ESP-DEVKIT] Bluetooth init failed (%d)\n", err);
        return;
    }
    
    printk("[ESP-DEVKIT] Bluetooth ready\n");
    printk("[ESP-DEVKIT] Looking for M5Core2 relay devices...\n");
    printk("[ESP-DEVKIT] Target Service UUID: 33333333-3333-3333-3333-333333333333\n");
    printk("[ESP-DEVKIT] Target Char UUID:    44444444-4444-4444-4444-444444444444\n");

    bt_conn_cb_register(&conn_cb);

    int scan_err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    printk("[ESP-DEVKIT] bt_le_scan_start() = %d\n", scan_err);
    
    if (scan_err == 0) {
        printk("[ESP-DEVKIT] 🔍 Scanning for devices...\n");
    }
}


int main(void)
{
    printk("\n");
    printk("    ESP32 DevKit – BLE Data Sink for M5Core2 Relay\n");


    int err = bt_enable(bt_ready);
    printk("[ESP-DEVKIT] bt_enable() returned %d\n", err);

    while (1) {
        k_sleep(K_SECONDS(1));
    }
    return 0;
}
