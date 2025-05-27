#include <zephyr/kernel.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h> // For net_if_get_default()
#include <zephyr/net/dhcpv4.h> // For dhcp_lease and net_if_dhcpv4_get_lease()
#include <string.h> // For strlen

#define WIFI_SSID "YOUR_SSID"       // Replace with your Wi-Fi SSID
#define WIFI_PSK  "YOUR_PASSWORD"   // Replace with your Wi-Fi Password

static struct net_mgmt_event_callback wifi_cb;
static struct net_mgmt_event_callback ipv4_cb;

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                             uint32_t mgmt_event,
                             struct net_if *iface)
{
    const struct wifi_status *status = cb->info; // Requires Zephyr 3.0+

    switch (mgmt_event) {
    case NET_EVENT_WIFI_CONNECT_RESULT:
        printk("Wi-Fi: Interface %p_CON_RES EVT\n", iface);
        if (status->status) {
            printk("Wi-Fi: Connection failed (%d)\n", status->status);
        } else {
            printk("Wi-Fi: Connected!\n");
        }
        break;

    case NET_EVENT_WIFI_DISCONNECT_RESULT:
        printk("Wi-Fi: Interface %p_DISCON_RES EVT\n", iface);
        printk("Wi-Fi: Disconnected.\n");
        break;

    default:
        printk("Wi-Fi: Unknown event (0x%x)\n", mgmt_event);
        break;
    }
}

static void ipv4_event_handler(struct net_mgmt_event_callback *cb,
                             uint32_t mgmt_event,
                             struct net_if *iface)
{
    // char hr_addr[NET_IPV4_ADDR_LEN]; // Not used in this simplified version

    switch (mgmt_event) {
    case NET_EVENT_IPV4_ADDR_ADD:
        printk("IPv4: Interface %p ADD_ADDR_EVT - IP Address Added/Assigned.\n", iface);
        // With CONFIG_NET_LOG=y, the assigned IP address should appear in the system logs.
        // Further detailed IP processing can be added back once basic connectivity is confirmed.
        break;

    case NET_EVENT_IPV4_ADDR_DEL:
        printk("IPv4: Interface %p DEL_ADDR_EVT - IP Address Removed.\n", iface);
        break;

    default:
        printk("IPv4: Unknown event (0x%x) for interface %p\n", mgmt_event, iface);
        break;
    }
}

static int wifi_connect_to_ap(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) {
        printk("Error: No default network interface found.\n");
        return -1;
    }

    struct wifi_connect_req_params cnx_params = {
        .ssid = WIFI_SSID,
        .ssid_length = strlen(WIFI_SSID),
        .psk = WIFI_PSK,
        .psk_length = strlen(WIFI_PSK),
        .security = (strlen(WIFI_PSK) == 0) ? WIFI_SECURITY_TYPE_NONE : WIFI_SECURITY_TYPE_PSK,
        .channel = WIFI_CHANNEL_ANY,
    };

    printk("Wi-Fi: Connecting to SSID: %s\n", cnx_params.ssid);

    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params))) {
        printk("Wi-Fi: Connection request failed.\n");
        return -1;
    }
    printk("Wi-Fi: Connection request sent.\n");
    return 0;
}

int main(void)
{
    printk("Starting Wi-Fi Connection Example on %s\n", CONFIG_BOARD);

    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler, 
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_cb);

    net_mgmt_init_event_callback(&ipv4_cb, ipv4_event_handler, NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
    net_mgmt_add_event_callback(&ipv4_cb);

    if (wifi_connect_to_ap() != 0) {
        printk("Wi-Fi: Failed to initiate connection.\n");
    }

    // The rest of the Wi-Fi handling is event-driven via the callbacks.
    // You can add a loop here or other application logic.
    while (1) {
        k_sleep(K_SECONDS(5));
    }
    return 0;
} 