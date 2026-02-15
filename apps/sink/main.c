#include <stdio.h>
#include <string.h>

#include "ztimer.h"
#include "thread.h"

#include "periph/gpio.h"

#include "net/gnrc.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/netapi.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/rpl.h"
#include "net/gnrc/udp.h"

/* ---- Defines ---- */

#define MAGIC                       0x4F

#define TYPE_PLANT_HUB              0xA3

#define UDP_PORT                    0x7421

/* ---- Global variables/structs ---- */

kernel_pid_t receive_thread_pid = KERNEL_PID_UNDEF;
static char _receive_stack[THREAD_STACKSIZE_LARGE * 8];

netif_t *netif = NULL;

/* ---- Parse section ---- */

typedef struct
{
    uint8_t magic; // Magic byte to identify the network
    uint8_t type; // Type to distinguish different message formats
    uint8_t version; // Version of the message format, can be used for future extensions
    uint16_t id; // ID of the sending node (MAC address)
    uint16_t rank; // RPL rank of the sending node
    
    uint16_t sensor_connected_bitmap; // Bitmap indicating which sensors are connected
    uint16_t sensor_calibration_bitmap; // Bitmap indicating which sensors are calibrated
    uint16_t sensor_values[12]; // Latest sensor values for each port
} rf_plant_hub_message_t;

static void _parse_plant_hub_message(rf_plant_hub_message_t *msg)
{
    // Parse the data and print to stdout in json format
    printf("{");
    printf("\"type\": %u, ", msg->type);
    printf("\"version\": %u, ", msg->version);
    printf("\"id\": %u, ", msg->id);
    printf("\"rank\": %u, ", msg->rank);
    printf("\"scon_bitmap\": %u, ", msg->sensor_connected_bitmap);
    printf("\"scal_bitmap\": %u, ", msg->sensor_calibration_bitmap);

    printf("\"sensor_values\": [");
    for (int i = 0; i < 12; i++) {
        printf("%u", msg->sensor_values[i]);
        if (i < 11) {
            printf(", ");
        }
    }
    printf("]");
    printf("}\n");
}

/* ---- Receive section ---- */

static void _check_pkt_type_and_parse(gnrc_pktsnip_t *pkt)
{
    uint8_t *data = (uint8_t *) pkt->data;

    // First byte is the MAGIC number, check if it matches
    if (data[0] != MAGIC)
    {
        return;
    }

    // Parse message depending on its type
    switch (data[1])
    {
    case TYPE_PLANT_HUB:

        rf_plant_hub_message_t msg;
        memcpy(&msg, data, sizeof(rf_plant_hub_message_t));

        _parse_plant_hub_message(&msg);
        break;
    
    default:
        break;
    }
}

static void *receive_thread(void *arg)
{
    (void) arg;

    msg_t ipc_msg;
    msg_t ipc_msg_queue[64];

    /* Setup the message queue */
    msg_init_queue(ipc_msg_queue, 64);

    gnrc_pktsnip_t *pkt;

    while (1)
    {
        gpio_set(LED0_PIN); // Turn off red LED after processing

        msg_receive(&ipc_msg);

        gpio_clear(LED0_PIN); // Turn on red LED to indicate message processing

        if (ipc_msg.type == GNRC_NETAPI_MSG_TYPE_RCV)
        {
            pkt = (gnrc_pktsnip_t *) ipc_msg.content.ptr;
            _check_pkt_type_and_parse(pkt);
            gnrc_pktbuf_release(pkt);
        }
    }
    
    /* never reached */
    return NULL;
}

/* ---- Init section ---- */

static gnrc_netreg_entry_t udp_server = GNRC_NETREG_ENTRY_INIT_PID(GNRC_NETREG_DEMUX_CTX_ALL, KERNEL_PID_UNDEF);

int main(void)
{
    // Initialize network interface
    netif = netif_iter(NULL);

    // Initialize red LED
    gpio_init(LED0_PIN, GPIO_OUT);

    // Parse global adress and initialize RPL root
    ipv6_addr_t ipv6_glob_addr;
    const char *str_adr = "2001:db8::1";
    ipv6_addr_from_str(&ipv6_glob_addr, str_adr);

    uint16_t flags = GNRC_NETIF_IPV6_ADDRS_FLAGS_STATE_VALID;
    uint8_t prefix_len = 64;
    flags |= (prefix_len << 8);
    netif_set_opt(netif, NETOPT_IPV6_ADDR, flags, &ipv6_glob_addr, sizeof(ipv6_glob_addr));

    gnrc_rpl_init(netif_get_id(netif));
    gnrc_rpl_root_init(1, &ipv6_glob_addr, false, false);

    /* Create receive thread */
    receive_thread_pid = thread_create(_receive_stack, sizeof(_receive_stack), THREAD_PRIORITY_MAIN,
                                      THREAD_CREATE_STACKTEST,
                                      receive_thread, NULL, "receive_thread");

    // Initialize UDP server
    udp_server.target.pid = receive_thread_pid;
    udp_server.demux_ctx = UDP_PORT;
    gnrc_netreg_register(GNRC_NETTYPE_UDP, &udp_server);

    thread_sleep();

    return 0;
}
