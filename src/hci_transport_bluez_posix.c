#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include "hci_transport_bluez_posix.h"

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size);
static int hci_transport_bluez_posix_list_ifaces(void);

static void (*packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size) = dummy_handler;
//const char *btstack_iface = "";

/**
 * @brief Enumerate available Bluetooth HCI interfaces
 */
static int hci_transport_bluez_posix_list_ifaces()
{
    int sock;
    struct hci_dev_list_req *dl;
    int i;

    // Open a raw HCI socket
    sock = socket(AF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    // Allocate memory for device list (max 16 adapters)
    int max_devs = HCI_MAX_DEV;
    size_t m_size = max_devs * sizeof(struct hci_dev_req) + sizeof(struct hci_dev_list_req);

    dl = malloc(m_size);
    if (!dl) {
        perror("malloc");
        close(sock);
        return 1;
    }

    memset(dl, 0, m_size);
    dl->dev_num = max_devs;

    // Fetch the device list
    if (ioctl(sock, HCIGETDEVLIST, (void *)dl) < 0) {
        perror("ioctl(HCIGETDEVLIST)");
        free(dl);
        close(sock);
        return 1;
    }

    printf("Available Bluetooth interfaces:\n");
    for (i = 0; i < dl->dev_num; i++) {
        struct hci_dev_info di;
        memset(&di, 0, sizeof(di));
        di.dev_id = dl->dev_req[i].dev_id;

        if (ioctl(sock, HCIGETDEVINFO, (void *)&di) < 0) {
            perror("ioctl(HCIGETDEVINFO)");
            continue;
        }

        char addr[18] = {0};
        //ba2str(&di.bdaddr, addr);

        printf("  hci%d - %s (Name: %s, Flags: 0x%x, Type: %d)\n",
               di.dev_id, addr, di.name, di.flags, di.type);
    }

    free(dl);

    close(sock);
    return 0;
}

void hci_transport_bluez_init(const void * transport_config)
{
    const char* iface = transport_config;
    printf("%s() iface='%s'\n", __FUNCTION__, iface);
#if 0
    int fd = socket(PF_BLUETOOTH, SOCK_RAW, BTPROTO_HCI);
    printf("%s() fd=%d\n", __FUNCTION__, fd);
    if (fd < 0)
    {
        printf("socket(): fd < 0\n");
        return;
    }
#endif

    hci_transport_bluez_posix_list_ifaces();
}

int hci_transport_bluez_open(void)
{
    printf("Called %s()\n", __FUNCTION__);
    return 0;
}

int hci_transport_bluez_close(void)
{
    printf("Called %s()\n", __FUNCTION__);
    return 0;
}

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size)
{
    (void)packet_type;
    (void)packet;
    (void)size;
}

void hci_transport_bluez_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size))
{
    printf("Called %s()\n", __FUNCTION__);
    packet_handler = handler;
}


int hci_transport_bluez_can_send_now(uint8_t packet_type)
{
    printf("Called %s(%u)\n", __FUNCTION__, packet_type);
    (void)packet_type;
    return 1;
}

int hci_transport_bluez_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    printf("Called %s(%u, %d) ", __FUNCTION__, packet_type, size);
    for (int i = 0; i < size; i++)
    {
        printf("[%02x]", packet[i]);
    }
    printf("\n");

    (void)packet_type;
    (void)packet;
    (void)size;

    return 0;
}
