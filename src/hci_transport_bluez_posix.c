#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include "hci_transport_bluez_posix.h"

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size);
static int hci_transport_bluez_posix_list_ifaces(void);

static void (*packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size) = dummy_handler;
//const char *btstack_iface = "";

static int _bluez_fd = 0;

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
    _bluez_fd = -1;
}

int hci_transport_bluez_open(void)
{
    int sock;

    printf("Called %s()\n", __FUNCTION__);
    // Open a raw HCI socket
//    sock = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, BTPROTO_HCI);
    sock = socket(PF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    int dev_id = 0xffff;
    struct sockaddr_hci a;
    /* Bind socket to the HCI device */
    memset(&a, 0, sizeof(a));
    a.hci_family = AF_BLUETOOTH;
    a.hci_dev = dev_id;
    a.hci_channel = HCI_CHANNEL_CONTROL;
//    a.hci_channel = HCI_CHANNEL_RAW;
    if (bind(sock, (struct sockaddr *) &a, sizeof(a)) < 0)
    {
        printf("bind(): mgmt.fd");
        close(sock);
        return -1;
    }

    _bluez_fd = sock;

    return 0;
}

int hci_transport_bluez_close(void)
{
    printf("Called %s()\n", __FUNCTION__);

    if (_bluez_fd < 0)
    {
        return -1;
    }

    close(_bluez_fd);
    _bluez_fd = -1;

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
//    printf("Called %s(%u)\n", __FUNCTION__, packet_type);
    (void)packet_type;
    return (_bluez_fd > 0);
}

int hci_transport_bluez_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    printf("Called %s(%u %d) ", __FUNCTION__, packet_type, size);
    for (int i = 0; i < size; i++)
    {
        printf("[%02x]", packet[i]);
    }
    printf("\n");

    (void)packet_type;
    (void)packet;
    (void)size;

#if 0
    struct hci_filter nf, of;
    socklen_t olen;

    olen = sizeof(of);
    if (getsockopt(_bluez_fd, SOL_HCI, HCI_FILTER, &of, &olen) < 0)
        return -1;

    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT,  &nf);
    hci_filter_set_event(EVT_CMD_STATUS, &nf);
    hci_filter_set_event(EVT_CMD_COMPLETE, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
//    hci_filter_set_event(r->event, &nf);
//    hci_filter_set_opcode(opcode, &nf);
    if (setsockopt(_bluez_fd, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0)
        return -1;

//    ssize_t written = write(_bluez_fd, packet, size);
//    printf("written = %zd\n", written);
#endif
    while (write(_bluez_fd, packet, size) < 0) {
        printf("errno=%d\n", size);
        if (errno == EAGAIN || errno == EINTR)
            continue;
        goto failed;
    }

    printf("Exit %s()\n", __FUNCTION__);

#if 0
    if (setsockopt(_bluez_fd, SOL_HCI, HCI_FILTER, &of, sizeof(of)) < 0)
        return -1;
#endif
    return 0;

failed:

#if 0
    if (setsockopt(_bluez_fd, SOL_HCI, HCI_FILTER, &of, sizeof(of)) < 0)
        return -1;
#endif

    return 0;

    return -1;
}
