#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include "hci_transport_bluez_posix.h"

/**
 * @brief Enumerate available Bluetooth HCI interfaces
 */
int hci_transport_bluez_posix_list_ifaces()
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
    dl = malloc(max_devs * sizeof(struct hci_dev_req) + sizeof(uint16_t));
    if (!dl) {
        perror("malloc");
        close(sock);
        return 1;
    }

    memset(dl, 0, max_devs * sizeof(struct hci_dev_req) + sizeof(uint16_t));
    dl->dev_num = max_devs - 1;

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
