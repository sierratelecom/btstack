
#define BTSTACK_FILE__ "hci_transport_bluez.c"

#include <sys/socket.h>

#include "btstack_config.h"

#include "btstack_debug.h"
#include "hci.h"
#include "hci_transport.h"
#include "hci_transport_bluez_posix.h"

/*
*  hci_transport_bluez.c
*
*  HCI Transport API implementation for USB
*
*  Created by Dmitry Savinkin on 30/5/25.
*/

// configure and return bluez singleton
const hci_transport_t * hci_transport_bluez_instance(void)
{
    static const hci_transport_t hci_transport_bluez = {

            /* const char * name; */                                        "BlueZ",
            /* void   (*init) (const void *transport_config); */            &hci_transport_bluez_init,
            /* int    (*open)(void); */                                     &hci_transport_bluez_open,
            /* int    (*close)(void); */                                    &hci_transport_bluez_close,
            /* void   (*register_packet_handler)(void (*handler)(...); */   &hci_transport_bluez_register_packet_handler,
            /* int    (*can_send_packet_now)(uint8_t packet_type); */       &hci_transport_bluez_can_send_now,
            /* int    (*send_packet)(...); */                               &hci_transport_bluez_send_packet,
            /* int    (*set_baudrate)(uint32_t baudrate); */                NULL,
            /* void   (*reset_link)(void); */                               NULL,
            /* void   (*set_sco_config)(uint16_t voice_setting, int num_connections); */ NULL,
    };

    return &hci_transport_bluez;
}
