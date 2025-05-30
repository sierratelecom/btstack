
#define BTSTACK_FILE__ "hci_transport_bluez.c"

#include "btstack_config.h"

#include "btstack_debug.h"
#include "hci.h"
#include "hci_transport.h"

/*
*  hci_transport_bluez.c
*
*  HCI Transport API implementation for USB
*
*  Created by Dmitry Savinkin on 30/5/25.
*/

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size);


static void (*packet_handler)(uint8_t packet_type, uint8_t *packet, uint16_t size) = dummy_handler;
//const char *btstack_iface = "";

static void dummy_handler(uint8_t packet_type, uint8_t *packet, uint16_t size)
{
    UNUSED(packet_type);
    UNUSED(packet);
    UNUSED(size);
}

void hci_transport_bluez_init(const void * transport_config)
{

}

int hci_transport_bluez_open(void)
{
    return 0;
}

int hci_transport_bluez_close(void)
{
    return 0;
}

static void hci_transport_bluez_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size))
{
    packet_handler = handler;
}

int hci_transport_bluez_can_send_now(uint8_t packet_type)
{
    UNUSED(packet_type);
    return 0;
}

int hci_transport_bluez_send_packet(uint8_t packet_type, uint8_t *packet, int size)
{
    UNUSED(packet_type);
    UNUSED(packet);
    UNUSED(size);

    return 0;
}

int hci_transport_bluez_set_baudrate(uint32_t baudrate)
{
    UNUSED(baudrate);
    return 0;
}


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
            /* int    (*set_baudrate)(uint32_t baudrate); */                &hci_transport_bluez_set_baudrate,
            /* void   (*reset_link)(void); */                               NULL,
            /* void   (*set_sco_config)(uint16_t voice_setting, int num_connections); */ NULL,
    };

    //btstack_iface = iface_name;
    return &hci_transport_bluez;
}
