
#ifndef HCI_TRANSPORT_BLUEZ_POSIX_H
#define HCI_TRANSPORT_BLUEZ_POSIX_H

void hci_transport_bluez_init(const void * transport_config);
int hci_transport_bluez_open(void);
int hci_transport_bluez_close(void);
void hci_transport_bluez_register_packet_handler(void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size));
int hci_transport_bluez_can_send_now(uint8_t packet_type);
int hci_transport_bluez_send_packet(uint8_t packet_type, uint8_t *packet, int size);

#endif //HCI_TRANSPORT_BLUEZ_POSIX_H
