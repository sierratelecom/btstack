/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "battery_service_v1_server.c"

/**
 * Implementation of the GATT Battery Service Server 
 * To use with your application, add '#import <battery_service.gatt' to your .gatt file
 */

#include "btstack_defines.h"
#include "ble/att_db.h"
#include "ble/att_server.h"
#include "btstack_util.h"
#include "bluetooth_gatt.h"
#include "btstack_debug.h"

#include <stdio.h>

#include "ble/gatt-service/battery_service_v1_server.h"

static battery_service_data_t * bs_battery_services;
static uint16_t bs_battery_services_num = 0;

static uint16_t bs_characteristic_uuids[] = {
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_LEVEL_STATUS,
    ORG_BLUETOOTH_CHARACTERISTIC_ESTIMATED_SERVICE_DATE,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_CRITCAL_STATUS,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_ENERGY_STATUS,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_TIME_STATUS,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_HEALTH_STATUS,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_HEALTH_INFORMATION,
    ORG_BLUETOOTH_CHARACTERISTIC_BATTERY_INFORMATION,
    ORG_BLUETOOTH_CHARACTERISTIC_MANUFACTURER_NAME_STRING,
    ORG_BLUETOOTH_CHARACTERISTIC_MODEL_NUMBER_STRING,
    ORG_BLUETOOTH_CHARACTERISTIC_SERIAL_NUMBER_STRING
};

static char * bs_characteristic_uuid_name[] = {
    "BATTERY_LEVEL              ",
    "BATTERY_LEVEL_STATUS       ",
    "ESTIMATED_SERVICE_DATE     ",
    "BATTERY_CRITCAL_STATUS     ",
    "BATTERY_ENERGY_STATUS      ",
    "BATTERY_TIME_STATUS        ",
    "BATTERY_HEALTH_STATUS      ",
    "BATTERY_HEALTH_INFORMATION ",
    "BATTERY_INFORMATION        ",
    "MANUFACTURER_NAME_STRING   ",
    "MODEL_NUMBER_STRING        ",
    "SERIAL_NUMBER_STRING       "
};

static uint16_t battery_service_read_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t offset, uint8_t * buffer, uint16_t buffer_size){
	UNUSED(con_handle);

	// if (attribute_handle == battery_value_handle){
	// 	return att_read_callback_handle_byte(battery_value, offset, buffer, buffer_size);
	// }
	// if (attribute_handle == battery_value_client_configuration_handle){
	// 	return att_read_callback_handle_little_endian_16(battery_value_client_configuration, offset, buffer, buffer_size);
	// }
	return 0;
}

static int battery_service_write_callback(hci_con_handle_t con_handle, uint16_t attribute_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
	UNUSED(transaction_mode);
	UNUSED(offset);
	UNUSED(buffer_size);

	// if (attribute_handle == battery_value_client_configuration_handle){
	// 	battery_value_client_configuration = little_endian_read_16(buffer, 0);
	// 	battery_value_client_configuration_connection = con_handle;
	// }
	return 0;
}

void battery_service_v1_server_init(uint8_t battery_services_num, battery_service_data_t * battery_services){
    btstack_assert(battery_services_num != 0);
    btstack_assert(battery_services != 0);
	memset(battery_services, 0, battery_services_num * sizeof(battery_service_data_t));
    // get service handle range
	uint16_t start_handle = 0;
	uint16_t end_handle   = 0xffff;
	
    bs_battery_services = battery_services;
    bs_battery_services_num = 0;
    
    // search battery services
    while ( (start_handle < end_handle) && (bs_battery_services_num < battery_services_num)) {
        int service_found = gatt_server_get_handle_range_for_service_with_uuid16(ORG_BLUETOOTH_SERVICE_BATTERY_SERVICE, &start_handle, &end_handle);
        btstack_assert(service_found != 0);

#ifdef ENABLE_TESTING_SUPPORT
        printf("\nBattery Service 0x%02x - 0x%02x \n", start_handle, end_handle);
#endif        
        log_info("Found Battery Service 0x%02x-0x%02x", start_handle, end_handle);

        uint8_t i;
        battery_service_data_t * battery = &battery_services[bs_battery_services_num];
        btstack_assert(battery != NULL);

        uint16_t chr_start_handle = start_handle;
        
        for (i = (uint8_t) BS_CHARACTERISTIC_INDEX_BATTERY_LEVEL; i < (uint8_t) BS_CHARACTERISTIC_INDEX_NUM; i++){
            uint16_t chr_uuid16 = bs_characteristic_uuids[i];
            battery->characteristics[i].value_handle = gatt_server_get_value_handle_for_characteristic_with_uuid16(chr_start_handle, end_handle, chr_uuid16);
            battery->characteristics[i].client_configuration_handle = gatt_server_get_client_configuration_handle_for_characteristic_with_uuid16(chr_start_handle, end_handle, chr_uuid16);
            
            if (battery->characteristics[i].value_handle != 0){
                chr_start_handle = battery->characteristics[i].value_handle + 1;
#ifdef ENABLE_TESTING_SUPPORT
                printf("    %s[%d]                 0x%02x \n", bs_characteristic_uuid_name[i], bs_battery_services_num, battery->characteristics[i].value_handle);
#endif
            }

            if (battery->characteristics[i].client_configuration_handle != 0){
                chr_start_handle = battery->characteristics[i].client_configuration_handle + 1;
#ifdef ENABLE_TESTING_SUPPORT
                printf("    %s[%d] CCD             0x%02x \n", bs_characteristic_uuid_name[i], bs_battery_services_num, battery->characteristics[i].client_configuration_handle);
#endif
            }
        }
        // register service with ATT Server
        battery->att_service.start_handle   = start_handle;
        battery->att_service.end_handle     = end_handle;
        battery->att_service.read_callback  = &battery_service_read_callback;
        battery->att_service.write_callback = &battery_service_write_callback;
        att_server_register_service_handler(&battery->att_service);

        bs_battery_services_num++;
        start_handle = chr_start_handle;
        end_handle = 0xffff;
    }
}


