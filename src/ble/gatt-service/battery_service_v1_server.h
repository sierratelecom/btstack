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

/**
 * @title Battery Service Server
 * 
 */

#ifndef BATTERY_SERVICE_V1_SERVER_H
#define BATTERY_SERVICE_V1_SERVER_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/**
 * @text The Battery Service allows to query your device's battery level in a standardized way.
 * 
 * To use with your application, add `#import <battery_service.gatt>` to your .gatt file. 
 * After adding it to your .gatt file, you call *battery_service_server_init(value)* with the
 * current value of your battery. The valid range for the battery level is 0-100.
 *
 * If the battery level changes, you can call *battery_service_server_set_battery_value(value)*. 
 * The service supports sending Notifications if the client enables them.
 */

#ifndef BATTERY_SERVICE_SERVER_MAX_NUM_BATTERIES
#define BATTERY_SERVICE_SERVER_MAX_NUM_BATTERIES 1
#endif

/* API_START */

typedef enum {
    BS_CHARACTERISTIC_INDEX_BATTERY_LEVEL = 0,
    BS_CHARACTERISTIC_INDEX_BATTERY_LEVEL_STATUS,
    BS_CHARACTERISTIC_INDEX_ESTIMATED_SERVICE_DATE,
    BS_CHARACTERISTIC_INDEX_BATTERY_CRITCAL_STATUS,
    BS_CHARACTERISTIC_INDEX_BATTERY_ENERGY_STATUS,                                    
    BS_CHARACTERISTIC_INDEX_BATTERY_TIME_STATUS,                                           
    BS_CHARACTERISTIC_INDEX_BATTERY_HEALTH_STATUS,
    BS_CHARACTERISTIC_INDEX_BATTERY_HEALTH_INFORMATION,
    BS_CHARACTERISTIC_INDEX_BATTERY_INFORMATION,
    BS_CHARACTERISTIC_INDEX_MANUFACTURER_NAME_STRING,
    BS_CHARACTERISTIC_INDEX_MODEL_NUMBER_STRING,
    BS_CHARACTERISTIC_INDEX_SERIAL_NUMBER_STRING,
    BS_CHARACTERISTIC_INDEX_NUM
} battery_service_characteristic_index_t;

typedef struct {
    uint16_t value_handle;
    uint16_t client_configuration_handle;
} battery_service_characteristic_t;

typedef struct {
    att_service_handler_t  att_service;
    battery_service_characteristic_t characteristics[BS_CHARACTERISTIC_INDEX_NUM];
} battery_service_data_t;

typedef struct {
    hci_con_handle_t con_handle;
    uint16_t client_configurations[BS_CHARACTERISTIC_INDEX_NUM];

    btstack_context_callback_registration_t  scheduled_tasks_callback;
    uint16_t scheduled_tasks[BATTERY_SERVICE_SERVER_MAX_NUM_BATTERIES];
} battery_service_server_connection_t;

/**
 * @brief Init Battery Service Server with ATT DB
 * 
 * @param battery_services_num
 * @param battery_services
 * @param clients_num
 * @param clients
 */
void battery_service_v1_server_init(uint8_t battery_services_num, battery_service_data_t * battery_services, 
     uint8_t clients_num, battery_service_server_connection_t * clients);


/* API_END */

#if defined __cplusplus
}
#endif

#endif

