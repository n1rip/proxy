#pragma once

#include "esp_err.h"
#include "esp_bt_defs.h"

#define SCAN_DURATION_SECONDS 5
#define SCAN_MAX_TRIES 5
#define TAG "proxy"

/**
 * @brief struct containing important information about a wanted device
 * @param name name of the device
 * @param address mac address of a bluetooth device
 * @param connected boolean value specifying whether the device is currently connected or not
*/
typedef struct n1_bt_device_struct {
    char name[256];
    esp_bd_addr_t address;
    esp_err_t connected;
} n1_bt_device_struct_t;