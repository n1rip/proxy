#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"
#include "definitions.h"

static n1_bt_device_struct_t mouse = {
    .name = {0},
    .address = {0xF8, 0x1C, 0x9D, 0x70, 0xE8, 0x3D}, // TODO easier way to specify device address
    .connected = 0
};

/**
 * @brief compares two bluetooth mac addresses
 * @param src source address to compare with
 * @param dst destination address to compare with
 * @return 1 if the addresses are different, 0 otherwise
*/
int compare_bd_addr(esp_bd_addr_t src, esp_bd_addr_t dst) {
    for (size_t i = 0; i < ESP_BD_ADDR_LEN; i++) {
        if (src[i] != dst[i]) {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief handles the recieved bluetooth events
 * @param handler_args event handler args
 * @param base unique pointer to a subsystem that exposes events
 * @param id hidh callback event id
 * @param event_data hidh event data component corresponding to the event id
*/
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
            if (param->open.status == ESP_OK) {
                const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
                ESP_LOGI(TAG, ESP_BD_ADDR_STR " open: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
            } else {
                ESP_LOGE(TAG, " open failed!");
            }
            break;
        }
        case ESP_HIDH_BATTERY_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " battery: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
            break;
        }
        case ESP_HIDH_INPUT_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " input: %8s, map: %2u, id: %3u, len: %d, data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
            ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
            break;
        }
        case ESP_HIDH_FEATURE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " feature: %8s, map: %2u, id: %3u, len: %d", ESP_BD_ADDR_HEX(bda),
                    esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                    param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
        }
        case ESP_HIDH_CLOSE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " close: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
            break;
        }
        default: {
            ESP_LOGI(TAG, "event: %d", event);
            break;
        }
    }   
}

/**
 * @brief scans for a specified device mac address
 * @param pvParameters a value that is passed as the paramater to the created task. 
*/
void connect_device(void* pvParameters) {
    n1_bt_device_struct_t* device = (n1_bt_device_struct_t*)pvParameters;
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    for (size_t i = 0; i < SCAN_MAX_TRIES; i++) {
        ESP_LOGI(TAG, "scanning for devices...");
        esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);

        ESP_LOGI(TAG, "scan: %u available device(s)", results_len);
        if (results_len) {
            esp_hid_scan_result_t *r = results;
            esp_hid_scan_result_t *target = NULL;

            while (r) {
                printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                printf("rssi: %d, ", r->rssi);
                printf("usage: %s, ", esp_hid_usage_str(r->usage));

#if CONFIG_BT_BLE_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    if (compare_bd_addr(device->address, r->bda)) {
                        target = r;
                    }
                    printf("appearance: 0x%04x, ", r->ble.appearance);
                    printf("addr_type: '%s', ", ble_addr_type_str(r->ble.addr_type));
                }
#endif

#if CONFIG_BT_HID_HOST_ENABLED
                if (r->transport == ESP_HID_TRANSPORT_BT) {
                    if (compare_bd_addr(device->address, r->bda)) {
                        target = r;
                    }
                    printf("cod: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                    esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                    printf("] srv 0x%03x, ", r->bt.cod.service);
                    print_uuid(&r->bt.uuid);
                    printf(", ");
                }
#endif

                printf("name: %s ", r->name ? r->name : "");
                printf("\n");
                r = r->next;
            }

            if (target) {
                ESP_LOGI(TAG, "target device found. connecting...");
                esp_hidh_dev_open(target->bda, target->transport, target->ble.addr_type);
                device->connected = 1;
                strpcy(device->name, target->name);
            }

            esp_hid_scan_results_free(results);
        }

        if (i >= SCAN_MAX_TRIES || device->connected) {
            break;
        }

        ESP_LOGI(TAG, "target device not found. retrying... (%d/%d)", i + 1, SCAN_MAX_TRIES);
    }

    if (!device->connected) {
        ESP_LOGE(TAG, "cannot find the specified device after %d tries.", SCAN_MAX_TRIES);
    }

    vTaskDelete(NULL);
}

/**
 * microcontroller firmware acting as a proxy between
 * a real bluetooth human interface device (such as a keyboard,
 * a mouse or a controller) and a computer. it processes their
 * input events according to the recieved fpga data.
*/
void app_main(void) {
    esp_err_t ret;
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };

#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));

#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
#endif

    ESP_ERROR_CHECK(esp_hidh_init(&config));

    xTaskCreate(&connect_device, "connect_device", 6 * 1024, &mouse, 2, NULL);
    ESP_ERROR_CHECK(mouse.connected);
}
