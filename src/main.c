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
#include "esp_gattc_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include "definitions.h"

/**
 * @brief represents the mouse
*/
static bt_device_struct_t mouse = {
    .address = {0xF8, 0x1C, 0x9D, 0x78, 0xE8, 0x3D},
    .connected = 0
};

/**
 * @brief represents the computer
*/
static bt_device_struct_t computer = {
    .connected = 0
};

/**
 * @brief contains information about a HID device and its task
 * @param task_hdl handle to the associated task
 * @param hid_dev pointer to the HID device
 * @param protocol_mode HID protocol mode
 * @param buffer unknown
*/
typedef struct local_param_s {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
    uint8_t protocol_mode;
    uint8_t *buffer;
} local_param_t;

static local_param_t hid_mouse = {0};

const unsigned char mouseReportDescriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x02,        //   Usage (Mouse)
    0xA1, 0x02,        //   Collection (Logical)
    0x85, 0x01,        //     Report ID (1)
    0x09, 0x01,        //     Usage (Pointer)
    0xA1, 0x00,        //     Collection (Physical)
    0x05, 0x09,        //       Usage Page (Button)
    0x19, 0x01,        //       Usage Minimum (0x01)
    0x29, 0x05,        //       Usage Maximum (0x05)
    0x15, 0x00,        //       Logical Minimum (0)
    0x25, 0x01,        //       Logical Maximum (1)
    0x95, 0x05,        //       Report Count (5)
    0x75, 0x01,        //       Report Size (1)
    0x81, 0x02,        //       Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x95, 0x01,        //       Report Count (1)
    0x75, 0x03,        //       Report Size (3)
    0x81, 0x01,        //       Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x01,        //       Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,        //       Usage (X)
    0x09, 0x31,        //       Usage (Y)
    0x95, 0x02,        //       Report Count (2)
    0x75, 0x10,        //       Report Size (16)
    0x16, 0x01, 0x80,  //       Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //       Logical Maximum (32767)
    0x81, 0x06,        //       Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xA1, 0x02,        //       Collection (Logical)
    0x85, 0x01,        //         Report ID (1)
    0x09, 0x38,        //         Usage (Wheel)
    0x35, 0x00,        //         Physical Minimum (0)
    0x45, 0x00,        //         Physical Maximum (0)
    0x95, 0x01,        //         Report Count (1)
    0x75, 0x10,        //         Report Size (16)
    0x16, 0x01, 0x80,  //         Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //         Logical Maximum (32767)
    0x81, 0x06,        //         Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0x85, 0x01,        //         Report ID (1)
    0x05, 0x0C,        //         Usage Page (Consumer)
    0x0A, 0x38, 0x02,  //         Usage (AC Pan)
    0x35, 0x00,        //         Physical Minimum (0)
    0x45, 0x00,        //         Physical Maximum (0)
    0x95, 0x01,        //         Report Count (1)
    0x75, 0x10,        //         Report Size (16)
    0x16, 0x01, 0x80,  //         Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //         Logical Maximum (32767)
    0x81, 0x06,        //         Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //       End Collection
    0xC0,              //     End Collection
    0xC0,              //   End Collection
    0xC0,              // End Collection
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x1A, 0xE0, 0x00,  //   Usage Minimum (0xE0)
    0x2A, 0xE7, 0x00,  //   Usage Maximum (0xE7)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x08,        //   Report Count (8)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x07,        //   Usage Page (Kbrd/Keypad)
    0x19, 0x00,        //   Usage Minimum (0x00)
    0x2A, 0x91, 0x00,  //   Usage Maximum (0x91)
    0x16, 0x00, 0x00,  //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x0A,        //   Report Count (10)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
    0x06, 0x07, 0xFF,  // Usage Page (Vendor Defined 0xFF07)
    0x0A, 0x12, 0x02,  // Usage (0x0212)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0xF0,        //   Report ID (-16)
    0x09, 0x01,        //   Usage (0x01)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x13,        //   Report Count (19)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0xB1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF1,        //   Report ID (-15)
    0x95, 0x13,        //   Report Count (19)
    0x09, 0x02,        //   Usage (0x02)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

/**
 * @brief HID raw report map used by the mouse configuration, see the HID specification: https://www.usb.org/sites/default/files/hid1_11.pdf
*/
static esp_hid_raw_report_map_t mouse_report_descriptors[] = {
    {
        .data = mouseReportDescriptor,
        .len = sizeof(mouseReportDescriptor)
    }
};

/**
 * @brief configuration of the HID mouse device exposed by the chip
*/
static esp_hid_device_config_t hidd_config = {
    .vendor_id = 0x1337,
    .product_id = 0x1337,
    .version = 0x1337,
    .device_name = "n1 mouse",
    .manufacturer_name = "n1",
    .serial_number = "1337",
    .report_maps = mouse_report_descriptors,
    .report_maps_len = 1
};

/**
 * @brief sends a mouse event
 * @param buttons button number
 * @param dx x displacement
 * @param dy y displacement
 * @param wheel mouse wheel roll
*/
void send_mouse(uint8_t buttons, char dx, char dy, char wheel) {
    static uint8_t buffer[4] = {0};

    buffer[0] = buttons;
    buffer[1] = dx;
    buffer[2] = dy;
    buffer[3] = wheel;
    esp_hidd_dev_input_set(hid_mouse.hid_dev, 0, 1, buffer, 4);
}

/**
 * @brief handles the bluetooth events for the device state of the controller
 * @param handler_args event handler args
 * @param base unique pointer to a subsystem that exposes events
 * @param id hidd callback event id
 * @param event_data hidd event data component corresponding to the event id
*/
static void hidd_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *HIDD_TAG = "HIDD";

    switch (event) {
    case ESP_HIDD_START_EVENT: {
        ESP_LOGI(HIDD_TAG, "start");
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT: {
        ESP_LOGI(HIDD_TAG, "connect");
        computer.connected = 1;
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT: {
        ESP_LOGI(HIDD_TAG, "protocol mode[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT: {
        ESP_LOGI(HIDD_TAG, "control[%u]: %ssuspend", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT: {
        ESP_LOGI(HIDD_TAG, "output[%u]: %8s id: %2u, len: %d, data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(HIDD_TAG, param->output.data, param->output.length);
        break;
    }
    case ESP_HIDD_FEATURE_EVENT: {
        ESP_LOGI(HIDD_TAG, "feature[%u]: %8s id: %2u, len: %d, data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(HIDD_TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT: {
        ESP_LOGI(HIDD_TAG, "disconnect: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
        esp_hid_ble_gap_adv_start();
        esp_restart();
        break;
    }
    case ESP_HIDD_STOP_EVENT: {
        ESP_LOGI(HIDD_TAG, "stop");
        break;
    }
    default:
        break;
    }
    return;
}

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
 * @brief handles the recieved bluetooth events for the host state of the controller
 * @param handler_args event handler args
 * @param base unique pointer to a subsystem that exposes events
 * @param id hidh callback event id
 * @param event_data hidh event data component corresponding to the event id
*/
void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;
    static const char *HIDH_TAG = "HIDH";

    switch (event) {
        case ESP_HIDH_OPEN_EVENT: {
            if (param->open.status == ESP_OK) {
                const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);

                ESP_LOGI(HIDH_TAG, ESP_BD_ADDR_STR " open: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
                esp_hidh_dev_dump(param->open.dev, stdout);
                memcpy(mouse.address, bda, 6);
            } else {
                ESP_LOGE(HIDH_TAG, " open failed!");
            }
            break;
        }
        case ESP_HIDH_BATTERY_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);

            ESP_LOGI(HIDH_TAG, ESP_BD_ADDR_STR " battery: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
            break;
        }
        case ESP_HIDH_INPUT_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);

            ESP_LOGI(HIDH_TAG, ESP_BD_ADDR_STR " input: %8s, map: %2u, id: %3u, len: %d, data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
            ESP_LOG_BUFFER_HEX(HIDH_TAG, param->input.data, param->input.length);
            esp_hidd_dev_input_set(hid_mouse.hid_dev, param->input.map_index, 1, param->input.data, param->input.length);
            break;
        }
        case ESP_HIDH_FEATURE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);

            ESP_LOGI(HIDH_TAG, ESP_BD_ADDR_STR " feature: %8s, map: %2u, id: %3u, len: %d", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id, param->feature.length);
            ESP_LOG_BUFFER_HEX(HIDH_TAG, param->feature.data, param->feature.length);
            break;
        }
        case ESP_HIDH_CLOSE_EVENT: {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);

            ESP_LOGI(HIDH_TAG, ESP_BD_ADDR_STR " close: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
            break;
        }
        default: {
            ESP_LOGI(HIDH_TAG, "event: %d", event);
            break;
        }
    }   
}

/**
 * @brief scans for a specified device mac address and connects to the device
 * @param device pointer to bt_device_struct_t
*/
void connect_device(bt_device_struct_t* device) {
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;

    ESP_LOGI(TAG, "scanning for devices...");
    while (!device->connected) {
        esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);

        if (results_len) {
            esp_hid_scan_result_t *r = results;
            esp_hid_scan_result_t *target = NULL;

            while (r) {
                if (!compare_bd_addr(device->address, r->bda)) {
                    break;
                }

                ESP_LOGI(TAG, "target device found");

                target = r;

                printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                printf("rssi: %d, ", r->rssi);
                printf("usage: %s, ", esp_hid_usage_str(r->usage));

                if (r->transport == ESP_HID_TRANSPORT_BLE) {
                    printf("appearance: 0x%04x, ", r->ble.appearance);
                    printf("addr_type: '%s', ", ble_addr_type_str(r->ble.addr_type));
                }

                printf("name: %s ", r->name ? r->name : "");
                printf("\n");
                r = r->next;
            }

            if (target && !device->connected) {
                ESP_LOGI(TAG, "connecting...");
                esp_hidh_dev_open(target->bda, target->transport, target->ble.addr_type);
                device->connected = 1;
            }

            esp_hid_scan_results_free(results);
        }
    }
}

/**
 * @note debug
 * @brief gets and print the connection parameters for a connected device
 * @param pvParameters undefined
*/
void get_params_task(void* pvParameters) {
    esp_gap_conn_params_t p = {0};

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(esp_ble_get_current_conn_params(mouse.address, &p));
        ESP_LOGW(TAG, "conn_params: interval=%d, latency=%d, timeout=%d", p.interval, p.latency, p.timeout);
    }

    vTaskDelete(NULL);
}

void connect_computer(bt_device_struct_t *computer) {
    ESP_LOGI(TAG, "waiting computer connexion...");

    while (!computer->connected) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
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

    // esp_ble_gap_set_prefer_conn_params(): before connection, master role only
    // esp_ble_gap_update_conn_params(): can only be used when the connexion is up
    // esp_ble_gap_prefer_ext_connect_params_set(): set aux parameters

#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "please turn on BT HID host or BLE!");
    return;
#endif

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(9));
    // ESP_ERROR_CHECK(esp_ble_gap_set_prefer_conn_params());
    
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_HOST_MODE));

    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, hidd_config.device_name));
    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return;
    }
    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&hidd_config, ESP_HID_TRANSPORT_BLE, hidd_callback, &hid_mouse.hid_dev));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler));
    ESP_ERROR_CHECK(esp_hidh_init(&config));

    connect_computer(&computer);
    connect_device(&mouse);

    xTaskCreate(&get_params_task, "get_params_task", 6 * 1024, NULL, 2, NULL);
}
