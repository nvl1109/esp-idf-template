#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "esp32_keypad.h"

#define MAIN_TAG "MAIN"

struct keypad_config key_cfg = {
    .row_num = 4,
    .col_num = 3,
    .row_pins = {GPIO_NUM_26, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32},
    .col_pins = {GPIO_NUM_35, GPIO_NUM_34, GPIO_NUM_39},
    .chars = {
        {'1', '2', '3'},
        {'4', '5', '6'},
        {'7', '8', '9'},
        {'*', '0', '#'},
    },
    .special_key = '*',
    .special_seq = {
        {.cmd = 1, .sequence = {'*', '1', '\0'},},
        {.cmd = 2, .sequence = {'*', '2', '\0'},},
    },
    .sleep_delay_ms = 10,
};

void app_main(void)
{
    // nvs_flash_init();
    // tcpip_adapter_init();
    // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    // ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    // ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    // ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_NULL) );

    struct keypad_config *key = &key_cfg;

    int ret = keypad_init(key);
    if (KEYPAD_OK != ret) {
        ESP_LOGE(MAIN_TAG, "Keypad init failed");
        return;
    }
    ret = keypad_start(key);
    if (KEYPAD_OK != ret) {
        ESP_LOGE(MAIN_TAG, "Keypad start failed");
        return;
    }

    struct keypad_message msg;
    while (1) {
        ret = keypad_get_msg(key, &msg, 20);
        if (ret != KEYPAD_OK) continue;

        ESP_LOGI(MAIN_TAG, "Get key: %s, len: %d", msg.msg, msg.len);
    }
}
