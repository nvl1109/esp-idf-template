#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"

#include "esp32_keypad.h"

#define KEYPAD_TAG "KEYPAD"

static void _s_keypad_task_handler(void *arg);

int keypad_init(struct keypad_config *config)
{
    int ret;
    int i,j;
    gpio_config_t io_conf;
    uint64_t tmp_mask;

    if ((config->_state == KEYPAD_ST_INIT) ||
        (config->_state == KEYPAD_ST_IDLE) ||
        (config->_state == KEYPAD_ST_SCANNING)) {
        ESP_LOGI(KEYPAD_TAG, "Already inited");
        return KEYPAD_OK;
    }

    // Configure GPIO pins for ROWs
    tmp_mask = 0;
    for (i = 0; i < config->row_num; ++i) {
        tmp_mask |= 1 << config->row_pins[i];
    }
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = tmp_mask;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    ret = gpio_config(&io_conf);
    if (ESP_OK != ret) {
        ESP_LOGE(KEYPAD_TAG, "Configure ROW pins failed");
        return KEYPAD_ERR;
    }

    // Set all rows to LOW
    for (i = 0; i < config->row_num; ++i) {
        gpio_set_level(config->row_pins[i], 0);
    }

    // Configure GPIO pins for COLs
    tmp_mask = 0;
    for (i = 0; i < config->col_num; ++i) {
        tmp_mask |= 1 << config->col_pins[i];
    }
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = tmp_mask;
    //Enable pull-down mode
    io_conf.pull_down_en = 1;
    ret = gpio_config(&io_conf);
    if (ESP_OK != ret) {
        ESP_LOGE(KEYPAD_TAG, "Configure COL pins failed");
        return KEYPAD_ERR;
    }

    // Create message queue
    config->_data_queue = xQueueCreate(KPAD_MAX_QUEUE, sizeof(struct keypad_message));
    if (NULL == config->_data_queue) {
        ESP_LOGE(KEYPAD_TAG, "Create message queue failed");
        return KEYPAD_ERR;
    }

    // Update state
    config->_state = KEYPAD_ST_INIT;
    config->_is_special = 0;

    return KEYPAD_OK;
}

int keypad_start(struct keypad_config *config)
{
    int ret;

    if (config->_state != KEYPAD_ST_INIT) {
        ESP_LOGE(KEYPAD_TAG, "Keypad is not initialized");
        return KEYPAD_ERR;
    }

    ret = xTaskCreate(_s_keypad_task_handler, "keypad", 2048, NULL, 10, &config->_taskhd);
    if (pdPASS != ret) {
        ESP_LOGE(KEYPAD_TAG, "Task create FAILED");
        return KEYPAD_ERR;
    }

    return KEYPAD_OK;
}

int keypad_deinit(struct keypad_config *config)
{
    if ((config->_state < KEYPAD_ST_INIT) || (config->_state == KEYPAD_ST_MAX)) {
        ESP_LOGD(KEYPAD_TAG, "Already de-inited or not inited");
        return KEYPAD_OK;
    }

    config->_state = KEYPAD_ST_INIT;
    vTaskDelete(config->_taskhd);

    vQueueDelete(config->_data_queue);
    config->_state = KEYPAD_ST_MAX;

    return KEYPAD_OK;
}

int keypad_get_msg(struct keypad_config *config, struct keypad_message *msg)
{

    return KEYPAD_OK;
}

static void _s_keypad_task_handler(void *arg)
{
    while((config->_state == KEYPAD_ST_IDLE) || (config->_state == KEYPAD_ST_SCANNING)) {
        // scan keypad

        // delay
        vTaskDelay(config->sleep_delay_ms / portTICK_PERIOD_MS);
    }
}
