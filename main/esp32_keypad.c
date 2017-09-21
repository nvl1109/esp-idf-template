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
#include "driver/timer.h"

#include "esp32_keypad.h"

#define KEYPAD_TAG "KEYPAD"

#define TIMER_DIVIDER   16               /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_FINE_ADJ   (1.4*(TIMER_BASE_CLK / TIMER_DIVIDER)/1000000) /*!< used to compensate alarm value */
#define TIMER_INTERVAL0_SEC   (3.4179)   /*!< test interval for timer 0 */

static void _s_keypad_task_handler(void *arg);
static uint8_t _s_keypad_scan(struct keypad_config *config);
static void IRAM_ATTR _s_timer_group0_isr(void *para);
static void _s_timer_start(int group, int idx);

int keypad_init(struct keypad_config *config)
{
    int ret;
    int i;
    gpio_config_t io_conf;
    uint64_t tmp_mask;
    // For timer
    timer_config_t tmr_cfg;

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

    // Init timer
    config->_timer_group = TIMER_GROUP_0;
    config->_timer_idx = TIMER_0;
    tmr_cfg.alarm_en = 1;
    tmr_cfg.auto_reload = 0;
    tmr_cfg.counter_dir = TIMER_COUNT_UP;
    tmr_cfg.divider = TIMER_DIVIDER;
    tmr_cfg.intr_type = TIMER_INTR_LEVEL;
    tmr_cfg.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(config->_timer_group, config->_timer_idx, &tmr_cfg);
    /*Stop timer counter*/
    timer_pause(config->_timer_group, config->_timer_idx);
    /*Load counter value */
    timer_set_counter_value(config->_timer_group, config->_timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(config->_timer_group, config->_timer_idx, TIMER_INTERVAL0_SEC * TIMER_SCALE - TIMER_FINE_ADJ);
    /*Enable timer interrupt*/
    timer_enable_intr(config->_timer_group, config->_timer_idx);
    /*Set ISR handler*/
    timer_isr_register(config->_timer_group, config->_timer_idx, _s_timer_group0_isr, (void*) config, ESP_INTR_FLAG_IRAM, NULL);

    // Update state
    config->_state = KEYPAD_ST_INIT;
    config->_is_special = 0;

    if (config->special_timeout_ms == 0) {
        config->special_timeout_ms = 1000; // Default special timeout is 1s
    }

    return KEYPAD_OK;
}

int keypad_start(struct keypad_config *config)
{
    int ret;

    if (config->_state != KEYPAD_ST_INIT) {
        ESP_LOGE(KEYPAD_TAG, "Keypad is not initialized");
        return KEYPAD_ERR;
    }

    ret = xTaskCreate(_s_keypad_task_handler, "keypad", 2048, (void *)config, 10, &config->_taskhd);
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

int keypad_get_msg(struct keypad_config *config, struct keypad_message *msg, uint32_t timeout_ms)
{
    if (pdTRUE != xQueueReceive(config->_data_queue, msg, timeout_ms/portTICK_PERIOD_MS)) {
        return KEYPAD_ERR;
    }
    return KEYPAD_OK;
}

static void _s_keypad_task_handler(void *arg)
{
    struct keypad_config *config = (struct keypad_config *)arg;
    uint8_t ret;

    while((config->_state == KEYPAD_ST_IDLE) || (config->_state == KEYPAD_ST_SCANNING)) {
        // delay
        vTaskDelay(config->sleep_delay_ms / portTICK_PERIOD_MS);

        // scan keypad
        ret = _s_keypad_scan(config);

        if (ret == 0) continue;

        if ((ret == config->special_key) && (!config->_is_special)) {
            // Special key, wait for another character
            config->_is_special = 1;
            // Add to buf
            config->_key_buf[0] = ret;
            config->_key_buf_idx = 1;
            // Start the timeout timer
            _s_timer_start(config->_timer_group, config->_timer_idx);
            // Also process this key
            struct keypad_message msg = {
                .len = 1,
                .msg = {ret},
            };
            if (pdTRUE != xQueueSend(config->_data_queue, &msg, 0)) {
                ESP_LOGE(KEYPAD_TAG, "Queue is full");
            }
        } else if (config->_is_special) {
            // Normal key after special key, still in special sequence
            config->_key_buf[config->_key_buf_idx++] = ret;
        } else {
            // Normal key, send message to queue
            struct keypad_message msg = {
                .len = 1,
                .msg = {ret},
            };

            if (pdTRUE != xQueueSend(config->_data_queue, &msg, 0)) {
                ESP_LOGE(KEYPAD_TAG, "Queue is full");
            }
        }
    }
}

static uint8_t _s_keypad_scan(struct keypad_config *config)
{
    uint8_t ret = 0;
    uint8_t i, j;
    int val = 0;

    for (i = 0; i < config->row_num; ++i) {
        // Set row pin
        gpio_set_level(config->row_pins[i], 1);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        for (j = 0; j < config->col_num; ++j) {
            // Get col pin
            val = gpio_get_level(config->col_pins[j]);
            if (val != 0) {
                // has button pressed
                ret = config->chars[i][j];
                break;
            }
        }
        gpio_set_level(config->row_pins[i], 0);
        if (0 != val) break;
    }

    return ret;
}

static void IRAM_ATTR _s_timer_group0_isr(void *para)
{
    struct keypad_config *config = (struct keypad_config *) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    int timer_idx = config->_timer_idx;

    if((intr_status & BIT(timer_idx)) && timer_idx == TIMER_0) {
        /*Timer0 is an example that doesn't reload counter value*/
        TIMERG0.hw_timer[timer_idx].update = 1;

        /* We don't call a API here because they are not declared with IRAM_ATTR.
           If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
           we can alloc this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API. */
        TIMERG0.int_clr_timers.t0 = 1;

        // Timer timeout, send _key_buf to queue
        struct keypad_message msg = {
            .len = config->_key_buf_idx,
        };
        memcpy(msg.msg, config->_key_buf, config->_key_buf_idx);
        config->_is_special = 0;
        config->_key_buf_idx = 0;
        xQueueSendFromISR(config->_data_queue, &msg, NULL);
    }
}

static void _s_timer_start(int group, int idx)
{
    /*Load counter value */
    timer_set_counter_value(group, idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(group, idx, TIMER_INTERVAL0_SEC * TIMER_SCALE - TIMER_FINE_ADJ);
    /*Start timer counter*/
    timer_start(group, idx);
}
