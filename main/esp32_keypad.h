#ifndef __ESP32_KEYPAD_H__
#define __ESP32_KEYPAD_H__ 1
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define KPAD_MAX_ROW 10U
#define KPAD_MAX_COL 10U

typedef enum keypad_err_t {
    KEYPAD_OK = 0,
    KEYPAD_ERR,
    KEYPAD_ERR_MAX,
} keypad_err_t;

typedef enum keypad_state_t {
    KEYPAD_ST_UNKNOWN = 0,
    KEYPAD_ST_INIT,
    KEYPAD_ST_IDLE,
    KEYPAD_ST_SCANNING,
    KEYPAD_ST_MAX,
} keypad_state_t;

struct keypad_config {
    keypad_state_t state;
    uint8_t row_num;
    uint8_t col_num;
    uint8_t rows[KPAD_MAX_ROW];
    uint8_t cols[KPAD_MAX_COL];
    uint32_t sleep_delay_ms;
    EventGroupHandle_t event;
    TaskHandle_t taskhd;
    QueueHandle_t data_queue;
};

int keypad_init(struct keypad_config *config);
int keypad_start(struct keypad_config *config);
int keypad_get_val(struct keypad_config *config, uint32_t *val);
int keypad_deinit(struct keypad_config *config);

#endif /* __ESP32_KEYPAD_H__ */
