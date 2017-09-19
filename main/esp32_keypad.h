#ifndef __ESP32_KEYPAD_H__
#define __ESP32_KEYPAD_H__ 1
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#define KPAD_MAX_ROW 10U
#define KPAD_MAX_COL 10U
#define KPAD_MAX_QUEUE 10U

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

struct keypad_special_seq {
    uint8_t cmd;
    uint8_t sequence[4];
};

struct keypad_message {
    uint8_t msg[4];
    uint8_t len;
};

struct keypad_config {
    // Public members
    uint8_t row_num;
    uint8_t col_num;
    uint8_t row_pins[KPAD_MAX_ROW];
    uint8_t col_pins[KPAD_MAX_COL];
    uint8_t chars[KPAD_MAX_ROW][KPAD_MAX_COL];
    uint8_t special_key;
    struct keypad_special_seq special_seq[KPAD_MAX_ROW];
    uint32_t sleep_delay_ms;
    // Private members
    keypad_state_t _state;
    EventGroupHandle_t _event;
    TaskHandle_t _taskhd;
    QueueHandle_t _data_queue;
    uint8_t _is_special;
};

int keypad_init(struct keypad_config *config);
int keypad_start(struct keypad_config *config);
int keypad_get_msg(struct keypad_config *config, struct keypad_message *msg);
int keypad_deinit(struct keypad_config *config);

#endif /* __ESP32_KEYPAD_H__ */
