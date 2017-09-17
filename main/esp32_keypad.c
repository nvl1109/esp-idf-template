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

int keypad_init(struct keypad_config *config)
{

    return KEYPAD_OK;
}

int keypad_start(struct keypad_config *config)
{

    return KEYPAD_OK;
}

int keypad_deinit(struct keypad_config *config)
{

    return KEYPAD_OK;
}

int keypad_get_val(struct keypad_config *config, uint32_t *val)
{

    return KEYPAD_OK;
}
