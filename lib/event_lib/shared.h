#ifndef SHARED_H_
#define SHARED_H_

#include <stdio.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "freertos/event_groups.h"

// Định nghĩa Event Group và các bit sự kiện
extern EventGroupHandle_t shared_event_group;

// Các bit sự kiện
#define EVENT_CLIENT_POSTED BIT0

// Hàm khởi tạo Event Group
void shared_event_group_init(void);

extern bool motor_1; 
extern bool motor_2;
extern bool motor_3;
extern bool motor_4;

#endif // SHARED_H
