#ifndef LIGHT_SEN_H_
#define LIGHT_SEN_H_

#include <stdio.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "cJSON.h"

#define TRUE (1==1)
#define FALSE (!TRUE)

#define pm_adc_bioot_width ADC_WIDTH_BIT_DEFAULT 
#define us_to_s_factor (uint64_t)1000000
#define pm_measure_every_mins (uint64_t)60

int calculate(int reading);
void light_num(gpio_num_t light);
void light_init(gpio_num_t light_1, gpio_num_t light_2, gpio_num_t light_3);
#endif