#ifndef DHT11_H_
#define DHT11_H_

#include <stdio.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "cJSON.h"

//Status error 
enum dht11_status {
    DHT11_CRC_ERROR = -2,
    DHT11_TIMEOUT_ERROR,
    DHT11_OK
};

//Struct data sensor
struct dht11_reading {
    int status;
    int temperature;
    int humidity;
};


int DHT11_waitOrTimeout(uint16_t microSeconds, int level);
int DHT11_checkCRC(uint8_t data[]);
void DHT11_sendStartSignal();
int DHT11_checkResponse();
struct dht11_reading DHT11_timeoutError();
struct dht11_reading DHT11_crcError();
void dht11_init(gpio_num_t gpio_num);
struct dht11_reading dht11_read();
void dht_num(gpio_num_t gpio_num);
void dht_init(gpio_num_t dht_1, gpio_num_t dht_2, gpio_num_t dht_3);
#endif