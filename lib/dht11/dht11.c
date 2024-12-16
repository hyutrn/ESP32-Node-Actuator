#include "dht11.h"

gpio_num_t dht_gpio;
int64_t last_read_time = -2000000;
struct dht11_reading dht11_last_read;


//DHT11
int DHT11_waitOrTimeout(uint16_t microSeconds, int level) {
    int micros_ticks = 0;
    while(gpio_get_level(dht_gpio) == level) { 
        if(micros_ticks++ > microSeconds) 
            return DHT11_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}
int DHT11_checkCRC(uint8_t data[]) {
    if(data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DHT11_OK;
    else
        return DHT11_CRC_ERROR;
}
void DHT11_sendStartSignal() {
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}

//DHT11 check 
int DHT11_checkResponse() {
    /* Wait for next step ~80us*/
    if(DHT11_waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;
    /* Wait for next step ~80us*/
    if(DHT11_waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR) 
        return DHT11_TIMEOUT_ERROR;
    return DHT11_OK;
}

//DHT11 ERROR
struct dht11_reading DHT11_timeoutError() {
    struct dht11_reading timeoutError = {DHT11_TIMEOUT_ERROR, -1, -1};
    return timeoutError;
}
struct dht11_reading DHT11_crcError() {
    struct dht11_reading crcError = {DHT11_CRC_ERROR, -1, -1};
    return crcError;
}

//DHT11 INIT
void dht11_init(gpio_num_t gpio_num) {
    /* Wait 1 seconds to make the device pass its initial unstable status */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    dht_gpio = gpio_num;
}

//DHT11 READ DATA
struct dht11_reading dht11_read() {
    /* Tried to sense too son since last read (dht11 needs ~2 seconds to make a new read) */
    if(esp_timer_get_time() - 2000000 < last_read_time) {
        return dht11_last_read;
    }
    last_read_time = esp_timer_get_time();
    uint8_t data[5] = {0,0,0,0,0};
    DHT11_sendStartSignal();
    if(DHT11_checkResponse() == DHT11_TIMEOUT_ERROR)
        return dht11_last_read = DHT11_timeoutError();
    /* Read response */
    for(int i = 0; i < 40; i++) {
        /* Initial data */
        if(DHT11_waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
            return dht11_last_read = DHT11_timeoutError();
                
        if(DHT11_waitOrTimeout(70, 1) > 28) {
            /* Bit received was a 1 */
            data[i/8] |= (1 << (7-(i%8)));
        }
    }
    if(DHT11_checkCRC(data) != DHT11_CRC_ERROR) {
        dht11_last_read.status = DHT11_OK;
        dht11_last_read.temperature = data[2];
        dht11_last_read.humidity = data[0];
        return dht11_last_read;
    } 
    else {
        return dht11_last_read = DHT11_crcError();
    }
}

//DHT11 
void dht_num(gpio_num_t gpio_num) {
    dht11_init(gpio_num);
    int air_temp, status, air_humi;
    
    // Read data from sensor 1
    struct dht11_reading reading_1 = dht11_read();
    air_temp = reading_1.temperature;
    status = reading_1.status;
    air_humi = reading_1.humidity;
    //printf("Status is %d\n", status);
    if(status != -1) {
        printf("Temperature is: %d°C\n", air_temp);
        printf("Humidity is: %d%%\n\n", air_humi);
    } else {
        printf("DHT11 sensor not connected. Please check again!\n\n");
    }
    vTaskDelay(1000/ portTICK_PERIOD_MS); //delay 1s
}
void dht_init(gpio_num_t dht_1, gpio_num_t dht_2, gpio_num_t dht_3) {
    printf("DHT11 sensor 1:\n");
    dht_num(dht_1);
    printf("DHT11 sensor 2:\n");
    dht_num(dht_2);
    printf("DHT11 sensor 3:\n");
    dht_num(dht_3);
    vTaskDelay(500/portTICK_PERIOD_MS);
}

