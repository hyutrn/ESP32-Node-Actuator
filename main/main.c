#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "string.h"
#include <sys/param.h>
#include <stdbool.h>
#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi_pro.h"
#include "server_cfg.h"
#include "dht11.h"
#include "light_sen.h"
#include "mois.h"
#include "mqtt_cfg.h" 
#include "shared.h"

// Khai báo chân GPIO cho các nút nhấn
#define BUTTON_1 GPIO_NUM_16 //Reset button
#define BUTTON_2 GPIO_NUM_17 //Restart button
#define BUTTON_3 GPIO_NUM_18 //Motor 1
#define BUTTON_4 GPIO_NUM_19 //Motor 2
#define BUTTON_5 GPIO_NUM_20 //Motor 3 

// Khai bao cac chan GPIO dieu khien relay
#define MOTOR_1 GPIO_NUM_21
#define MOTOR_2 GPIO_NUM_22
#define MOTOR_3 GPIO_NUM_23

// Semaphore cho từng nút nhấn
SemaphoreHandle_t xSemaphoreButton1 = NULL;
SemaphoreHandle_t xSemaphoreButton2 = NULL;
SemaphoreHandle_t xSemaphoreButton3 = NULL;
SemaphoreHandle_t xSemaphoreButton4 = NULL;
SemaphoreHandle_t xSemaphoreButton5 = NULL;
SemaphoreHandle_t xSemaphoreUpdateButton = NULL;

EventGroupHandle_t event_group;
const int EVENT_INIT_DONE = BIT0;

// Hàm xử lý ngắt ISR cho BUTTON_1
void IRAM_ATTR button1_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(xSemaphoreButton1, NULL);
}

// Hàm xử lý ngắt ISR cho BUTTON_2
void IRAM_ATTR button2_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(xSemaphoreButton2, NULL);
}

// Ham xu ly ngat ISR cho BUTTON_3
void IRAM_ATTR button3_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(xSemaphoreButton3, NULL);
}

void IRAM_ATTR button4_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(xSemaphoreButton4, NULL);
}

void IRAM_ATTR button5_isr_handler(void* arg) {
    xSemaphoreGiveFromISR(xSemaphoreButton5, NULL);
}

// Task xử lý BUTTON_1
void button1_task(void* arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreButton1, portMAX_DELAY) == pdTRUE) {
            printf("Button 1 pressed! Restart\n");

            // Debounce cho BUTTON_1
            gpio_intr_disable(BUTTON_1);
            while (!gpio_get_level(BUTTON_1)) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_intr_enable(BUTTON_1);

            //Restart module ESP
            esp_restart();
        }
    }
}

// Task xử lý BUTTON_2
void button2_task(void* arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreButton2, portMAX_DELAY) == pdTRUE) {
            printf("Button 2 pressed! Reset\n");

            // Debounce cho BUTTON_2
            gpio_intr_disable(BUTTON_2);
            while (!gpio_get_level(BUTTON_2)) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_intr_enable(BUTTON_2);

            // Xóa dữ liệu NVS
            esp_err_t err = nvs_flash_erase();
            if (err == ESP_OK) {
                printf("NVS erased successfully.\n");
            } else {
                printf("Failed to erase NVS: %s\n", esp_err_to_name(err));
            }

            // Restart ESP32
            esp_restart();
        }
    }
}

// Task xử lý BUTTON_3
void button3_task(void* arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreButton3, portMAX_DELAY) == pdTRUE) {
            printf("Button 3 pressed! Reset\n");

            // Debounce cho BUTTON_3
            gpio_intr_disable(BUTTON_3);
            while (!gpio_get_level(BUTTON_3)) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_intr_enable(BUTTON_3);
        }
    }
}

// Task xử lý BUTTON_4
void button4_task(void* arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreButton4, portMAX_DELAY) == pdTRUE) {
            printf("Button 4 pressed! Reset\n");

            // Debounce cho BUTTON_4
            gpio_intr_disable(BUTTON_4);
            while (!gpio_get_level(BUTTON_4)) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_intr_enable(BUTTON_4);
        }
    }
}

// Task xử lý BUTTON_5
void button5_task(void* arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreButton5, portMAX_DELAY) == pdTRUE) {
            printf("Button 5 ressed! Reset\n");

            // Debounce cho BUTTON_5
            gpio_intr_disable(BUTTON_5);
            while (!gpio_get_level(BUTTON_5)) {
                vTaskDelay(5 / portTICK_PERIOD_MS);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_intr_enable(BUTTON_5);
        }
    }
}

// Hàm khởi tạo các nút nhấn và LED
void init_gpio(void) {
    // Cấu hình GPIO cho các nút nhấn
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << BUTTON_1) | (1ULL << BUTTON_2) | (1ULL << BUTTON_3) | (1ULL << BUTTON_4) | (1ULL << BUTTON_5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_config);

    // Cài đặt ISR cho từng nút nhấn
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_1, button1_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_2, button2_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_3, button3_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_4, button4_isr_handler, NULL);
    gpio_isr_handler_add(BUTTON_5, button5_isr_handler, NULL);
}

void updateButtonTask(void *arg){
    while(1){
        ESP_LOGI("MAIN", "Main program running...");
        dataSend(client);
        vTaskDelay(4000/portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

     // Khởi tạo Event Group
    shared_event_group_init();

    initialise_wifi();

    // Chờ đợi sự kiện EVENT_CLIENT_POSTED
    printf("Waiting for init node...\n");
    xEventGroupWaitBits(shared_event_group, EVENT_CLIENT_POSTED, pdTRUE, pdTRUE, portMAX_DELAY);
    printf("Init Wifi and Server done!\n");


    // Tạo Semaphore cho từng nút nhấn
    xSemaphoreButton1 = xSemaphoreCreateBinary();
    xSemaphoreButton2 = xSemaphoreCreateBinary();
    xSemaphoreButton3 = xSemaphoreCreateBinary();
    xSemaphoreButton4 = xSemaphoreCreateBinary();
    xSemaphoreButton5 = xSemaphoreCreateBinary();
    xSemaphoreUpdateButton = xSemaphoreCreateBinary();

    // Tạo task xử lý từng task, nút nhấn
    xTaskCreate(button1_task, "Button1Task", 2048, NULL, 10, NULL);
    xTaskCreate(button2_task, "Button2Task", 2048, NULL, 10, NULL);
    xTaskCreate(button3_task, "Button3Task", 2048, NULL, 10, NULL);
    xTaskCreate(button4_task, "Button4Task", 2048, NULL, 10, NULL);
    xTaskCreate(button5_task, "Button5Task", 2048, NULL, 10, NULL);
    xTaskCreate(updateButtonTask, "updateButtonTask", 2096, NULL, 9, NULL);

    // Khởi tạo các nút nhấn và LED
    init_gpio();
}
