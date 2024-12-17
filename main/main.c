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

#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "wifi_pro.h"
#include "server_cfg.h"
#include "dht11.h"
#include "light_sen.h"
#include "mois.h"
#include "mqtt_cfg.h" 
#include "shared.h"

#include "esp_lcd_sh1107.h"
#include "esp_lcd_panel_vendor.h"
static const char *TAG = "Actuator";
#define I2C_HOST  0
#define LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define PIN_NUM_SDA           21
#define PIN_NUM_SCL           22
#define PIN_NUM_RST           -1
#define I2C_HW_ADDR           0x3C
// The pixel number in horizontal and vertical
#define LCD_H_RES              128
#define LCD_V_RES              64
// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}
QueueHandle_t oled_queue = NULL; // Queue để gửi dữ liệu hiển thị

lv_disp_t *disp = NULL; // Biến hiển thị của LVGL

typedef struct {
    char line1[64]; // Nội dung hiển thị dòng 1
    char line2[64]; // Nội dung hiển thị dòng 2
} oled_msg_t;

// Task hiển thị OLED với hiệu ứng chạy ngang
void oled_task(void *arg) {
    oled_msg_t msg;
    while (1) {
        // Chờ dữ liệu từ Queue
        if (xQueueReceive(oled_queue, &msg, portMAX_DELAY) == pdTRUE) {
            lv_obj_t *scr = lv_disp_get_scr_act(disp);
            lv_obj_clean(scr); // Xóa nội dung cũ

            // Tạo nhãn cho dòng 1
            lv_obj_t *label1 = lv_label_create(scr);
            lv_label_set_long_mode(label1, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label1, msg.line1);
            lv_obj_set_width(label1, disp->driver->hor_res);
            lv_obj_align(label1, LV_ALIGN_TOP_MID, 0, 0); // Căn giữa ở trên cùng

            // Tạo nhãn cho dòng 2
            lv_obj_t *label2 = lv_label_create(scr);
            lv_label_set_long_mode(label2, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
            lv_label_set_text(label2, msg.line2);
            lv_obj_set_width(label2, disp->driver->hor_res);
            lv_obj_align(label2, LV_ALIGN_LEFT_MID, 0, 0); // 

            ESP_LOGI(TAG, "OLED displayed: %s | %s", msg.line1, msg.line2);
        }
    }
}


// Hàm khởi tạo OLED
void oled_init() {
    ESP_LOGI(TAG, "Initialize OLED");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_NUM_SDA,
        .scl_io_num = PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = I2C_HW_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_bit_offset = 6,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = PIN_NUM_RST,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LCD_H_RES * LCD_V_RES,
        .double_buffer = true,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = true,
    };
    disp = lvgl_port_add_disp(&disp_cfg);
}

// Khai báo chân GPIO cho các nút nhấn
#define BUTTON_1 GPIO_NUM_12 //Reset button
#define BUTTON_2 GPIO_NUM_13 //Restart button
#define BUTTON_3 GPIO_NUM_14 //Motor 1
#define BUTTON_4 GPIO_NUM_15 //Motor 2
#define BUTTON_5 GPIO_NUM_16 //Motor 3 

// Khai bao cac chan GPIO dieu khien relay
#define MOTOR_1 GPIO_NUM_17
#define MOTOR_2 GPIO_NUM_18
#define MOTOR_3 GPIO_NUM_19

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
