#include "light_sen.h" 

int calculate(int reading) {
    return (reading * 3300) / (pow(2, pm_adc_bioot_width) - 1);
}

void light_num(gpio_num_t light) {
    // Configure ADC channels
    adc1_config_width(ADC_WIDTH_BIT_12);

   int adc1_channel;
    if (light == GPIO_NUM_25 || light == GPIO_NUM_26 || light == GPIO_NUM_27) {
        adc1_channel = light - GPIO_NUM_25;
    } else {
        printf("Invalid GPIO for Light Sensor\n");
        return;
    }

    adc1_config_channel_atten(adc1_channel, ADC_ATTEN_DB_11);

    int adc1_reading[5];
    int adc1_value = 0;

    for (int i = 0; i < 5; i++) {
        adc1_reading[i] = adc1_get_raw(adc1_channel);
    }

    adc1_value = 0;
    for (int i = 0; i < 5; i++) {
        adc1_value += adc1_reading[i];
    }
    adc1_value = adc1_value / 5;

    // Đặt ngưỡng giới hạn để xác định nếu có cảm biến được kết nối
    int invalid_threshold = 500; // Có thể cần điều chỉnh giá trị này dựa trên thực tế

    if (adc1_value < invalid_threshold) {
        printf("Sensor not connected. Please check again!\n\n");
    } else {
        int light_min = 0;
        int light_max = 3300;

        float mois_percentage = ((float)(adc1_value - light_min) / (light_max - light_min)) * 100.0;
        char value[14] = { '\0'};
        snprintf(value, 14, "%.2f", mois_percentage);
        printf("Light Sensor: %s%%\n\n", value);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}

void light_init(gpio_num_t light_1, gpio_num_t light_2, gpio_num_t light_3) {
    printf("light sensor 1: \n");
    light_num(light_1);
    printf("light snesor 2: \n");
    light_num(light_2);
    printf("light sensor 3: \n");
    light_num(light_3);
}