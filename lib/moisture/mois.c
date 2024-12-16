#include "mois.h" 


int calculate_vdc_mv(int reading) {
    return (reading * 3300) / (pow(2, pm_adc_bioot_width) - 1);
}

void mois_num(gpio_num_t mois) {
    // Configure ADC channels
    adc1_config_width(ADC_WIDTH_BIT_12);

    int adc1_channel;
    if (mois >= GPIO_NUM_32 && mois <= GPIO_NUM_39) {
        adc1_channel = mois - GPIO_NUM_32;
    } else {
        printf("Invalid GPIO for ADC1\n");
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
        int mois_min = 0;
        int mois_max = 3300;

        float mois_percentage = ((float)(adc1_value - mois_min) / (mois_max - mois_min)) * 100.0;
        char value[14] = { '\0'};
        snprintf(value, 14, "%.2f", mois_percentage);
        printf("Mois Sensor: %s%%\n\n", value);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}
