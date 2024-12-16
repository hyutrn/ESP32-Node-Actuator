#include "nvs_storage.h"

// Hàm lưu SSID và Password vào NVS
void save_wifi_credentials(const char* ssid, const char* password) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_store", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        printf("Error opening NVS handle!\n");
        return;
    }

    // Lưu SSID và Password
    err = nvs_set_str(nvs_handle, "ssid", ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(nvs_handle, "password", password);
    }

    if (err == ESP_OK) {
        nvs_commit(nvs_handle);
        printf("WiFi credentials saved successfully!\n");
    } else {
        printf("Failed to save WiFi credentials: %s\n", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
}

// Hàm đọc SSID và Password từ NVS
bool load_wifi_credentials(char* ssid, size_t ssid_size, char* password, size_t password_size) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_store", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        printf("No WiFi credentials found in NVS!\n");
        return false;
    }

    // Đọc SSID và Password
    err = nvs_get_str(nvs_handle, "ssid", ssid, &ssid_size);
    if (err == ESP_OK) {
        err = nvs_get_str(nvs_handle, "password", password, &password_size);
    }

    nvs_close(nvs_handle);

    if (err == ESP_OK) {
        printf("WiFi credentials loaded: SSID=%s\n", ssid);
        return true;
    } else {
        printf("Failed to load WiFi credentials: %s\n", esp_err_to_name(err));
        return false;
    }
}
