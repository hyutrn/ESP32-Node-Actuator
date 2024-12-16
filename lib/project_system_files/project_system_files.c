#include "project_system_files.h"
#include "esp_spiffs.h"


void files_spiffs_init(char* partition_label, size_t max_files, bool format_if_mount_failed)
{
  esp_vfs_spiffs_conf_t config = {
    .base_path = "/spiffs",
    .partition_label = partition_label,
    .max_files = max_files,
    .format_if_mount_failed = format_if_mount_failed,
  };
  esp_vfs_spiffs_register(&config);
}

long calculate_file_length(FILE *file) {
    long length;

    // Di chuyển con trỏ đến cuối file
    fseek(file, 0, SEEK_END);

    // Lấy vị trí con trỏ, đây chính là độ dài của file
    length = ftell(file);

    // Di chuyển con trỏ về đầu file
    fseek(file, 0, SEEK_SET);

    return length;
}
