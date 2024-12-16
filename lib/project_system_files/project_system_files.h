#ifndef __PROJECT_SYSTEM_FILES_H__
#define __PROJECT_SYSTEM_FILES_H__

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


void files_spiffs_init(char* partition_label, size_t max_files, bool format_if_mount_failed);

long calculate_file_length(FILE *file);

#endif // __PROJECT_SYSTEM_FILES_H__
