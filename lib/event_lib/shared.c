#include "shared.h"

// Khai báo Event Group
EventGroupHandle_t shared_event_group = NULL;

// Hàm khởi tạo Event Group
void shared_event_group_init(void) {
    if (shared_event_group == NULL) {
        shared_event_group = xEventGroupCreate();
    }
}
