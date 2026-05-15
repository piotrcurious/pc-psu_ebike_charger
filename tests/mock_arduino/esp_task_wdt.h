#ifndef ESP_TASK_WDT_H
#define ESP_TASK_WDT_H
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
void esp_task_wdt_init(int timeout, bool panic);
void esp_task_wdt_add(void* handle);
void esp_task_wdt_reset();
#ifdef __cplusplus
}
#endif
#endif
