#ifndef __TASK_BOOTCTL_H
#define __TASK_BOOTCTL_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

void TaskBootCtl_Init(uint32_t period);
void TaskBootCtl_Core(const void *argument);

#endif
