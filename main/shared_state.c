#include "shared_state.h"
#include <string.h>

car_state_t g_car;

void shared_state_init(void)
{
    memset(&g_car, 0, sizeof(g_car));
    g_car.mutex = xSemaphoreCreateMutex();
    atomic_store(&g_car.mode, SUSPENSION_MODE_PASSIVE);
}
