
#include <stdint.h>

void RPM_Init(uint32_t tickRate_us, uint32_t timeWindow_ticks);

void RPM_getAllMeasured_RPM(uint16_t *measured_RPM, uint16_t now_ticks);

void RPM_event(uint8_t channel, uint16_t timestamp);
