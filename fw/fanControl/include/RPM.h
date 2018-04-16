
#include <stdint.h>

void RPM_Init(void);

void RPM_getAllMeasured_RPM(uint16_t *measured_RPM);

void RPM_event(uint8_t channel, uint16_t timestamp);
