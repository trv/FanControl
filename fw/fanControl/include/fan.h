
#include <stdint.h>

#define FAN_NUM_CHANNELS		4

void fan_Init(void);

void fan_setAllTarget_mV(uint16_t *target_mV);
void fan_setAllTarget_RPM(uint16_t *target_RPM);	// TODO!

void fan_getAllMeasured_mV(uint16_t *measured_mV);
void fan_getAllMeasured_PWM(uint16_t *measured_PWM);
void fan_getAllMeasured_RPM(uint16_t *measured_RPM);
void fan_getAllMax_RPM(uint16_t *max_RPM);
