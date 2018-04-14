
#include <stdint.h>

#define FAN_NUM_CHANNELS		4

void Fan_Init(void);

void Fan_setAllTarget_mV(uint16_t *target_mV);
void Fan_setAllTarget_RPM(uint16_t *target_RPM);	// TODO!

void Fan_getAllMeasured_mV(uint16_t *measured_mV);
void Fan_getAllMeasured_PWM(uint16_t *measured_PWM);
void Fan_getAllMeasured_RPM(uint16_t *measured_RPM);
void Fan_getAllMax_RPM(uint16_t *max_RPM);
