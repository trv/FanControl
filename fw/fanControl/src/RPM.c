/*
RPM calcualtion

data:
	can be lower-resolution timer increments (16 bit per half-second = 10µs period = 100kHz)
	6000 RPM = 400Hz (at 4 edges per revolution) = 200 sample buffer maximum for half-second
	300 RPM = 20Hz = 10 samples in half-second window (50ms period)

	FIFO: store head/tail pointer, tail may be unnecessary if timestamps expire naturally
	Windowing: add fade-in and fade-out time
		aim to cover ~2 samples minimum?  100ms fade-in, 300ms full-on, 100ms fade-out
		straight trapezoidal ramp vs something fancier?
		need to calculate equivalent divisor for windowed samples - trapezoid would be 400ms

*/
#include "RPM.h"

#include <string.h>
#include "Fan.h"

#define MAX_LENGTH		256
#define TICKS_PER_REV	4

uint8_t event_index[FAN_NUM_CHANNELS];
uint16_t events[FAN_NUM_CHANNELS][MAX_LENGTH];
uint16_t rpm[FAN_NUM_CHANNELS] = {0};

uint32_t g_timeWindow_ticks;
uint32_t g_tickRate_us;

void RPM_Init(uint32_t tickRate_us, uint32_t timeWindow_ticks)
{
	g_timeWindow_ticks = timeWindow_ticks;
	g_tickRate_us = tickRate_us;

	memset(event_index, 0, sizeof(event_index));
	memset(events, 0, sizeof(events));
	memset(rpm, 0, sizeof(rpm));
}

#define FADE_DIV		3
#define FADE_COMP	(FADE_DIV - 1)
void RPM_getAllMeasured_RPM(uint16_t *measured_RPM, uint16_t now_ticks)
{
	uint16_t windowStart = now_ticks - g_timeWindow_ticks;
	uint16_t fade_time = g_timeWindow_ticks / FADE_DIV;
	int i = 0;
	uint32_t count = 0;
	uint8_t index = event_index[i] - 1;
	while (index != event_index[i]) {
		uint16_t delta_ticks = now_ticks - events[i][index];
		if (delta_ticks < fade_time) {
			count += (1000 * delta_ticks + fade_time/2) / fade_time;
		} else if (delta_ticks < g_timeWindow_ticks - fade_time) {
			count += 1000;
		} else if (delta_ticks < g_timeWindow_ticks) {
			count += (1000 * (g_timeWindow_ticks - delta_ticks) + fade_time/2) / fade_time;
		} else if (count != 0) {
			break;
		}
		index--;
	}
	uint32_t divisor = (FADE_COMP*g_timeWindow_ticks*g_tickRate_us)/FADE_DIV;
	measured_RPM[i] = (count*15000 + divisor/2)/divisor;
}

void RPM_event(uint8_t channel, uint16_t timestamp)
{
	events[channel][event_index[channel]++] = timestamp;
}

#ifdef RPM_TEST
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
int main(int argc, char *argv[])
{
	uint16_t sim_RPM = 300;

	if (argc == 2) {
		sim_RPM = atoi(argv[1]);
	}

	uint16_t rpm[FAN_NUM_CHANNELS];
	RPM_Init(20, 25000);		// 20us * 25000 ticks = 0.5 seconds

	uint32_t now_ticks = 30000 * 65536;	// init current time at least a window past the start


	uint32_t period = (50000u * 65536u / sim_RPM) * 15;
	uint32_t jitter = period / 20;

	uint16_t meas_ticks = now_ticks >> 16;
	uint16_t meas_period = 100;

	// populate events
	bool even = true;
	uint32_t last_tick = 0;
	for (int i = 0; i < 500; i++) {
		uint16_t event_tick = 0;
		if (true) {
			event_tick = (now_ticks + rand() % jitter) >> 16;
		} else {
			event_tick = (now_ticks - rand() % jitter) >> 16;
		}
		even = !even;

		RPM_event(0, event_tick);
		printf("    event at %6d\n", event_tick);
		last_tick = now_ticks;
		now_ticks += period;

		// test reported RPM
		uint16_t delta = (now_ticks >> 16) - meas_ticks;
		while (delta > meas_period) {
			RPM_getAllMeasured_RPM(rpm, meas_ticks);
			printf("time: %8d\trpm: %4d\n", meas_ticks, rpm[0]);
			meas_ticks += meas_period;
			delta -= meas_period;
		}
	}

	printf("\nactual RPM: %4d\n", (65536u*50000u)/(period/15u));
	printf("actual period32: %u\n", period);
	printf("actual period16: %u\n", period>>16);

}
#endif
