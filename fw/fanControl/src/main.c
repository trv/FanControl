#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#include "stm32f10x_dbgmcu.h"

#include "RetargetSerial.h"
#include "Timer.h"
#include "LCD.h"
#include "Fan.h"
#include "RPM.h"

void Display_Update(void);


/*
UART1: serial output (PA9 = TX, PA10 = RX)
ADC: sample CH10-13 @ 200kHz, DMA to memory (double-buffered, use half-full and complete flags)
TIM2-3: output PWM at 200kHz, update based on ADC or RPM values
TIMx?: timer for EXTI-triggered GPIO RPM sense pins (internal only)
TIM14: trigger display refresh with latest ADC/PWM out/RPM in values
TIM15: LCD DMA timing
 */

/*
TODO:
- pulse-skipping for 100% duty cycle? need to periodically output high to discharge cap
- better RPM algorithm (report 0 when pulses stop, nicer filtering, disable interrupts after each edge?)
- RPM feedback loop to control voltage based on desired RPM
- fan hotplug detection (add, remove, stalled, etc)

- UI?
- SMBUS?
 */

static uint16_t target_mV[FAN_NUM_CHANNELS] = {12000, 12000, 12000, 12000};

static uint16_t max_RPM[FAN_NUM_CHANNELS] = {0};
static uint16_t max_mV[FAN_NUM_CHANNELS] = {0};


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int main(int argc, char* argv[])
{
	//DBGMCU_Config(DBGMCU_TIM15_STOP, ENABLE);
	RetargetSerial_Init();
	Fan_Init();
	Fan_setAllTarget_mV(target_mV);

	timer_start();
	timer_sleep(100);

	printf("FanControl\n");

	LCD_Init();

	char *hello = " Hello World!";
	char *test = "Test \xF4 \010\011\012\013\014\015\016\017";

	LCD_Write(LCD_Line1, 0, hello, strlen(hello));
	LCD_Write(LCD_Line2, 0, test, strlen(test));
	LCD_Update();
	timer_sleep(250);

	// wait at default max voltage to measure max RPM
	for (int i = 0; i < 4; i++) {
		timer_sleep(1000);
		Display_Update();
	}

	// reset target to 7.5V
	for (int i = 0; i < 4; i++) {
		target_mV[i] = 7500;
	}
	Fan_setAllTarget_mV(target_mV);

	//size_t i = 0;

	for (;;) {
		/*
		if (i > 16) {
			if (i & 0x10) {
				target_mV[0] = 6000;
				target_mV[1] = 9000;
			} else {
				target_mV[0] = 9000;
				target_mV[1] = 6000;
			}
			target_mV[2] = 7500;
			target_mV[3] = 7500;
		}
		i++;
		Fan_setAllTarget_mV(target_mV);
		*/
		timer_sleep(1000);

		Display_Update();
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

void Display_Update(void)
{
	char line1[17];
	char line2[17];

	uint16_t max_PWM = 240;

	uint16_t measured_mV[FAN_NUM_CHANNELS] = {0};
	uint16_t measured_PWM[FAN_NUM_CHANNELS] = {0};
	uint16_t measured_RPM[FAN_NUM_CHANNELS] = {0};

	Fan_getAllMeasured_mV(measured_mV);
	Fan_getAllMeasured_PWM(measured_PWM);

	printf("\033[H\033[J");

	RPM_getAllMeasured_RPM(measured_RPM);

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		if (measured_RPM[i] > max_RPM[i]) {
			max_RPM[i] = measured_RPM[i];
		}
		if (measured_mV[i] > max_mV[i]) {
			max_mV[i] = measured_mV[i];
		}
	}

	printf("\033[37;1m\033[40m\033[1m%8s%8s%8s%8s%8s%8s%8s\033[0m\r\n", "channel", "target", "actual", "max mV", "PWM", "RPM", "MaxRPM");
	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		printf("%8d%8d%8d%8d%8d%8d%8d\r\n", i+1, target_mV[i], measured_mV[i], max_mV[i], measured_PWM[i], measured_RPM[i], max_RPM[i]);
	}

	uint8_t adc_target, adc_value, pwm_value, rpm_value;

	const char *lookup1 = "        \010\011\012\013\014\015\016\017\017";
	const char *lookup2 = "\010\011\012\013\014\015\016\017         ";

	for (int i = 0; i < FAN_NUM_CHANNELS; i++) {
		adc_target = 16 * target_mV[i] / max_mV[i];
		adc_value = 16 * measured_mV[i] / max_mV[i];
		pwm_value = 16 * measured_PWM[i] / max_PWM;
		rpm_value = 16 * measured_RPM[i] / max_RPM[i];
		snprintf(&line1[4*i], 5, "%c%c%c%c ", lookup1[adc_target], lookup1[adc_value], lookup1[pwm_value], lookup1[rpm_value]);
		snprintf(&line2[4*i], 5, "%c%c%c%c ", lookup2[adc_target], lookup2[adc_value], lookup2[pwm_value], lookup2[rpm_value]);
	}

	LCD_Write(LCD_Line1, 0, line1, 16);
	LCD_Write(LCD_Line2, 0, line2, 16);
	LCD_Update();
}

