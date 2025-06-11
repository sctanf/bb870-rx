#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <hal/nrf_gpio.h>

#include "system/power.h"
#include "system/battery.h"
#include "connection/esb.h"

#include <math.h>

int battery_mV = 0;
int16_t battery_pptt = 0;

static int32_t psu_mV = 0;

static const struct adc_dt_spec psu_adc_channel = ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

#define PWM_PERIOD 100

static bool motor_state = false;
static float motor_pwm = 0.0f;
static int64_t last_pwm_time = 0;

static const struct pwm_dt_spec motor_pwm_channel = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));

static bool charger_state = false;
static bool motor_source = false; // 0 = battery, 1 = psu
static bool motor_state_actual = false;

static const struct gpio_dt_spec battery_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), bat_enable_gpios);
static const struct gpio_dt_spec psu_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), psu_enable_gpios);
static const struct gpio_dt_spec charger_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), chg_enable_gpios);
static const struct gpio_dt_spec motor_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), motor_enable_gpios);
static const struct gpio_dt_spec sys_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), sys_enable_gpios);

static bool battery_enabled = false; 
static bool psu_enabled = false; 
static bool charger_enabled = false; 

static void battery_thread(void)
{
	while (1)
	{
		battery_pptt = read_batt_mV(&battery_mV);
		set_bat(battery_pptt / 100);
		k_msleep(100);
	}
}

K_THREAD_DEFINE(battery_thread_id, 512, battery_thread, NULL, NULL, NULL, 6, 0, 0);

static void psu_thread(void)
{
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	adc_channel_setup_dt(&psu_adc_channel);

	while (1) {
		int32_t val_mv;

		adc_sequence_init_dt(&psu_adc_channel, &sequence);

		adc_read_dt(&psu_adc_channel, &sequence);

		val_mv = (int32_t)buf;
		adc_raw_to_millivolts_dt(&psu_adc_channel, &val_mv);
		psu_mV = val_mv * (uint64_t)2420000 / 220000;

		if (motor_state_actual == true) {
			k_usleep(1);
		} else {
			k_msleep(100);
		}
	}
}

K_THREAD_DEFINE(psu_thread_id, 512, psu_thread, NULL, NULL, NULL, 6, 0, 0);

/*
off (0) 0.0
10.5
lowest (11) 0.125
21.5
a (22) 0.25
32.5
b (33) 0.375
43.5
c (44) 0.5
53.5?
d (55) 0.625
64.5
e (66) 0.75
75.5
f (77) 0.875
86
highest (88) 1.0
*/

/*
min sent by rx: 0.1
max sent by rx: 1.0
off: 0.00
min: 0.11
max: 0.88
step: 0.11
steps: 9, i guess
*/

static void motor_thread(void)
{
	gpio_pin_configure_dt(&sys_enable, GPIO_INPUT);
	float last_pwm = 0;
	while (1) {
		float duty_cycle = get_val() / 32767.0f;
		if (duty_cycle > 0.1f) {
			duty_cycle = (duty_cycle - 0.1f) / 0.9f * 0.89f + 0.11f;
		}
		if (motor_source && duty_cycle > 0.6f) {
			duty_cycle = 0.6f; // limit to 60% on PSU
		}
//		duty_cycle = (duty_cycle < 0.1f && duty_cycle > 0) ? 0.1f : duty_cycle;
//		duty_cycle = duty_cycle < 0 ? 0 : duty_cycle;
//		duty_cycle = duty_cycle > 1 ? 1 : duty_cycle;
//		if (duty_cycle > 0.1f) {
//			duty_cycle = (duty_cycle - 0.1f) / 0.9f * 0.675f + 0.125f; // limit to 80%
//		}
//		motor_pwm = duty_cycle;
//		int step = duty_cycle / 0.125f;
//		float step_pwm = (duty_cycle - step * 0.125f) / 0.125f;
//		int sub_pwm = step_pwm * PWM_PERIOD;
//		duty_cycle = step * 0.125f;
//		if (k_uptime_get() % PWM_PERIOD < sub_pwm) {
//			duty_cycle += 0.125f;
//		}
		motor_pwm = duty_cycle;
		if (duty_cycle != 0 || duty_cycle != last_pwm) {
			if (duty_cycle == 0 && k_uptime_get() - last_pwm_time < 500) {
				duty_cycle = 0.1f;
			} else {
				last_pwm_time = k_uptime_get();
			}
			last_pwm = duty_cycle;
			if (motor_state == false) {
				printk("Motor starting\n");
				motor_state = true;
			}
			pwm_set_pulse_dt(&motor_pwm_channel, motor_state_actual ? motor_pwm_channel.period * duty_cycle : 0);
		} else if (motor_state == true && motor_pwm == 0 && k_uptime_get() - last_pwm_time > 5000) {
			motor_state = false;
			printk("Motor stopping\n");
		}
		if (motor_state_actual == true) {
			k_usleep(100);
		} else {
			k_msleep(100);
		}
	}
}

K_THREAD_DEFINE(motor_thread_id, 512, motor_thread, NULL, NULL, NULL, 6, 0, 0);

static void power_thread(void)
{
	gpio_pin_configure_dt(&battery_enable, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&psu_enable, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&charger_enable, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&motor_enable, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&sys_enable, GPIO_INPUT);
	while (1) {
		if (psu_mV > 24000 && charger_state == false) {
			printk("Enabling charger\n");
			charger_enabled = true;
			gpio_pin_set_dt(&charger_enable, 1);
			charger_state = true;
		} else if (psu_mV < 23000 && charger_state == true) {
			printk("Disabling charger\n");
			charger_enabled = false;
			gpio_pin_set_dt(&charger_enable, 0);
			charger_state = false;
		}
		if (gpio_pin_get_dt(&sys_enable) == 0 || (battery_mV && (battery_mV < 3000 * 7))) {
			if (gpio_pin_get_dt(&sys_enable) == 0) {
				nrf_gpio_cfg_sense_input(NRF_PIN_PORT_TO_PIN_NUMBER(sys_enable.pin, 1), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
			} else {
				nrf_gpio_cfg_sense_input(NRF_PIN_PORT_TO_PIN_NUMBER(sys_enable.pin, 1), NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);
			}
			printk("System off\n");
			set_val(0);
			battery_enabled = false;
			psu_enabled = false;
			gpio_pin_set_dt(&motor_enable, 0);
			gpio_pin_set_dt(&battery_enable, 0);
			gpio_pin_set_dt(&psu_enable, 0);
			motor_state_actual = false;
			while (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk)
			{
				set_val(0);
				if (gpio_pin_get_dt(&sys_enable) == 1 && (!battery_mV || (battery_mV >= 3000 * 7))) {
					break;
				}
				if (psu_mV > 24000 && charger_state == false) {
					printk("Enabling charger\n");
					charger_enabled = true;
					gpio_pin_set_dt(&charger_enable, 1);
					charger_state = true;
				} else if (psu_mV < 23000 && charger_state == true) {
					printk("Disabling charger\n");
					charger_enabled = false;
					gpio_pin_set_dt(&charger_enable, 0);
					charger_state = false;
				}
				k_msleep(100);
			}
			if (gpio_pin_get_dt(&sys_enable) == 1 && (!battery_mV || (battery_mV >= 3000 * 7))) {
				set_val(0);
				printk("Cancel system off\n");
				continue;
			}
//			k_msleep(1000); // grace period
			nrf_gpio_cfg_input(31, NRF_GPIO_PIN_NOPULL);
			if (gpio_pin_get(psu_enable.port, 31)) {
				printk("PSU on\n");
				gpio_pin_set_dt(&charger_enable, 1);
				nrf_gpio_cfg_sense_input(31, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_LOW);
			} else {
				printk("PSU off\n");
				gpio_pin_set_dt(&charger_enable, 0);
				nrf_gpio_cfg_sense_input(31, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_SENSE_HIGH);
			}
			sys_request_system_off();
		}
		if (psu_mV > 24000 && motor_source == 0) {
			if (motor_state_actual == true) {
				printk("Switching to PSU\n");
				battery_enabled = false;
				gpio_pin_set_dt(&battery_enable, 0);
				k_busy_wait(50);
				gpio_pin_set_dt(&psu_enable, 1);
				psu_enabled = true;
				printk("Done\n");
			} else {
				printk("Source set to PSU\n");
			}
			motor_source = 1;
		}
		if (psu_mV < 23000 && motor_source == 1) {
			if (motor_state_actual == true) {
				printk("Switching to Battery\n");
				psu_enabled = false;
				gpio_pin_set_dt(&psu_enable, 0);
				k_busy_wait(50);
				gpio_pin_set_dt(&battery_enable, 1);
				battery_enabled = true;
				printk("Done\n");
			} else {
				printk("Source set to Battery\n");
			}
			motor_source = 0;
		}
		if (motor_state) {
			if (motor_state_actual == false) {
				battery_enabled = false;
				psu_enabled = false;
				gpio_pin_set_dt(&battery_enable, 0);
				gpio_pin_set_dt(&psu_enable, 0);
				k_busy_wait(50);
				if (motor_source == 0) {
					battery_enabled = true;
					gpio_pin_set_dt(&battery_enable, 1);
					printk("Enabling battery\n");
				} else {
					psu_enabled = true;
					gpio_pin_set_dt(&psu_enable, 1);
					printk("Enabling PSU\n");
				}
				gpio_pin_set_dt(&motor_enable, 1);
				motor_state_actual = true;
			}
			gpio_pin_set_dt(&motor_enable, 1);
		} else {
			battery_enabled = false;
			psu_enabled = false;
			gpio_pin_set_dt(&motor_enable, 0);
			gpio_pin_set_dt(&battery_enable, 0);
			gpio_pin_set_dt(&psu_enable, 0);
			motor_state_actual = false;
		}
		if (motor_state_actual == true) {
			k_usleep(1);
		} else {
			k_msleep(100);
		}
	}
}

K_THREAD_DEFINE(power_thread_id, 512, power_thread, NULL, NULL, NULL, 6, 0, 0);

int main(void)
{
	while (1) {
		printk("PSU %5dmV, Bat %6.2f%% (%5dmV), PWM %6.2f%%, Mtr %d (%d), Src %d, Chg %d, B %d, P %d, C %d\n", psu_mV, battery_pptt / 100.0, battery_mV, (double)motor_pwm * 100, motor_state, motor_state_actual, motor_source, charger_state, battery_enabled, psu_enabled, charger_enabled);
		k_msleep(500);
	}
	return 0;
}
