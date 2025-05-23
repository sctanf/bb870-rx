#include <zephyr/kernel.h>

#include "system.h"
#include "connection/esb.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/device.h>

#include "power.h"

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

static uint32_t *dbl_reset_mem __attribute__((unused)) = ((uint32_t *)DFU_DBL_RESET_MEM); // retained

static void disable_DFU_thread(void);
K_THREAD_DEFINE(disable_DFU_thread_id, 128, disable_DFU_thread, NULL, NULL, NULL, 6, 0, 500); // disable DFU if the system is running correctly

void sys_request_system_off(void)
{
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	sys_poweroff();
}

void sys_request_system_reboot(void)
{
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
	sys_reboot(SYS_REBOOT_COLD);
}

static void disable_DFU_thread(void)
{
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
}
