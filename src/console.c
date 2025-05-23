#include <zephyr/logging/log.h>
#include "system/system.h"

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>
#include <zephyr/console/console.h>
#include <zephyr/logging/log_ctrl.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log_ctrl.h>
#include "connection/esb.h"

#include <ctype.h>

#define DFU_DBL_RESET_MEM 0x20007F7C
#define DFU_DBL_RESET_APP 0x4ee5677e

uint32_t* dbl_reset_mem = ((uint32_t*) DFU_DBL_RESET_MEM);

LOG_MODULE_REGISTER(console, LOG_LEVEL_INF);

static void usb_init_thread(void);
K_THREAD_DEFINE(usb_init_thread_id, 256, usb_init_thread, NULL, NULL, NULL, 6, 0, 0);

static void console_thread(void);
static struct k_thread console_thread_id;
static K_THREAD_STACK_DEFINE(console_thread_id_stack, 512);

static const char *meows[] = {
	"Mew",
	"Meww",
	"Meow",
	"Meow meow",
	"Mrrrp",
	"Mrrf",
	"Mreow",
	"Mrrrow",
	"Mrrr",
	"Purr",
	"mew",
	"meww",
	"meow",
	"meow meow",
	"mrrrp",
	"mrrf",
	"mreow",
	"mrrrow",
	"mrrr",
	"purr",
};

static const char *meow_punctuations[] = {
	".",
	"?",
	"!",
	"-",
	"~",
	""
};

static const char *meow_suffixes[] = {
	" :3",
	" :3c",
	" ;3",
	" ;3c",
	" x3",
	" x3c",
	" X3",
	" X3c",
	" >:3",
	" >:3c",
	" >;3",
	" >;3c",
	""
};

static void skip_dfu(void)
{
	(*dbl_reset_mem) = DFU_DBL_RESET_APP; // Skip DFU
}

static void console_thread_create(void)
{
	k_thread_create(&console_thread_id, console_thread_id_stack, K_THREAD_STACK_SIZEOF(console_thread_id_stack), (k_thread_entry_t)console_thread, NULL, NULL, NULL, 6, 0, K_NO_WAIT);
}

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	switch (status)
	{
	case USB_DC_CONNECTED:
		console_thread_create();
		break;
	case USB_DC_DISCONNECTED:
		k_thread_abort(&console_thread_id);
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static void usb_init_thread(void)
{
	usb_enable(status_cb);
}

static void print_meow(void)
{
	int64_t ticks = k_uptime_ticks();

	ticks %= ARRAY_SIZE(meows) * ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes); // silly number generator
	uint8_t meow = ticks / (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	ticks %= (ARRAY_SIZE(meow_punctuations) * ARRAY_SIZE(meow_suffixes));
	uint8_t punctuation = ticks / ARRAY_SIZE(meow_suffixes);
	uint8_t suffix = ticks % ARRAY_SIZE(meow_suffixes);

	printk("%s%s%s\n", meows[meow], meow_punctuations[punctuation], meow_suffixes[suffix]);
}

static void console_thread(void)
{
	console_getline_init();
	while (log_data_pending())
		k_usleep(1);
	k_msleep(100);

	printk("reboot                       Soft reset the device\n");
	printk("pair                         Enter pairing mode\n");
	printk("exit                         Exit pairing mode\n");
	printk("clear                        Clear stored devices\n");
	printk("dfu                          Enter DFU bootloader\n");
	printk("meow                         Meow!\n");

	uint8_t command_reboot[] = "reboot";
	uint8_t command_pair[] = "pair";
	uint8_t command_exit[] = "exit";
	uint8_t command_clear[] = "clear";
	uint8_t command_dfu[] = "dfu";
	uint8_t command_meow[] = "meow";

	while (1) {
		uint8_t *line = console_getline();
		for (uint8_t *p = line; *p; ++p) {
			*p = tolower(*p);
		}

		if (memcmp(line, command_reboot, sizeof(command_reboot)) == 0)
		{
			skip_dfu();
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_pair, sizeof(command_pair)) == 0)
		{
			esb_reset_pair();
		}
		else if (memcmp(line, command_exit, sizeof(command_exit)) == 0)
		{
			esb_finish_pair();
		}
		else if (memcmp(line, command_clear, sizeof(command_clear)) == 0) 
		{
			esb_clear();
		}
		else if (memcmp(line, command_dfu, sizeof(command_dfu)) == 0)
		{
			NRF_POWER->GPREGRET = 0x57;
			sys_reboot(SYS_REBOOT_COLD);
		}
		else if (memcmp(line, command_meow, sizeof(command_meow)) == 0) 
		{
			print_meow();
		}
		else
		{
			printk("Unknown command\n");
		}
	}
}
