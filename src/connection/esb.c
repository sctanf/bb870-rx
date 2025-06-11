#include <zephyr/logging/log.h>
#include "system/system.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/crc.h>

#include "esb.h"

#define MAX_TRACKERS 3

static uint8_t stored_trackers = 0;
static uint64_t stored_tracker_addr[MAX_TRACKERS] = {0};

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);

uint8_t pairing_buf[8] = {0};

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 6, 0, 0);

static int16_t pot_val = 0.4 * 32767;
static int64_t last_data_sent = 0;

int16_t get_val(void)
{
	if (last_data_sent && k_uptime_get() - last_data_sent > 1000)
	{
		pot_val = 0;
	}
	return pot_val;
}

void set_val(int16_t val)
{
	pot_val = val;
}

void set_bat(uint8_t bat)
{
	tx_payload.data[0] = bat;
	tx_payload.data[1] = bat;
}

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (rx_payload.length == 8) {
				memcpy(pairing_buf, rx_payload.data, 8);
				esb_write_payload(&tx_payload_pair); // Add to TX buffer
			} else if (rx_payload.length == 10) {
				if (rx_payload.data[0] != rx_payload.data[2]) break;
				if (rx_payload.data[1] != rx_payload.data[3]) break;
				pot_val = (int16_t)rx_payload.data[0] << 8 | rx_payload.data[1];
				last_data_sent = k_uptime_get();
				esb_write_payload(&tx_payload); // Add to TX buffer
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr)
	{
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0)
	{
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do
	{
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res)
		{
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx)
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		config.bitrate = ESB_BITRATE_1MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}
	else
	{
		// config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		config.bitrate = ESB_BITRATE_1MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = 8;
		// config.retransmit_delay = 600;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_AUTO;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
	}

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	err = esb_init(&config);

	if (!err)
		esb_set_base_address_0(base_addr_0);

	if (!err)
		esb_set_base_address_1(base_addr_1);

	if (!err)
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	if (err)
	{
		LOG_ERR("ESB initialization failed: %d", err);
		return err;
	}

	esb_initialized = true;
	return 0;
}

static void esb_deinitialize(void)
{
	if (esb_initialized)
		esb_disable();
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++)
	{
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++)
		addr_buffer[i + 8] = buf[5] + i;
	for (int i = 0; i < 16; i++)
	{
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55 || addr_buffer[i] == 0xAA) // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static bool esb_pairing = false;
static bool esb_paired = false;

void esb_pair(void)
{
	LOG_INF("Pairing");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is not actually guaranteed, see datasheet)
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	esb_pairing = true;
	while (esb_pairing)
	{
		uint64_t found_addr = (*(uint64_t *)pairing_buf >> 16) & 0xFFFFFFFFFFFF;
		uint16_t send_tracker_id = stored_trackers; // Use new tracker id
		for (int i = 0; i < stored_trackers; i++) // Check if the device is already stored
		{
			if (found_addr != 0 && stored_tracker_addr[i] == found_addr)
			{
				send_tracker_id = i;
			}
		}
		uint8_t checksum = crc8_ccitt(0x07, &pairing_buf[2], 6); // make sure the packet is valid
		if (checksum == 0)
			checksum = 8;
		if (checksum == pairing_buf[0] && found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < MAX_TRACKERS) // New device, add to NVS
		{
			LOG_INF("Added device on id %d with address %012llX", stored_trackers, found_addr);
			stored_tracker_addr[stored_trackers] = found_addr;
			sys_write(STORED_ADDR_0 + stored_trackers, NULL, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
			stored_trackers++;
			sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		}
		if (checksum == pairing_buf[0] && send_tracker_id < MAX_TRACKERS) // Make sure the dongle is not full
			tx_payload_pair.data[0] = pairing_buf[0]; // Use checksum sent from device to make sure packet is for that device
		else
			tx_payload_pair.data[0] = 0; // Invalidate packet
		tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
		k_msleep(10);
	}
	esb_disable();
	esb_receive();
}

void esb_reset_pair(void)
{
	esb_deinitialize(); // make sure esb is off
	esb_paired = false;
}

void esb_finish_pair(void)
{
	esb_pairing = false;
}

void esb_clear(void)
{
	stored_trackers = 0;
	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	LOG_INF("NVS Reset");
	esb_reset_pair();
}

void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	if (stored_trackers)
		esb_paired = true;
	for (int i = 0; i < stored_trackers; i++)
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	LOG_INF("%d/%d devices stored", stored_trackers, MAX_TRACKERS);

	if (esb_paired)
	{
		esb_receive();
		esb_initialize(false);
		esb_start_rx();
	}

	while (1)
	{
		if (!esb_paired)
		{
			esb_pair();
			esb_initialize(false);
			esb_start_rx();
		}
		k_msleep(100);
	}
}
