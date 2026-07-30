#include "DShotRMT.h"
#include "freertos/task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

static const char *TAG = "dshot-rmt";

#define DSHOT_ERROR_CHECK(x) ({        \
	esp_err_t __ret = x;               \
	if (__ret != ESP_OK)               \
		return __ret;                  \
	__ret;                             \
})

// from https://github.com/bitdump/BLHeli/blob/master/BLHeli_32%20ARM/BLHeli_32%20Firmware%20specs/Digital_Cmd_Spec.txt
enum dshot_cmd_t
{
	DIGITAL_CMD_MOTOR_STOP, 		      // Currently not implemented
	DIGITAL_CMD_BEEP1, 			      // Wait at least length of beep (260ms) before next command
	DIGITAL_CMD_BEEP2, 			      // Wait at least length of beep (260ms) before next command
	DIGITAL_CMD_BEEP3, 			      // Wait at least length of beep (280ms) before next command
	DIGITAL_CMD_BEEP4, 			      // Wait at least length of beep (280ms) before next command
	DIGITAL_CMD_BEEP5, 			      // Wait at least length of beep (1020ms) before next command
	DIGITAL_CMD_ESC_INFO,  		      // Wait at least 12ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_1, 	      // Need 6x, no wait required
	DIGITAL_CMD_SPIN_DIRECTION_2, 	      // Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_OFF, 		      // Need 6x, no wait required
	DIGITAL_CMD_3D_MODE_ON,  		      // Need 6x, no wait required
	DIGITAL_CMD_SETTINGS_REQUEST,  	      // Currently not implemented
	DIGITAL_CMD_SAVE_SETTINGS,  		      // Need 6x, wait at least 35ms before next command
	DIGITAL_CMD_SPIN_DIRECTION_NORMAL = 20, 	      // Need 6x, no wait required
	DIGITAL_CMD_SPIN_DIRECTION_REVERSED, 	      // Need 6x, no wait required
	DIGITAL_CMD_LED0_ON, 			      // No wait required
	DIGITAL_CMD_LED1_ON, 			      // No wait required
	DIGITAL_CMD_LED2_ON, 			      // No wait required
	DIGITAL_CMD_LED3_ON, 			      // No wait required
	DIGITAL_CMD_LED0_OFF, 		      // No wait required
	DIGITAL_CMD_LED1_OFF, 		      // No wait required
	DIGITAL_CMD_LED2_OFF, 		      // No wait required
	DIGITAL_CMD_LED3_OFF, 		      // No wait required
};

// DSHOT Timings
#define DSHOT_TICKS_PER_BIT 19

#define DSHOT_T0H 7
#define DSHOT_T0L (DSHOT_TICKS_PER_BIT - DSHOT_T0H)

#define DSHOT_T1H 14
#define DSHOT_T1L (DSHOT_TICKS_PER_BIT - DSHOT_T1H)

#define DSHOT_PAUSE (DSHOT_TICKS_PER_BIT * 200)
// !DSHOT Timings

#define RMT_CMD_SIZE (sizeof(_dshotCmd) / sizeof(_dshotCmd[0]))

#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

#define DSHOT_ARM_DELAY (5000 / portTICK_PERIOD_MS)

DShotRMT::DShotRMT()
{
	// initialize cmd buffer
	setData(0);

	// DShot packet delay + RMT end marker
	_dshotCmd[16].duration0 = DSHOT_PAUSE;
	_dshotCmd[16].level0 = 0;
	_dshotCmd[16].duration1 = 0;
	_dshotCmd[16].level1 = 0;
}

DShotRMT::~DShotRMT()
{
	// TODO write destructor
}

esp_err_t DShotRMT::install(gpio_num_t gpio, rmt_channel_t rmtChannel)
{
	_rmtChannel = rmtChannel;

	// Fully initialize the config. If the flags field contains an accidental
	// RMT_CHANNEL_FLAGS_AWARE_DFS bit, the legacy driver uses the 1 MHz
	// reference clock and makes the DShot waveform roughly 80x too slow.
	rmt_config_t config = RMT_DEFAULT_CONFIG_TX(gpio, rmtChannel);
	config.clk_div = 7;
	config.flags = 0;

	DSHOT_ERROR_CHECK(rmt_config(&config));

	uint32_t counterClockHz = 0;
	DSHOT_ERROR_CHECK(rmt_get_counter_clock(rmtChannel, &counterClockHz));
	ESP_LOGI(TAG, "RMT channel %d counter clock: %lu Hz",
			 static_cast<int>(rmtChannel),
			 static_cast<unsigned long>(counterClockHz));

	// DShot600 uses an RMT counter near 80 MHz / 7 = 11.43 MHz.
	// Refuse to send malformed motor commands if the wrong clock is active.
	if (counterClockHz < 11000000U || counterClockHz > 12000000U)
	{
		ESP_LOGE(TAG,
				 "RMT channel %d counter clock is invalid for DShot600",
				 static_cast<int>(rmtChannel));
		return ESP_ERR_INVALID_STATE;
	}

	return rmt_driver_install(rmtChannel, 0, 0);
}

esp_err_t DShotRMT::uninstall()
{
	// TODO implement uninstall
	return ESP_OK;
}

esp_err_t DShotRMT::init(bool wait)
{
	ESP_LOGD(TAG, "Sending reset command");
	for (int i = 0; i < 50; i++)
	{
		writeData(0, true);
	}

	ESP_LOGD(TAG, "Holding zero throttle for arming delay");
	if (wait)
		DSHOT_ERROR_CHECK(repeatPacketTicks({0, 0}, DSHOT_ARM_DELAY));
	else
		DSHOT_ERROR_CHECK(writePacket({0, 0}, false));

	ESP_LOGD(TAG, "ESC armed");
	return ESP_OK;
}

esp_err_t DShotRMT::sendThrottle(uint16_t throttle)
{
	if (throttle == 0)
		return writePacket({0, 0}, false);

	if (throttle < DSHOT_THROTTLE_MIN || throttle > DSHOT_THROTTLE_MAX)
		return ESP_ERR_INVALID_ARG;

	return writePacket({throttle, 0}, false);
}

esp_err_t DShotRMT::sendDirectionCommand(bool reversed)
{
	// BLHeli_S and BLHeli_32 require direction commands to be sent with the
	// telemetry bit set while the motor is stopped. The caller must send the
	// same command at least six times; ten repetitions provide margin.
	return writePacket(
			{reversed ? DIGITAL_CMD_SPIN_DIRECTION_REVERSED
					  : DIGITAL_CMD_SPIN_DIRECTION_NORMAL,
			 1},
			false);
}

esp_err_t DShotRMT::setReversed(bool reversed)
{
	// Commands are accepted only at zero throttle. Command 21 temporarily
	// reverses the saved ESC direction; command 20 restores the saved direction.
	DSHOT_ERROR_CHECK(
			repeatPacketTicks({0, 0}, pdMS_TO_TICKS(300)));
	for (int i = 0; i < 10; i++)
	{
		DSHOT_ERROR_CHECK(sendDirectionCommand(reversed));
		vTaskDelay(1);
	}
	DSHOT_ERROR_CHECK(
			repeatPacketTicks({0, 0}, pdMS_TO_TICKS(500)));
	return ESP_OK;
}

esp_err_t DShotRMT::beep()
{
	DSHOT_ERROR_CHECK(writePacket({DIGITAL_CMD_BEEP1, 1}, true));
	vTaskDelay(260 / portTICK_PERIOD_MS);
	return ESP_OK;
}

void DShotRMT::setData(uint16_t data)
{
	for (int i = 0; i < 16; i++, data <<= 1)
	{
		if (data & 0x8000)
		{
			// set one
			_dshotCmd[i].duration0 = DSHOT_T1H;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = DSHOT_T1L;
			_dshotCmd[i].level1 = 0;
		}
		else
		{
			// set zero
			_dshotCmd[i].duration0 = DSHOT_T0H;
			_dshotCmd[i].level0 = 1;
			_dshotCmd[i].duration1 = DSHOT_T0L;
			_dshotCmd[i].level1 = 0;
		}
	}
}

uint8_t DShotRMT::checksum(uint16_t data)
{
	uint16_t csum = 0;

	for (int i = 0; i < 3; i++)
	{
		csum ^= data;
		data >>= 4;
	}

	return csum & 0xf;
}

esp_err_t DShotRMT::writeData(uint16_t data, bool wait)
{
    TickType_t tx_wait_ticks = pdMS_TO_TICKS(100);
    DSHOT_ERROR_CHECK(rmt_wait_tx_done(_rmtChannel, tx_wait_ticks));

	setData(data);

	return rmt_write_items(_rmtChannel,
						   _dshotCmd, RMT_CMD_SIZE,
						   wait);
}

esp_err_t DShotRMT::writePacket(dshot_packet_t packet, bool wait)
{
	uint16_t data = packet.payload;

	data <<= 1;
	data |= packet.telemetry;

	data = (data << 4) | checksum(data);

	return writeData(data, wait);
}

esp_err_t DShotRMT::repeatPacket(dshot_packet_t packet, int n)
{
	for (int i = 0; i < n; i++)
	{
		DSHOT_ERROR_CHECK(writePacket(packet, true));
		portYIELD();
	}
	return ESP_OK;
}

esp_err_t DShotRMT::repeatPacketTicks(dshot_packet_t packet, TickType_t ticks)
{
	DSHOT_ERROR_CHECK(rmt_wait_tx_done(_rmtChannel, ticks));

	TickType_t repeatStop = xTaskGetTickCount() + ticks;
	while (xTaskGetTickCount() < repeatStop)
	{
		DSHOT_ERROR_CHECK(writePacket(packet, false));
		vTaskDelay(1);
	}
	return ESP_OK;
}
