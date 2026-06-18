#ifndef COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_
#define COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_

//Migrated to the ESP-IDF v5.x/v6.x RMT driver. The legacy RMT driver
//(driver/rmt.h, rmt_config()/rmt_write_items()) was removed in ESP-IDF v6.0.
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

class DShotRMT
{
public:
	DShotRMT();
	~DShotRMT();

	// NOTE: the new RMT driver allocates channels dynamically, so an explicit
	// RMT channel number is no longer required (or accepted).
	esp_err_t install(gpio_num_t gpio);
	esp_err_t uninstall();

	esp_err_t init(bool wait = true);
	esp_err_t sendThrottle(uint16_t throttle);
	esp_err_t setReversed(bool reversed);
	esp_err_t beep();

private:
	struct dshot_packet_t
	{
		uint16_t payload;
		bool telemetry;
	};

	void setData(uint16_t data);
	static uint8_t checksum(uint16_t data);
	esp_err_t writeData(uint16_t data, bool wait);
	esp_err_t writePacket(dshot_packet_t packet, bool wait);
	esp_err_t repeatPacket(dshot_packet_t packet, int n);
	esp_err_t repeatPacketTicks(dshot_packet_t packet, TickType_t ticks);

	// 16 data bits + 1 inter-frame pause / end marker
	rmt_symbol_word_t _dshotCmd[17];
	rmt_channel_handle_t _rmtChannel = nullptr;
	rmt_encoder_handle_t _encoder = nullptr;
};

#endif /* COMPONENTS_DSHOT_RMT_INCLUDE_DSHOTRMT_H_ */
