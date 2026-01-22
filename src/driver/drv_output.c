// Basic Output Driver
// Grabs basic output IORoles, keeps track of pins assigned those roles
// and processes them accordingly.

#include "../obk_config.h"

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "drv_local.h"
#include "drv_uart.h"
#include "../httpserver/new_http.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_adc.h"
#include "../hal/hal_ota.h"
#include "../quicktick.h"

uint32_t g_driverIndex;

uint32_t g_relayPins;
uint32_t g_relayCount;
uint32_t g_wifiPins;

#define WIFI_LED_FAST_BLINK_DURATION 250
#define WIFI_LED_SLOW_BLINK_DURATION 500

static void LED_quickTick() {
	if (!g_wifiPins || !g_enable_pins)
		return;

	uint32_t wifiPins = g_wifiPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(wifiPins, pinIndex))
			continue; // not my pin
		static uint32_t wifiLedToggleTime = 0;
		static bool wifi_ledState = false;

		// WiFi LED
		// In Open Access point mode, fast blink
		if (Main_IsOpenAccessPointMode()) {
			wifiLedToggleTime += QUICK_TMR_DURATION;
			if (wifiLedToggleTime > WIFI_LED_FAST_BLINK_DURATION) {
				wifi_ledState = !wifi_ledState;
				wifiLedToggleTime = 0;
				HAL_PIN_SetOutputValue(pinIndex, wifi_ledState);
			}
	#if ENABLE_MQTT
		} else if (Main_HasMQTTConnected()) {
	#else
		} else if (Main_HasWiFiConnected()) {
	#endif // ENABLE_MQTT 

			// In WiFi client success mode, just stay enabled
			HAL_PIN_SetOutputValue(pinIndex, (PIN_GetPinRoleForPinIndex(pinIndex) == IOR_LED_WIFI_n) ? false : true);
		} else {
			// in connecting mode, slow blink
			wifiLedToggleTime += QUICK_TMR_DURATION;
			if (wifiLedToggleTime > WIFI_LED_SLOW_BLINK_DURATION) {
				wifi_ledState = !wifi_ledState;
				wifiLedToggleTime = 0;
				HAL_PIN_SetOutputValue(pinIndex, wifi_ledState);
			}
		}
		if (!BIT_CLEAR(wifiPins, pinIndex))
			break;
	}
}

// basic output quick tick timer function
void Output_quickTick() {
	if (g_wifiPins)
		LED_quickTick();
}

static bool Output_noOfChannels(int pinIORole) {
	switch (pinIORole) {
	case IOR_LED_WIFI:
	case IOR_LED_WIFI_n:
		return false;
	
	case IOR_LED:
	case IOR_LED_n:
	case IOR_Relay:
	case IOR_Relay_n:
		return true;

	default:
		break;
	}
}

static bool Output_activatePin(int pinIndex) {
	int channelValue = CHANNEL_Get(PIN_GetPinChannelForPinIndex(pinIndex));

	HAL_PIN_Setup_Output(pinIndex);

	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_LED_WIFI_n:
	case IOR_LED_WIFI:
		BIT_SET(g_wifiPins, pinIndex);
		return true;
	
	case IOR_LED_n:
	case IOR_Relay_n:
		channelValue = !channelValue;
	case IOR_LED:
	case IOR_Relay:
		HAL_PIN_SetOutputValue(pinIndex, !channelValue);
		BIT_SET(g_relayPins, pinIndex);
		g_relayCount++;
		return true;
	case IOR_AlwaysHigh:
		HAL_PIN_SetOutputValue(pinIndex, true);
		break;
	case IOR_AlwaysLow:
		HAL_PIN_SetOutputValue(pinIndex, false);
		break;

	default:
		return false;
	}
	return true;
}

static void Output_ReleasePin(int pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_LED_WIFI_n:
	case IOR_LED_WIFI:
		BIT_CLEAR(g_wifiPins, pinIndex);
		break;

	case IOR_LED:
	case IOR_LED_n:
	case IOR_Relay:
	case IOR_Relay_n:
		g_relayCount--;
		BIT_CLEAR(g_relayPins, pinIndex);
		break;
	}
}

static void Output_StopDriver() {
	g_relayCount = 0;
	g_relayPins = 0;
	g_wifiPins = 0;
}

uint32_t Output_relayCount() {
	return g_relayCount;
}

static void Output_init() {
	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}
// framework request function arg can be driver index, pin role or pin index
uint32_t Output_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest)
	{
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_LED] = PIN_pinIORoleDriver()[IOR_LED_n] \
					  = PIN_pinIORoleDriver()[IOR_Relay] = PIN_pinIORoleDriver()[IOR_Relay_n] \
					  = PIN_pinIORoleDriver()[IOR_LED_WIFI] = PIN_pinIORoleDriver()[IOR_LED_WIFI_n] \
					  = PIN_pinIORoleDriver()[IOR_AlwaysHigh] = PIN_pinIORoleDriver()[IOR_AlwaysLow] \
					  = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return Output_activatePin(arg);

	case OBKF_ReleasePin:
		Output_ReleasePin(arg);
		break;

	case OBKF_Stop:
		Output_StopDriver();
		break;
		
	case OBKF_ShouldPublish:
	case OBKF_NoOfChannels:
		return Output_noOfChannels(arg);

	case OBKF_Init:
		Output_init();
		break;

	default:
		break;
	}

	return true;
}

void Output_onChanged(uint32_t channel, uint32_t iVal) {
	if (!g_relayPins || !g_enable_pins)
		return;

	uint32_t relayPins = g_relayPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(relayPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channel)
			continue; // channel not for pin

		bool channelValue = (iVal > 0);
		switch (pinIndex) {
		case IOR_LED_n:
		case IOR_Relay_n:
			channelValue = !channelValue;
		case IOR_LED:
		case IOR_Relay:
			HAL_PIN_SetOutputValue(pinIndex, !channelValue);
		}

		if (!BIT_CLEAR(relayPins, pinIndex))
			break;
	}
}

bool Output_isPowerRelay(uint32_t channel) {
	uint32_t relayPins = g_relayPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(relayPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channel)
			continue; // channel not for pin

		switch (pinIndex) {
		case IOR_Relay_n:
		case IOR_Relay:
			return true;
		}
		if (!BIT_CLEAR(relayPins, pinIndex))
			break;
	}
	return false;
}

bool Output_isRelay(uint32_t channel) {
	uint32_t relayPins = g_relayPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(relayPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channel)
			continue; // channel not for pin

		return true;
	}
	return false;
}