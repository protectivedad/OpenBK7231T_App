// Basic Output Driver
// Grabs basic output IORoles, keeps track of pins assigned those roles
// and processes them accordingly. A maximum of 32 LEDs, 32 Relays one WIFI_LEDs

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
uint32_t g_wifiLEDIndex;
uint32_t g_led;

#define WIFI_LED_FAST_BLINK_DURATION 250
#define WIFI_LED_SLOW_BLINK_DURATION 500

static void LED_QuickTick() {
	static uint32_t wifiLedToggleTime = 0;
	static bool wifi_ledState = false;

	if (!g_enable_pins)
		return;

	// WiFi LED
	// In Open Access point mode, fast blink
	if (Main_IsOpenAccessPointMode()) {
		wifiLedToggleTime += QUICK_TMR_DURATION;
		if (wifiLedToggleTime > WIFI_LED_FAST_BLINK_DURATION) {
			wifi_ledState = !wifi_ledState;
			wifiLedToggleTime = 0;
			HAL_PIN_SetOutputValue(g_wifiLEDIndex, wifi_ledState);
		}
#if ENABLE_MQTT
	} else if (Main_HasMQTTConnected()) {
#else
	} else if (Main_HasWiFiConnected()) {
#endif // ENABLE_MQTT 

		// In WiFi client success mode, just stay enabled
		HAL_PIN_SetOutputValue(g_wifiLEDIndex, (PIN_GetPinRoleForPinIndex(g_wifiLEDIndex) == IOR_LED_WIFI_n) ? false : true);
	} else {
		// in connecting mode, slow blink
		wifiLedToggleTime += QUICK_TMR_DURATION;
		if (wifiLedToggleTime > WIFI_LED_SLOW_BLINK_DURATION) {
			wifi_ledState = !wifi_ledState;
			wifiLedToggleTime = 0;
			HAL_PIN_SetOutputValue(g_wifiLEDIndex, wifi_ledState);
		}
	}
}

// basic output quick tick timer function
void Output_QuickTick() {
	if (g_wifiLEDIndex)
		LED_QuickTick();
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

static void Output_ChannelChange() {

}

// TODO: add arg so pinIORole can be passed with pinIndex
static bool Output_ActivatePin(int pinIndex) {
	int channelValue = CHANNEL_Get(PIN_GetPinChannelForPinIndex(pinIndex));

	HAL_PIN_Setup_Output(pinIndex);

	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_LED_WIFI_n:
	case IOR_LED_WIFI:
		g_wifiLEDIndex = pinIndex;
		return true;
	
	case IOR_LED_n:
	case IOR_Relay_n:
		channelValue = !channelValue;
	case IOR_LED:
	case IOR_Relay:
		HAL_PIN_SetOutputValue(pinIndex, !channelValue);
		return true;

	default:
		break;
	}
	return true;
}

static void Output_ReleasePin(int pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_LED_WIFI_n:
	case IOR_LED_WIFI:
		g_wifiLEDIndex = 0;
		return;
	}
}

static void Output_StopDriver() {
	g_wifiLEDIndex = 0;
}

// framework request function
int Output_frameworkRequest(int obkfRequest, int arg) {
	switch (obkfRequest)
	{
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_LED] \
		              = PIN_pinIORoleDriver()[IOR_LED_n] \
					  = PIN_pinIORoleDriver()[IOR_Relay] \
					  = PIN_pinIORoleDriver()[IOR_Relay_n] \
					  = PIN_pinIORoleDriver()[IOR_LED_WIFI] \
					  = PIN_pinIORoleDriver()[IOR_LED_WIFI_n] \
					  = arg;
		break;
	
	case OBKF_AcquirePin:
		return Output_ActivatePin(arg);

	case OBKF_ReleasePin:
		Output_ReleasePin(arg);
		break;

	case OBKF_Stop:
		Output_StopDriver();
		break;
		
	case OBKF_ShouldPublish:
	case OBKF_NoOfChannels:
		return Output_noOfChannels(arg);

	// case OBKF_ChannelChange:

	case OBKF_Init:
	default:
		break;
	}

	return true;
}
