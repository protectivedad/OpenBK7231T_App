// Basic Digital Input Driver

#include "../obk_config.h"

#include "../new_common.h"
#if ENABLE_DRIVER_DIGITAL

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

uint32_t g_driverPins;
uint32_t g_lastValidState;
uint32_t g_digitalCount;
uint32_t g_dynamicWakeEdge = 0xFFFFFFFF;
uint32_t g_defaultWakeEdge = 0x00000000;

#if ENABLE_DEEPSLEEP
void Digital_setWakeUpEdge(uint32_t pinIndex, uint32_t edgeCode) {
	if (edgeCode == 2) {
		BIT_CLEAR(g_defaultWakeEdge, pinIndex);
		BIT_SET(g_dynamicWakeEdge, pinIndex);
	} else {
		BIT_CLEAR(g_dynamicWakeEdge, pinIndex);
		BIT_SET_TO(g_defaultWakeEdge, pinIndex, edgeCode);
	}
}
void Digital_setAllWakeUpEdges(uint32_t edgeCode) {
	g_dynamicWakeEdge = g_defaultWakeEdge = 0x00000000;
	if (edgeCode == 0)
		return;
	if (edgeCode == 2)
		g_dynamicWakeEdge = 0xFFFFFFFF;
	else if (edgeCode == 1)
		g_defaultWakeEdge = 0xFFFFFFFF;
}

void Digital_setEdges() {
	if (!g_driverPins)
		return;

	uint32_t driverPins = g_driverPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		uint32_t pinRole = PIN_GetPinRoleForPinIndex(pinIndex);
		bool falling = false;
		switch (pinRole) {
		case IOR_DigitalInput:
		case IOR_DigitalInput_n:
		case IOR_DigitalInput_NoPup:
		case IOR_DigitalInput_NoPup_n:
			if (BIT_CHECK(g_dynamicWakeEdge, pinIndex))
				falling = HAL_PIN_ReadDigitalInput(pinIndex);
			else
				falling = BIT_CHECK(g_defaultWakeEdge, pinIndex);
			break;
		}
// #if PLATFORM_XRADIO
// 			int pull = 1;
// 			if(
// #if ENABLE_DRIVER_DOORSENSOR
// 				g_cfg.pins.roles[i] == IOR_DoorSensor_NoPup ||
// #endif
// 				g_cfg.pins.roles[i] == IOR_DigitalInput_NoPup
// 				|| g_cfg.pins.roles[i] == IOR_DigitalInput_NoPup_n)
// 			{
// 				pull = 0;
// 			}
// #if ENABLE_DRIVER_DOORSENSOR
// 			else if(g_cfg.pins.roles[i] == IOR_DoorSensor_pd)
// 			{
// 				pull = 2;
// 			}
// #endif
// 			SetWUPIO(i, pull, falling);
// #else
		PIN_setGPIActive(pinIndex, 1, falling);
// #endif
		if (!BIT_CLEAR(driverPins, pinIndex))
			break;
	}
}

commandResult_t CMD_Digital_setEdge(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	// strlen("DSEdge") == 6
	if (Tokenizer_GetArgsCount() > 1) // Digital_setEdge [Edge] [Pin]
		Digital_setWakeUpEdge(Tokenizer_GetArgInteger(1),Tokenizer_GetArgInteger(0));
	else // Digital_setEdge [Edge]
		Digital_setAllWakeUpEdges(Tokenizer_GetArgInteger(0));

	return CMD_RES_OK;
}
#endif

// reads, debounces and set channels for digital input pins
void Digital_quickTick(uint32_t timeSinceLast) {
	if (!g_driverPins)
		return;

	uint32_t driverPins = g_driverPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		static uint32_t g_times[PLATFORM_GPIO_MAX];
		static uint32_t g_times2[PLATFORM_GPIO_MAX];

		uint32_t debounceMS = CFG_HasFlag(OBK_FLAG_BTN_INSTANTTOUCH) ? 100 : 250;
		uint32_t pinRole = PIN_GetPinRoleForPinIndex(pinIndex);
		bool pinValue = HAL_PIN_ReadDigitalInput(pinIndex);
		switch (pinRole) {
		case IOR_DigitalInput_n:
		case IOR_DigitalInput_NoPup_n:
			pinValue = !pinValue;
		case IOR_DigitalInput:
		case IOR_DigitalInput_NoPup:
			// debouncing
			if (pinValue) {
				if (g_times[pinIndex] > debounceMS) {
					if (!BIT_CHECK(g_lastValidState,pinIndex)) {
						BIT_TGL(g_lastValidState, pinIndex);
						CHANNEL_Set(g_cfg.pins.channels[pinIndex], pinValue, 0);
					}
				}
				else {
					g_times[pinIndex] += timeSinceLast;
				}
				g_times2[pinIndex] = 0;
			} else {
				if (g_times2[pinIndex] > debounceMS) {
					if (BIT_CHECK(g_lastValidState,pinIndex)) {
						BIT_TGL(g_lastValidState, pinIndex);
						CHANNEL_Set(g_cfg.pins.channels[pinIndex], pinValue, 0);
					}
				}
				else {
					g_times2[pinIndex] += timeSinceLast;
				}
				g_times[pinIndex] = 0;
			}
			break;

		case IOR_ToggleChannelOnToggle:
			if (pinValue) {
				if (g_times[pinIndex] > debounceMS) {
					if (!BIT_CHECK(g_lastValidState,pinIndex)) {
						BIT_TGL(g_lastValidState, pinIndex);
						if (!CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
							CHANNEL_Toggle(g_cfg.pins.channels[pinIndex]);
							EventHandlers_FireEvent(CMD_EVENT_PIN_ONTOGGLE, pinIndex);
						} else {
							ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
						}
					}
				} else {
					g_times[pinIndex] += timeSinceLast;
				}
				g_times2[pinIndex] = 0;
			} else {
				if (g_times2[pinIndex] > debounceMS) {
					if (BIT_CHECK(g_lastValidState,pinIndex)) {
						BIT_TGL(g_lastValidState, pinIndex);
						if (!CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
							CHANNEL_Toggle(g_cfg.pins.channels[pinIndex]);
							EventHandlers_FireEvent(CMD_EVENT_PIN_ONTOGGLE, pinIndex);
						} else {
							ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
						}
					}
				} else {
					g_times2[pinIndex] += timeSinceLast;
				}
				g_times[pinIndex] = 0;
			}
			break;

		default:
			{
			static bool printedOnce = false;
			if (!printedOnce) {
				printedOnce = false;
				ADDLOG_ERROR(LOG_FEATURE_DRV, "%s - Error pin role %i not accounted for.", __func__, pinRole);
			}
			break;
			}
		}
		// clear processed pin and exit if no more left to process
		if (!BIT_CLEAR(driverPins, pinIndex))
			break;
	}
}

uint32_t Digital_digitalCount() {
	return g_digitalCount;
}

static bool Digital_activatePin(uint32_t pinIndex) {
	bool channelValue = CHANNEL_Get(PIN_GetPinChannelForPinIndex(pinIndex));
	bool falling, pullup;
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_ToggleChannelOnToggle:
		falling = true;
		break;

	case IOR_DigitalInput_n:
		falling = true;
	case IOR_DigitalInput:
		pullup = true;
		g_digitalCount++;
		break;

	case IOR_DigitalInput_NoPup_n:
		falling = true;
	case IOR_DigitalInput_NoPup:
		g_digitalCount++;
		break;

	default:
		return false;
	}
	
	if (pullup)
		HAL_PIN_Setup_Input_Pullup(pinIndex);
	else
		HAL_PIN_Setup_Input(pinIndex);

	PIN_setGPIActive(pinIndex, 1, falling);
	BIT_SET_TO(g_lastValidState, pinIndex, channelValue);
	BIT_SET(g_driverPins, pinIndex);
	return true;
}

static void Digital_releasePin(uint32_t pinIndex) {
	BIT_CLEAR(g_driverPins, pinIndex);
	PIN_setGPIActive(pinIndex, 0, 0);
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_DigitalInput_n:
	case IOR_DigitalInput:
	case IOR_DigitalInput_NoPup_n:
	case IOR_DigitalInput_NoPup:
		g_digitalCount--;
		break;
	}
}

static void Digital_stopDriver() {
	g_digitalCount = 0;
	g_dynamicWakeEdge = 0xFFFFFFFF;
	g_defaultWakeEdge = 0x00000000;
	for (uint32_t usedIndex = 0; g_driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(g_driverPins, pinIndex))
			continue;
		PIN_setGPIActive(pinIndex, 0, 0);
		BIT_CLEAR(g_driverPins, pinIndex);
	}
}

static void Digital_init() {
#if ENABLE_DEEPSLEEP
	//cmddetail:{"name":"Digital_setEdge","args":"[edgeCode][optionalPinIndex]",
	//cmddetail:"descr":"Deep sleep (PinDeepSleep) wake configuration command. 0 means always wake up on rising edge, 1 means on falling, 2 means if state is high, use falling edge, if low, use rising. Default is 2. Second argument is optional and allows to set per-pin DSEdge instead of setting it for all pins.",
	//cmddetail:"fn":"CMD_Digital_setEdge","file":"driver/drv_digital.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("Digital_setEdge", CMD_Digital_setEdge, NULL);
#endif
	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}

// framework request function
uint32_t Digital_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_DigitalInput] = PIN_pinIORoleDriver()[IOR_DigitalInput_n] \
		              = PIN_pinIORoleDriver()[IOR_DigitalInput_NoPup] = PIN_pinIORoleDriver()[IOR_DigitalInput_NoPup_n] \
					  = PIN_pinIORoleDriver()[IOR_ToggleChannelOnToggle] = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return Digital_activatePin(arg);

	case OBKF_ReleasePin:
		Digital_releasePin(arg);
		break;

	case OBKF_Stop:
		Digital_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return (arg == IOR_ToggleChannelOnToggle) ? false : true;

	case OBKF_NoOfChannels:
		return 1;

	case OBKF_Init:
		Digital_init();
		break;

	default:
		break;
	}

	return true;
}

bool Digital_isDigital(uint32_t channelIndex) {
	if (!g_driverPins)
		return false;

	uint32_t driverPins = g_driverPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(g_driverPins, pinIndex))
			continue;
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue;
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_DigitalInput_n:
		case IOR_DigitalInput:
		case IOR_DigitalInput_NoPup_n:
		case IOR_DigitalInput_NoPup:
			return true;
		}

	}

}
#else
void Digital_setEdges() {
	// silently ignore the command
}
void Digital_quickTick(uint32_t timeSinceLast) {
}
#endif // ENABLE_DRIVER_DIGITAL
