// Driver Template Not for Compiling

#include "../obk_config.h"

#if ENABLE_DRIVER_ADC

#include "drv_local.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"
#include "../quicktick.h"
#include "../hal/hal_adc.h"

static uint32_t g_driverIndex;
static uint32_t g_driverPins;

#if OBK_ADC_INCLUDE_BUTTON
// Like: 450 1250 2900
static int *g_ranges = 0;
static uint32_t g_numRanges = 0;
static int32_t g_prevButton = -1;

commandResult_t Cmd_ADCButtonMap(const void* context, const char* cmd, const char* args, int cmdFlags) {
	uint32_t cnt, i;

	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	if (g_ranges)
		free(g_ranges);
	cnt = Tokenizer_GetArgsCount();
	g_ranges = (int*)malloc(sizeof(int)*cnt);
	for (i = 0; i < cnt; i++) {
		g_ranges[i] = Tokenizer_GetArgInteger(i);
	}
	g_numRanges = cnt;

	return CMD_RES_OK;
}
#endif // OBK_ADC_INCLUDE_BUTTON

static void ADC_init() {
#if OBK_ADC_INCLUDE_BUTTON
	//cmddetail:{"name":"AB_Map","args":"[int]",
	//cmddetail:"descr":"Sets margines for ADC button codes. For given N margins, there are N+1 possible ADC button values (one should be reserved for 'no button')",
	//cmddetail:"fn":"Cmd_ADCButtonMap","file":"driver/drv_adc.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("AB_Map", Cmd_ADCButtonMap, NULL);
#endif // OBK_ADC_INCLUDE_BUTTON

	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}

static bool ADC_activatePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_ADC_Button:
	case IOR_ADC:
		BIT_SET(g_driverPins, pinIndex);
		// init ADC for given pin
#if PLATFORM_XRADIO
		OBK_HAL_ADC_Init(index);
#else
		HAL_ADC_Init(pinIndex);
#endif
		break;

	default:
		return false;
	}
	return true;
}

static void ADC_releasePin(uint32_t pinIndex) {
	HAL_ADC_Deinit(pinIndex);
	BIT_CLEAR(g_driverPins, pinIndex);
}

static void ADC_stopDriver() {
	uint32_t pinIndex;
	PINS_PROCESS_WITH_CODE(g_driverPins, pinIndex, 
		ADC_releasePin(pinIndex);
	)
}

// framework request function
uint32_t ADC_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		// replace with real values for this driver
		g_driverIndex = PIN_pinIORoleDriver()[IOR_ADC] = PIN_pinIORoleDriver()[IOR_ADC_Button] \
		              = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return ADC_activatePin(arg);

	case OBKF_ReleasePin:
		ADC_releasePin(arg);
		break;

	case OBKF_Stop:
		ADC_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return (arg == IOR_ADC);

	case OBKF_NoOfChannels:
		return 1;

	case OBKF_Init:
		ADC_init();
		break;

	default:
		break;
	}

	return true;
}

void ADC_onEverySecond() {
	uint32_t pinIndex;
	PINS_PROCESS_WITH_CODE(g_driverPins, pinIndex, 
		int value = HAL_ADC_Read(pinIndex);
		CHANNEL_Set(PIN_GetPinChannelForPinIndex(pinIndex), value, CHANNEL_SET_FLAG_SILENT);
	)
}

// lookup channel and determine is a ADC pin is assigned to it
bool ADC_isMyChannel(uint32_t channelIndex) {

	uint32_t pinIndex;
	PINS_PROCESS_WITH_CODE(g_driverPins, pinIndex, 
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not assigned to pin
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_ADC:
		case IOR_ADC_Button:
			return true;
		}
	)
	return false;
}

static int chooseButton(int value) {
	for (uint32_t i = 0; i < g_numRanges; i++)
		if (g_ranges[i] > value)
			return i;

	return g_numRanges;
}

void ADC_quickTick() {
#if OBK_ADC_INCLUDE_BUTTON
	static uint32_t timeAccum = 0;
	timeAccum += g_deltaTimeMS;
	if (timeAccum > 100) {
		uint32_t pinIndex;
		PINS_PROCESS_WITH_CODE(g_driverPins, pinIndex, 
			if (PIN_GetPinRoleForPinIndex(pinIndex) == IOR_ADC_Button) {
				int32_t adcValue = HAL_ADC_Read(pinIndex);
				int32_t newButton = chooseButton(adcValue);
				ADDLOG_INFO(LOG_FEATURE_GENERAL, "ADC %i -> button %i (total %i)\r\n", adcValue, newButton, g_numRanges);
				if (newButton != g_prevButton) {
					EventHandlers_FireEvent(CMD_EVENT_ADC_BUTTON, newButton);
					g_prevButton = newButton;
				}
			}
		)
		timeAccum = 0;
	}
#endif // OBK_ADC_INCLUDE_BUTTON
}

#endif // ENABLE_DRIVER_ADC
