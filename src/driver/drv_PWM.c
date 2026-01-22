// Driver PWM Not for Compiling

#include "../obk_config.h"

#include "drv_local.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"

uint32_t g_driverIndex;
uint32_t g_driverPins;
uint32_t g_countPWM;

#define PWM_FREQUENCY_DEFAULT 1000 //Default Frequency
#define PWM_FREQUENCY_SLOW 600 //Slow frequency for LED Drivers requiring slower PWM Freq

int g_pwmFrequency = PWM_FREQUENCY_DEFAULT;

commandResult_t CMD_PWMFrequency(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	
	g_pwmFrequency = Tokenizer_GetArgInteger(0);

#ifdef PLATFORM_ESPIDF
	esp_err_t err = ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)g_pwmFrequency);
	if(err == ESP_ERR_INVALID_ARG)
	{
		ADDLOG_ERROR(LOG_FEATURE_CMD, "ledc_set_freq: invalid arg");
		return CMD_RES_BAD_ARGUMENT;
	}
	else if(err == ESP_FAIL)
	{
		ADDLOG_ERROR(LOG_FEATURE_CMD, "ledc_set_freq: Can not find a proper pre-divider number base on the given frequency and the current duty_resolution");
		return CMD_RES_ERROR;
	}
#endif
	// reapply PWM settings
	PIN_SetupPins();

	return CMD_RES_OK;
}

static void PWM_init() {
	//cmddetail:{"name":"PWMFrequency","args":"[FrequencyInHz]",
	//cmddetail:"descr":"Sets the global PWM frequency.",
	//cmddetail:"fn":"CMD_PWMFrequency","file":"driver/drv_PWM.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("PWMFrequency", CMD_PWMFrequency, NULL);

	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}

static bool PWM_activatePin(uint32_t pinIndex) {
	uint32_t channelIndex = PIN_GetPinChannelForPinIndex(pinIndex); 
	float channelValue = CHANNEL_GetFloat(channelIndex);

	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_PWM_n:
		g_countPWM++;
	case IOR_PWM_ScriptOnly_n:
		channelValue = 100.0f - channelValue;
		break;

	case IOR_PWM:
		g_countPWM++;
	case IOR_PWM_ScriptOnly:
		break;

	default:
		return false;
	}

	HAL_PIN_PWM_Start(pinIndex, CFG_HasFlag(OBK_FLAG_SLOW_PWM) ? PWM_FREQUENCY_SLOW : g_pwmFrequency);
	HAL_PIN_PWM_Update(pinIndex, channelValue);
	BIT_SET(g_driverPins, pinIndex);
	return true;
}

static void PWM_releasePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_PWM_n:
	case IOR_PWM:
		g_countPWM--;
	case IOR_PWM_ScriptOnly_n:
	case IOR_PWM_ScriptOnly:
		HAL_PIN_PWM_Stop(pinIndex);
		BIT_CLEAR(g_driverPins, pinIndex);
		break;
	}
}

static void PWM_stopDriver() {
	for (uint32_t usedIndex = 0; g_driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(g_driverPins, pinIndex))
			continue;
		PWM_releasePin(pinIndex);
	}
}

static bool PWM_shouldPublish(uint32_t pinRole) {
	switch (pinRole) {
	case IOR_PWM:
	case IOR_PWM_n:
		return true;
	}
	return false;
}

static uint32_t PWM_noOfChannels(uint32_t pinRole) {
	switch (pinRole) {
	case IOR_PWM_n:
	case IOR_PWM_ScriptOnly_n:
	case IOR_PWM:
	case IOR_PWM_ScriptOnly:
		return 1;
	}
	return 0;
}

// framework request function
uint32_t PWM_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_PWM] = PIN_pinIORoleDriver()[IOR_PWM_n] \
					  = PIN_pinIORoleDriver()[IOR_PWM_ScriptOnly] = PIN_pinIORoleDriver()[IOR_PWM_ScriptOnly_n] \
		              = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return PWM_activatePin(arg);

	case OBKF_ReleasePin:
		PWM_releasePin(arg);
		break;

	case OBKF_Stop:
		PWM_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return PWM_shouldPublish(arg);

	case OBKF_NoOfChannels:
		return PWM_noOfChannels(arg);

	case OBKF_Init:
		PWM_init();
		break;

	default:
		break;
	}

	return true;
}

void PWM_onChanged(uint32_t channelIndex, float iVal) {
	if (!g_enable_pins)
		return;

	uint32_t driverPins = g_driverPins;
	for (uint32_t usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not for pin

		float channelValue = iVal;
		switch (pinIndex) {
		case IOR_PWM_n:
		case IOR_PWM_ScriptOnly_n:
			channelValue = 100.0f - channelValue;
		case IOR_PWM:
		case IOR_PWM_ScriptOnly:
			HAL_PIN_SetOutputValue(pinIndex, !channelValue);
		}

		BIT_CLEAR(driverPins, pinIndex);
	}
}

// lookup channel and determine is a PWM pin is assigned to it
bool PWM_isPWM(uint32_t channelIndex) {
	uint32_t driverPins = g_driverPins;

	for (uint32_t usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not assigned to pin
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_PWM:
		case IOR_PWM_n:
			return true;
		}
		BIT_CLEAR(driverPins, pinIndex);
	}
	return false;
}

uint32_t PWM_maxPWM(uint32_t channelIndex) {
	uint32_t driverPins = g_driverPins;

	for (uint32_t usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not assigned to pin
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_PWM_n:
		case IOR_PWM_ScriptOnly_n:
		case IOR_PWM:
		case IOR_PWM_ScriptOnly:
			return 100;
		}
		BIT_CLEAR(driverPins, pinIndex);
	}
	return false;
}

uint32_t PWM_countPins() {
	return g_countPWM;
}

uint32_t PWM_countChannels() {
	uint32_t pwmBits, pwmCount;
	uint32_t driverPins = g_driverPins;

	for (int usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		int pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		int pinRole = PIN_GetPinRoleForPinIndex(pinIndex);
		switch (pinRole) {
		case IOR_PWM:
		case IOR_PWM_n:
			// if we have two PWMs on single channel, count it once
			BIT_SET(pwmBits, PIN_GetPinChannelForPinIndex(pinIndex));
			break;
		default:
			break;
		}
		BIT_CLEAR(driverPins, pinIndex);
	}
	for (int i = 0; i < 32; i++) {
		if (BIT_CHECK(pwmBits, i))
			pwmCount++;
	}
	return pwmCount;
}