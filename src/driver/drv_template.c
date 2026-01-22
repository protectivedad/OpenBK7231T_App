// Driver Template Not for Compiling

#include "../obk_config.h"

#include "drv_local.h"
#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../logging/logging.h"

uint32_t g_driverIndex;
uint32_t g_driverPins;

// delete these and use proper ones 
#define IOR_Template 1
#define IOR_Template_n 2

commandResult_t CMD_Template_someCommand(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	if (Tokenizer_GetArgsCount() > 1)
		; // TODO: do some action
	else
		; // TODO: do some action

	return CMD_RES_OK;
}

static void Template_init() {
	//cmddetail:{"name":"Template_someCommand","args":"TODO",
	//cmddetail:"descr":"TODO",
	//cmddetail:"fn":"CMD_Template_someCommand","file":"driver/drv_template.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("Template_someCommand", CMD_Template_someCommand, NULL);

	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}

static bool Template_activatePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_Template:
		BIT_SET(g_driverPins, pinIndex);
		break;
	
	case IOR_Template_n:
		BIT_SET(g_driverPins, pinIndex);
		break;

	default:
		return false;
	}
	return true;
}

static void Template_releasePin(uint32_t pinIndex) {
	BIT_CLEAR(g_driverPins, pinIndex);
}

static void Template_stopDriver() {
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(g_driverPins, pinIndex))
			continue;
		// do something if required
		if (!BIT_CLEAR(g_driverPins, pinIndex))
			break;
	}
}

static bool Template_shouldPublish(uint32_t pinRole) {
	switch (pinRole) {
	case IOR_Template:
	case IOR_Template_n:
		return true;
	}
	return false;
}

static uint32_t Template_noOfChannels(uint32_t pinRole) {
	switch (pinRole) {
	case IOR_Template:
	case IOR_Template_n:
		return 1;
	}
	return 0;
}

// framework request function
uint32_t Template_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		// replace with real values for this driver
		g_driverIndex = PIN_pinIORoleDriver()[IOR_Template] = PIN_pinIORoleDriver()[IOR_Template_n] \
		              = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return Template_activatePin(arg);

	case OBKF_ReleasePin:
		Template_releasePin(arg);
		break;

	case OBKF_Stop:
		Template_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return Template_shouldPublish(arg);

	case OBKF_NoOfChannels:
		return Template_noOfChannels(arg);

	case OBKF_Init:
		Template_init();
		break;

	default:
		break;
	}

	return true;
}

// lookup channel and determine is a Template pin is assigned to it
bool Template_isTemplate(uint32_t channelIndex) {
	uint32_t driverPins = g_driverPins;

	for (uint32_t usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not assigned to pin
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_Template:
		case IOR_Template_n:
			return true;
		}
		BIT_CLEAR(driverPins, pinIndex);
	}
	return false;
}