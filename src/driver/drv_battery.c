#include "../obk_config.h"

#if ENABLE_DRIVER_BATTERY

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

int32_t g_pin_adc = -1, g_pin_rel = -1, g_val_rel = -1;
uint32_t g_battcycle = 1, g_battcycleref = 10;
float g_battvoltage = 0.0, g_battlevel = 0.0;
uint32_t g_lastbattvoltage = 0, g_lastbattlevel = 0;
float g_vref = 2400, g_vdivider = 2.29, g_maxbatt = 3000, g_minbatt = 2000, g_adcbits = 4096;
uint32_t g_driverIndex = 0;
bool g_measureTrigger;

#ifdef WINDOWS
void Simulator_Force_Batt_Measure() {
	Battery_measure();
}
#endif

static bool Battery_assignPin(uint32_t pinIndex) {
	uint32_t pinIORole = PIN_GetPinRoleForPinIndex(pinIndex);
	switch (pinIORole) {
	case IOR_BAT_ADC:
		g_pin_adc = pinIndex;
		HAL_ADC_Init(g_pin_adc);
		return true;
	case IOR_BAT_Relay:
		g_pin_rel = pinIndex;
		g_val_rel = 1;
		HAL_PIN_Setup_Output(g_pin_rel);
		HAL_PIN_SetOutputValue(g_pin_rel, !g_val_rel);
		return true;
	case IOR_BAT_Relay_n:
		g_pin_rel = pinIndex;
		g_val_rel = 0;
		HAL_PIN_Setup_Output(g_pin_rel);
		HAL_PIN_SetOutputValue(g_pin_rel, !g_val_rel);
		return true;
	default:
		return false;
	}
}

static void Battery_measure() {
	float batt_ref, batt_res, vref;
	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);

	if (g_pin_adc == -1) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : ADC pin not registered, reset device");
		return;
	}

	g_battvoltage = HAL_ADC_Read(g_pin_adc);
	ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : ADC binary Measurement : %f", g_battvoltage);
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : Calculation with param : %f %f %f", g_vref, g_adcbits, g_vdivider);
	// batt_value = batt_value / vref / 12bits value should be 10 un doc ... but on CBU is 12 ....
	vref = g_vref / g_adcbits;
	g_battvoltage = g_battvoltage * vref;
	// multiply by 2 cause ADC is measured after the Voltage Divider
	g_battvoltage = g_battvoltage * g_vdivider;

	// ignore values less then half the minimum but don't quit trying
	if ((g_minbatt / 2) > g_battvoltage) {
		ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : Reading invalid, ignoring will try again");
		return;
	}

	batt_ref = g_maxbatt - g_minbatt;
	batt_res = g_battvoltage - g_minbatt;
	ADDLOG_DEBUG(LOG_FEATURE_DRV, "DRV_BATTERY : Ref battery: %f, rest battery %f", batt_ref, batt_res);
	g_battlevel = (batt_res / batt_ref) * 100;
	if (g_battlevel < 0)
		g_battlevel = 0;
	if (g_battlevel > 100)
		g_battlevel = 100;

#if ENABLE_MQTT
	if (MQTT_IsReady()) {
		MQTT_PublishMain_StringInt("voltage", (int)g_battvoltage, 0);
		MQTT_PublishMain_StringInt("battery", (int)g_battlevel, 0);
	} else 	{
		char sValue[8];   // channel value as a string
		sprintf(sValue, "%i", (int)g_battvoltage);
		MQTT_QueuePublish(CFG_GetMQTTClientId(), "voltage/get", sValue, 0); // queue the publishing
		sprintf(sValue, "%i", (int)g_battlevel);
		MQTT_QueuePublish(CFG_GetMQTTClientId(), "battery/get", sValue, 0); // queue the publishing
	}
#endif
	g_lastbattlevel = (uint32_t)g_battlevel;
	g_lastbattvoltage = (uint32_t)g_battvoltage;
	ADDLOG_INFO(LOG_FEATURE_DRV, "DRV_BATTERY : battery voltage : %f and percentage %f%%", g_battvoltage, g_battlevel);
	g_battcycle--;
}

commandResult_t Battery_setup(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 2))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	g_minbatt = Tokenizer_GetArgFloat(0);
	g_maxbatt = Tokenizer_GetArgFloat(1);
	if (Tokenizer_GetArgsCount() > 2) {
		g_vdivider = Tokenizer_GetArgFloat(2);
	}
	if (Tokenizer_GetArgsCount() > 3) {
		g_vref = Tokenizer_GetArgFloat(3);
	}
	if (Tokenizer_GetArgsCount() > 4) {
		g_adcbits = Tokenizer_GetArgFloat(4);
	}

	ADDLOG_INFO(LOG_FEATURE_CMD, "Battery Setup : Min %f Max %f Vref %f adcbits %f vdivider %f", g_minbatt, g_maxbatt, g_vref, g_adcbits, g_vdivider);

	return CMD_RES_OK;
}


commandResult_t Battery_cycle(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	g_battcycleref = Tokenizer_GetArgInteger(0);

	ADDLOG_INFO(LOG_FEATURE_CMD, "Battery Cycle : Measurement will run every %i seconds", g_battcycleref);

	return CMD_RES_OK;
}

// startDriver Battery
static void Battery_init() {

	//cmddetail:{"name":"Battery_setup","args":"[minbatt][maxbatt][V_divider][Vref][AD Bits]",
	//cmddetail:"descr":"measure battery based on ADC. <br />req. args: minbatt in mv, maxbatt in mv. <br />optional: V_divider(2), Vref(default 2400), ADC bits(4096)",
	//cmddetail:"fn":"Battery_setup","file":"driver/drv_battery.c","requires":"",
	//cmddetail:"examples":"Battery_setup 1500 3000 2 2400 4096"}
	CMD_RegisterCommand("Battery_setup", Battery_setup, NULL);

	//cmddetail:{"name":"Battery_cycle","args":"[int]",
	//cmddetail:"descr":"change cycle of measurement by default every 10 seconds",
	//cmddetail:"fn":"Battery_cycle","file":"driver/drv_battery.c","requires":"",
	//cmddetail:"examples":"Battery_cycle 60"}
	CMD_RegisterCommand("Battery_cycle", Battery_cycle, NULL);

	if ((g_pin_rel != -1) && (g_pin_adc != -1)) {
		HAL_PIN_SetOutputValue(g_pin_rel, g_val_rel);
		HAL_ADC_Read(g_pin_adc);
		HAL_PIN_SetOutputValue(g_pin_rel, !g_val_rel);
	}

	ADDLOGF_TIMING("%i - %s - Registered ADC pin %i, with relay pin %i, activated with %i", xTaskGetTickCount(), __func__, g_pin_adc, g_pin_rel, g_val_rel);
}

void Battery_quickTick() {
	static bool enabledRelay;
	if (!g_measureTrigger)
		return;

	if (!enabledRelay && (g_pin_rel != -1)) {
		enabledRelay = true;
		HAL_PIN_SetOutputValue(g_pin_rel, g_val_rel);
	} else if (enabledRelay || (g_pin_rel == -1)) {
		Battery_measure();
		if (enabledRelay)
			HAL_PIN_SetOutputValue(g_pin_rel, !g_val_rel);
		enabledRelay = false;
		g_measureTrigger = false;
	}
}

void Battery_onEverySecond() {
	if (OTA_GetProgress() >= 0)
		return;

	if (g_pin_adc == -1)
		return;

	// Do nothing if cycle is set to zero and the last cycle is complete
	if (!g_battcycleref && !g_battcycle)
		return;

	if (!g_battcycle)
		g_battcycle = g_battcycleref;

	ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Measurement will run in %i cycle(s)", __func__, g_battcycle - 1);
	if (!g_measureTrigger) {
		if (g_battcycle == 1)
			g_measureTrigger = true;
		else
			g_battcycle--;
	}
}

static void Battery_releasePin(uint32_t pinIndex) {
	if (pinIndex == g_pin_adc) {
		HAL_ADC_Deinit(g_pin_adc);
		g_pin_adc = -1;
	} else if (pinIndex == g_pin_rel)
		g_pin_rel = -1;
}

static void Battery_stopDriver() {
	if (g_pin_adc != -1)
		Battery_releasePin(g_pin_adc);
	if (g_pin_rel != -1)
		Battery_releasePin(g_pin_rel);
}

void Battery_appendHTML(http_request_t* request, int bPreState)
{
	if (!bPreState)
		hprintf255(request, "<h2>Battery level=%.2f%%, voltage=%.2fmV</h2>", g_battlevel, g_battvoltage);
}

uint32_t Battery_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest)
	{
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_BAT_ADC] \
		              = PIN_pinIORoleDriver()[IOR_BAT_Relay] \
					  = PIN_pinIORoleDriver()[IOR_BAT_Relay_n] \
					  = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_Init:
		Battery_init();
		break;

	case OBKF_AcquirePin:
		return Battery_assignPin(arg);

	case OBKF_ReleasePin:
		Battery_releasePin(arg);
		break;

	case OBKF_Stop:
		Battery_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return false;

	case OBKF_NoOfChannels:
		return 0;

	default:
		break;
	}
	return true;
}
bool Battery_safeToUpdate() {
	if (g_pin_adc == -1)
		return true;

	return g_lastbattlevel > 5;
}
#else
bool Battery_safeToUpdate() {
	return true;
}
#endif // ENABLE_DRIVER_BATTERY
