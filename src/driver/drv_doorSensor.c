// Door Sensor Driver
// Keeps track of one pin and posts any updates to one channel 
// using quick tick timing. Diplays open/closed status along with
// time until sleep. Counts down time to sleep on the every second
// timer loop. When time to sleep reaches zero driver requests the
// system go to sleep. Time to sleep resets when pin changes state.

// Driver is responsible for:
// 1 - taking ownership of IORoles
// 2 - initialization
// 3 - registering commands
// 4 - assigning pins to allocated IORoles
// 5 - every second loop function
// 6 - quick tick loop function

#include "../obk_config.h"
#if ENABLE_DRIVER_DOORSENSOR

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

uint32_t g_noChangeTimePassed; // time without change. Every event of the doorsensor channel resets it.
uint32_t g_emergencyTimeWithNoConnection; // time without connection to MQTT. Extends the interval till Deep Sleep until connection is established or EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT

int32_t  g_registeredPin = -1; // pin found on initialization
uint32_t setting_automaticWakeUpAfterSleepTime = 0;
uint32_t setting_timeRequiredUntilDeepSleep = 60;
uint32_t g_driverIndex;

// this is the invert of the initial state of the pin on wake
uint32_t g_ds_defaultWakeEdge = 2;
bool g_ds_lastPinState = false;
bool g_ds_lastChState = false;

#define EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT 60 * 5

#ifdef WINDOWS
int Simulator_GetNoChangeTimePassed() {
	return g_noChangeTimePassed;
}
int Simulator_GetDoorSennsorAutomaticWakeUpAfterSleepTime() {
	return setting_automaticWakeUpAfterSleepTime;
}
#endif

static commandResult_t DoorSensor_SetTime(const void* context, const char* cmd, const char* args, int cmdFlags) {
	const char *a;

	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	a = Tokenizer_GetArg(0);
	if (a[0] == 'c') {
		g_noChangeTimePassed = 0;
	} else if (a[0] == '+') {
		g_noChangeTimePassed -= atoi(a + 1);;
	}
	else {
		setting_timeRequiredUntilDeepSleep = Tokenizer_GetArgInteger(0);
	}
	if (Tokenizer_GetArgsCount() > 1) {
		setting_automaticWakeUpAfterSleepTime = Tokenizer_GetArgInteger(1);
	}

	return CMD_RES_OK;
}

static commandResult_t DoorSensor_SetEdge(const void* context, const char* cmd, const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, TOKENIZER_ALLOW_QUOTES | TOKENIZER_DONT_EXPAND);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1))
	{
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	// strlen("DoorSensor_SetEdge") == 6
	if (Tokenizer_GetArgsCount() == 1) {
		g_ds_defaultWakeEdge = Tokenizer_GetArgInteger(0);
	}

	return CMD_RES_OK;
}

// clear timer values
static void DoorSensor_clearTimers() {
	g_noChangeTimePassed = 0;
	g_emergencyTimeWithNoConnection = 0;
}

// Finds associated pin, first assigned is chosen
// returns false if no pin is found
static bool DoorSensor_AssignPin(uint32_t pinIndex) {
	uint32_t pinIORole = PIN_GetPinRoleForPinIndex(pinIndex);
	switch (pinIORole) {
	case IOR_DoorSensor:
		g_registeredPin = pinIndex;
		HAL_PIN_Setup_Input_Pullup(g_registeredPin);
		// this is input - sample initial state down below
		return true;
	case IOR_DoorSensor_pd:
		g_registeredPin = pinIndex;
		HAL_PIN_Setup_Input_Pulldown(g_registeredPin);
		// this is input - sample initial state down below
		return true;
	case IOR_DoorSensor_NoPup:
		g_registeredPin = pinIndex;
		HAL_PIN_Setup_Input(g_registeredPin);
		// this is input - sample initial state down below
		return true;
	default:
		return false;
	}
}

// assigns pin and does any activation, active status, last state, etc
// returns true when pin is found
static bool DoorSensor_activatePin(uint32_t pinIndex) {
	if (DoorSensor_AssignPin(pinIndex)) {
		bool pinValue = !HAL_PIN_ReadDigitalInput(g_registeredPin);
		ADDLOG_INFO(LOG_FEATURE_DRV, "%s - Activate pin %i with falling = %i", __func__, pinIndex, pinValue);
		PIN_setGPIActive(g_registeredPin, 1, (g_ds_defaultWakeEdge == 2) ? pinValue : g_ds_defaultWakeEdge);
		g_ds_lastChState = CHANNEL_Get(PIN_GetPinChannelForPinIndex(g_registeredPin));
		g_ds_lastPinState = CFG_HasFlag(OBK_FLAG_DOORSENSOR_INVERT_STATE) ? !g_ds_lastChState : g_ds_lastChState;
	} else {
		g_registeredPin = -1;
	}

	return (g_registeredPin != -1);
}

// clears timing counters and registers commands
static void DoorSensor_Init() {
	DoorSensor_clearTimers();

	//cmddetail:{"name":"DSTime","args":"[timeSeconds][optionalAutoWakeUpTimeSeconds]",
	//cmddetail:"descr":"DoorSensor driver configuration command. Time to keep device running before next sleep after last door sensor change. In future we may add also an option to automatically sleep after MQTT confirms door state receival. You can also use this to extend current awake time (at runtime) with syntax: 'DSTime +10', this will make device stay awake 10 seconds longer. You can also restart current value of awake counter by 'DSTime clear', this will make counter go from 0 again.",
	//cmddetail:"fn":"DoorSensor_SetTime","file":"driver/drv_doorSensor.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("DoorSensor_SetTime", DoorSensor_SetTime, NULL);

	//cmddetail:{"name":"DoorSensor_SetEdge","args":"[edgeCode]",
	//cmddetail:"descr":"DoorSensor wake configuration command. 0 means always wake up on rising edge, 1 means on falling, 2 means if state is high, use falling edge, if low, use rising. Default is 2. Second argument is optional and allows to set per-pin DSEdge instead of setting it for all pins.",
	//cmddetail:"fn":"DoorSensor_SetEdge","file":"driver/drv_doorSensor.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("DoorSensor_SetEdge", DoorSensor_SetEdge, NULL);

	ADDLOGF_TIMING("%i - %s - Pin index %i, last pin state %i", xTaskGetTickCount(), __func__, g_registeredPin, g_ds_lastPinState);
}

static void DoorSensor_ReleasePin(uint32_t pinIndex) {
	PIN_setGPIActive(pinIndex, 0, 0);
	if (pinIndex == g_registeredPin)
		g_registeredPin = -1;
}

// runs deactivation of any previous assigned pin, clears pin and timers
static void DoorSensor_StopDriver() {
	// reset pins and time passed values
	if (g_registeredPin != -1)
		DoorSensor_ReleasePin(g_registeredPin);
	DoorSensor_clearTimers();
}

// every second timing loop function
// counts seconds and requests deep sleep when countdown reaches zero
void DoorSensor_onEverySecond() {
	if (OTA_GetProgress() >= 0)
		return;

	if (g_registeredPin == -1)
		return;

#if ENABLE_MQTT
	if (Main_HasMQTTConnected()) { // executes every second when connection is established
		if (++g_noChangeTimePassed >= setting_timeRequiredUntilDeepSleep) {
			g_bWantPinDeepSleep = true;
			g_pinDeepSleepWakeUp = setting_automaticWakeUpAfterSleepTime;
		}
	}
	else 
#endif
	{ // executes every second while the device is woken up, but offline
		if (++g_emergencyTimeWithNoConnection >= EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT) {
			g_bWantPinDeepSleep = true;
			g_pinDeepSleepWakeUp = setting_automaticWakeUpAfterSleepTime;
		}
	}
}

// posts html information
void DoorSensor_appendHTML(http_request_t* request, int bPreState)
{
	if (bPreState)
		return;
	uint32_t untilSleep = EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT - g_emergencyTimeWithNoConnection;

#if ENABLE_MQTT
	if (Main_HasMQTTConnected())
		untilSleep = setting_timeRequiredUntilDeepSleep - g_noChangeTimePassed;
#endif
	hprintf255(request, "<h2>Door %s: deep sleep: %i (s)</h2>", g_ds_lastChState ? "open" : "closed", untilSleep);
}

// framework request function
uint32_t DoorSensor_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest)
	{
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_DoorSensor] \
		              = PIN_pinIORoleDriver()[IOR_DoorSensor_NoPup] \
					  = PIN_pinIORoleDriver()[IOR_DoorSensor_pd] \
					  = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_Init:
		DoorSensor_Init();
		break;

	case OBKF_AcquirePin:
		return DoorSensor_activatePin(arg);

	case OBKF_ReleasePin:
		DoorSensor_ReleasePin(arg);
		break;

	case OBKF_Stop:
		DoorSensor_StopDriver();
		break;
		
	case OBKF_ShouldPublish:
	case OBKF_NoOfChannels:
		return true;

	default:
		break;
	}
	return true;
}

// quick tick timing loop function
// reads pin value posts changes to channel
void DoorSensor_quickTick() {
	if (OTA_GetProgress() >= 0)
		return;

	if (g_registeredPin == -1)
		return;

	// might need debouncing
	if (HAL_PIN_ReadDigitalInput(g_registeredPin) != g_ds_lastPinState) {
		g_ds_lastPinState = !g_ds_lastPinState;
		g_ds_lastChState = CFG_HasFlag(OBK_FLAG_DOORSENSOR_INVERT_STATE) ? !g_ds_lastPinState : g_ds_lastPinState;
		DoorSensor_clearTimers();
		CHANNEL_Set(PIN_GetPinChannelForPinIndex(g_registeredPin), g_ds_lastChState, 0);
		PIN_setGPIActive(g_registeredPin, 1, (g_ds_defaultWakeEdge == 2) ? g_ds_lastChState : g_ds_defaultWakeEdge);
		ADDLOGF_TIMING("%i - %s - Door Sensor channel is being set to state %i", xTaskGetTickCount(), __func__, g_ds_lastChState);
	}
}
#endif // ENABLE_DRIVER_DOORSENSOR
