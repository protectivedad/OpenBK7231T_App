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

static int g_noChangeTimePassed; // time without change. Every event of the doorsensor channel resets it.
static int g_emergencyTimeWithNoConnection; // time without connection to MQTT. Extends the interval till Deep Sleep until connection is established or EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT

static int g_registeredPin = -1; // pin found on initialization
static bool g_lastValidState = false;
static int setting_automaticWakeUpAfterSleepTime = 0;
static int setting_timeRequiredUntilDeepSleep = 60;
static int g_driverIndex;
static int g_defaultWakeEdge = 2;

#define EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT 60 * 5

int Simulator_GetNoChangeTimePassed() {
	return g_noChangeTimePassed;
}
int Simulator_GetDoorSennsorAutomaticWakeUpAfterSleepTime() {
	return setting_automaticWakeUpAfterSleepTime;
}

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
		g_defaultWakeEdge = Tokenizer_GetArgInteger(0);
	}

	return CMD_RES_OK;
}

// clear timer values
static void DoorSensor_clearTimers() {
	g_noChangeTimePassed = 0;
	g_emergencyTimeWithNoConnection = 0;
}

// read and returns monitored pin value (inverted if flag is set)
static bool DoorSensor_pinValue() {
	bool pinValue = HAL_PIN_ReadDigitalInput(g_registeredPin);
	return CFG_HasFlag(OBK_FLAG_DOORSENSOR_INVERT_STATE) ? !pinValue : pinValue;
}

// Finds associated pin, first assigned is chosen
// returns false if no pin is found
static bool DoorSensor_AssignPin(int pinIndex) {
	int pinIORole = PIN_GetPinRoleForPinIndex(pinIndex);
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
static int DoorSensor_ActivatePin(int pinIndex) {
	if (DoorSensor_AssignPin(pinIndex)) {
		setGPIActive(g_registeredPin, 1, (g_defaultWakeEdge == 2) ? DoorSensor_pinValue() : g_defaultWakeEdge);
		g_lastValidState = CHANNEL_Get(PIN_GetPinChannelForPinIndex(g_registeredPin));
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

	ADDLOGF_TIMING("%i - %s - Pin index %i, last valid state %i", xTaskGetTickCount(), __func__, g_registeredPin, g_lastValidState);
}

static void DoorSensor_ReleasePin(int pinIndex) {
	setGPIActive(pinIndex, 0, 0);
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
void DoorSensor_OnEverySecond() {
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
void DoorSensor_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if (bPreState)
		return;
	int untilSleep = EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT - g_emergencyTimeWithNoConnection;

#if ENABLE_MQTT
	if (Main_HasMQTTConnected())
		untilSleep = setting_timeRequiredUntilDeepSleep - g_noChangeTimePassed;
#endif
	hprintf255(request, "<h2>Door %s: deep sleep: %i (s)</h2>", g_lastValidState ? "open" : "closed", untilSleep);
}

// framework request function
int DoorSensor_frameworkRequest(int obkfRequest, int arg) {
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
		return DoorSensor_ActivatePin(arg);

	case OBKF_ReleasePin:
		DoorSensor_ReleasePin(arg);
		break;

	case OBKF_Stop:
		DoorSensor_StopDriver();
		break;
		
	case OBKF_ShouldPublish:
	case OBKF_NoOfChannels:
		return 1;

	default:
		break;
	}
	return true;
}

// quick tick timing loop function
// reads pin value posts changes to channel
void DoorSensor_QuickTick() {
	if (OTA_GetProgress() >= 0)
		return;

	if (g_registeredPin == -1)
		return;

	// might need debouncing
	bool pinValue = DoorSensor_pinValue();
	if (pinValue != g_lastValidState) {
		CHANNEL_Set(PIN_GetPinChannelForPinIndex(g_registeredPin), pinValue, 0);
		g_lastValidState = pinValue;
		DoorSensor_clearTimers();
		ADDLOGF_TIMING("%i - %s - Door Sensor channel is being set to state %i", xTaskGetTickCount(), __func__, pinValue);
	}
}
#endif // ENABLE_DRIVER_DOORSENSOR
