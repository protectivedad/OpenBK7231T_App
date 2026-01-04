// Power-saving door sensor driver with deep sleep
// Device reboots and waits for MQTT connection, while monitoring door sensor status.
// After a certain amount of time with MQTT connection, to make sure that door state is 
// already reported, and with no futher door state changes, device goes to deep sleep 
// and waits for wakeup from door sensor input state change.

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

static int g_noChangeTimePassed = 0; // time without change. Every event of the doorsensor channel resets it.
static int g_emergencyTimeWithNoConnection = 0; // time without connection to MQTT. Extends the interval till Deep Sleep until connection is established or EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT
static int g_registeredPin = -1; // pin found on initialization
static int setting_automaticWakeUpAfterSleepTime = 0;
static int setting_timeRequiredUntilDeepSleep = 60;

#define EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT 60 * 5

int Simulator_GetNoChangeTimePassed() {
	return g_noChangeTimePassed;
}
int Simulator_GetDoorSennsorAutomaticWakeUpAfterSleepTime() {
	return setting_automaticWakeUpAfterSleepTime;
}

// addEventHandler OnClick 8 DSTime +100
commandResult_t DoorDeepSleep_SetTime(const void* context, const char* cmd, const char* args, int cmdFlags) {
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

int DoorDeepSleep_Load() {
	for (int i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (IS_PIN_DS_ROLE(g_cfg.pins.roles[i])) {
			g_registeredPin = i;
			break;
		}
	}
	return g_registeredPin;
}

void DoorDeepSleep_Init() {
	// 0 seconds since last change
	g_noChangeTimePassed = 0;

	//cmddetail:{"name":"DSTime","args":"[timeSeconds][optionalAutoWakeUpTimeSeconds]",
	//cmddetail:"descr":"DoorSensor driver configuration command. Time to keep device running before next sleep after last door sensor change. In future we may add also an option to automatically sleep after MQTT confirms door state receival. You can also use this to extend current awake time (at runtime) with syntax: 'DSTime +10', this will make device stay awake 10 seconds longer. You can also restart current value of awake counter by 'DSTime clear', this will make counter go from 0 again.",
	//cmddetail:"fn":"DoorDeepSleep_SetTime","file":"driver/drv_doorSensorWithDeepSleep.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("DSTime", DoorDeepSleep_SetTime, NULL);

	DoorDeepSleep_Load();
	ADDLOGF_TIMING("%i - %s - Registered pin %i", xTaskGetTickCount(), __func__, g_registeredPin);
}

void DoorDeepSleep_OnEverySecond() {

	if (OTA_GetProgress() >= 0) {
		return;
	}

	if (g_registeredPin == -1) {
		ADDLOG_WARN(LOG_FEATURE_DRV, "doorsensor: Driver not assigned to pin, restart device");
		return;
	}
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

void DoorDeepSleep_StopDriver() {
	// reset pins and time passed values
	g_registeredPin = -1;
	g_noChangeTimePassed = 0;
	g_emergencyTimeWithNoConnection = 0;
}

void DoorDeepSleep_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if (bPreState){
		return;
	}
	int untilSleep;

#if ENABLE_MQTT
	if (Main_HasMQTTConnected()) {
		untilSleep = setting_timeRequiredUntilDeepSleep - g_noChangeTimePassed;
		hprintf255(request, "<h2>Door (initial: %i): time until deep sleep: %i</h2>", g_initialPinStates, untilSleep);
	}
	else 
#endif
	{
		untilSleep = EMERGENCY_TIME_TO_SLEEP_WITHOUT_MQTT - g_emergencyTimeWithNoConnection;
		hprintf255(request, "<h2>Door (initial: %i): waiting for MQTT connection (but will emergency sleep in %i)</h2>", g_initialPinStates, untilSleep);
	}
}

void DoorDeepSleep_OnChannelChanged(int ch, int value) {
	// detect door state change
	// (only sleep when there are no changes for certain time)

	if (g_cfg.pins.channels[g_registeredPin] == ch) {
		ADDLOGF_TIMING("%i - %s - Channel %i is being set to state %i", xTaskGetTickCount(), __func__, ch, value);
		// 0 seconds since last change
		g_noChangeTimePassed = 0;
		g_emergencyTimeWithNoConnection = 0;
	}
}

#endif // ENABLE_DRIVER_DOORSENSOR
