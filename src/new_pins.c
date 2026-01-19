
#include "new_common.h"
#include "new_pins.h"
#include "quicktick.h"
#include "new_cfg.h"
#include "httpserver/new_http.h"
#include "logging/logging.h"
#include "mqtt/new_mqtt.h"
// Commands register, execution API and cmd tokenizer
#include "cmnds/cmd_public.h"
#include "i2c/drv_i2c_public.h"
#include "driver/drv_tuyaMCU.h"
#include "driver/drv_girierMCU.h"
#include "driver/drv_public.h"
#include "driver/drv_local.h"
#include "hal/hal_flashVars.h"
#include "hal/hal_pins.h"
#include "hal/hal_adc.h"

#ifdef PLATFORM_BEKEN
#include <gpio_pub.h>
#include "driver/drv_ir.h"
#elif PLATFORM_ESPIDF || PLATFORM_ESP8266
#include "hal/espidf/hal_pinmap_espidf.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#elif PLATFORM_XRADIO
#undef HAL_ADC_Init
#include "hal/xradio/hal_pinmap_xradio.h"
#include "driver/chip/hal_gpio.h"
#include "driver/chip/hal_wakeup.h"
#include "pm/pm.h"
#if PLATFORM_XR809
#define DEEP_SLEEP PM_MODE_POWEROFF
#else
#define DEEP_SLEEP PM_MODE_HIBERNATION
#endif
#elif PLATFORM_BL602
#include "bl_flash.h"
#include "bl602_hbn.h"
#endif

#ifdef PLATFORM_BEKEN_NEW
#include "manual_ps_pub.h"
#endif

byte *g_defaultWakeEdge = 0;

uint32_t g_pinIORoleDriver[IOR_Total_Options];
uint32_t g_registeredPinCount;
uint32_t registeredPinDetails[PLATFORM_GPIO_MAX];

#if ALLOW_SSID2
//20241125 XJIKKA SSID retain - last used SSID will be preserved
// To enable this feature, the channel that will be used to store the last SSID 
// must be set using the setStartupSSIDChannel command in early.bat. 
// It has to be in early.bat.Autoexec.bat is processed after the wifi data is loaded.
int g_StartupSSIDRetainChannel = -1; // -1 disabled, 0..MAX_RETAIN_CHANNELS-1 channel to store last SSID

int FV_GetStartupSSID_StoredValue(int adefault) {
	if ((g_StartupSSIDRetainChannel < 0) || (g_StartupSSIDRetainChannel >= MAX_RETAIN_CHANNELS)) return adefault;
	int fval = HAL_FlashVars_GetChannelValue(g_StartupSSIDRetainChannel);
	return (fval & 1);	////only SSID1 (0) and SSID2 (1) allowed
}
void FV_UpdateStartupSSIDIfChanged_StoredValue(int assidindex) {
	if ((g_StartupSSIDRetainChannel < 0) || (g_StartupSSIDRetainChannel >= MAX_RETAIN_CHANNELS)) return;
	if ((assidindex < 0) && (assidindex > 1)) return;	//only SSID1 (0) and SSID2 (1) allowed
	int fval = HAL_FlashVars_GetChannelValue(g_StartupSSIDRetainChannel);
	if (fval == assidindex) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "WiFi unchanged (SSID%i), HAL_FlashVars_SaveChannel skipped", assidindex+1);
		return;	//same value, no update
	}
	HAL_FlashVars_SaveChannel(g_StartupSSIDRetainChannel,assidindex);
}
#endif

#if ENABLE_DEEPSLEEP
void PIN_DeepSleep_MakeSureEdgesAreAlloced() {
	int i;
	if (g_defaultWakeEdge == 0) {
		g_defaultWakeEdge = (byte*)malloc(PLATFORM_GPIO_MAX);
		for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
			g_defaultWakeEdge[i] = 2;//default
		}
	}
}
void PIN_DeepSleep_SetWakeUpEdge(int pin, byte edgeCode) {
	PIN_DeepSleep_MakeSureEdgesAreAlloced();
	g_defaultWakeEdge[pin] = edgeCode;
}
void PIN_DeepSleep_SetAllWakeUpEdges(byte edgeCode) {
	int i;
	PIN_DeepSleep_MakeSureEdgesAreAlloced();
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		g_defaultWakeEdge[i] = edgeCode;
	}
}
#endif // ENABLE_DEEPSLEEP

// overall pins enable.
// if zero, all hardware action is disabled.
char g_enable_pins = 0;

// it was nice to have it as bits but now that we support PWM...
//int g_channelStates;
int g_channelValues[CHANNEL_MAX] = { 0 };
float g_channelValuesFloats[CHANNEL_MAX] = { 0 };

// a bitfield indicating which GPI are inputs.
// could be used to control edge triggered interrupts...
/*  @param  gpio_index_map:The gpio bitmap which set 1 enable wakeup deep sleep.
 *              gpio_index_map is hex and every bits is map to gpio0-gpio31.
 *          gpio_edge_map:The gpio edge bitmap for wakeup gpios,
 *              gpio_edge_map is hex and every bits is map to gpio0-gpio31.
 *              0:rising,1:falling.
 */
 // these map directly to void bk_enter_deep_sleep(uint32_t gpio_index_map,uint32_t gpio_edge_map);
uint32_t g_gpio_index_map[2] = { 0, 0 };
uint32_t g_gpio_edge_map[2] = { 0, 0 }; // note: 0->rising, 1->falling

#if PLATFORM_XRADIO
void SetWUPIO(int index, int pull, int edge)
{
	if(g_pins[index].wakeup != -1)
	{
		GPIO_InitParam param;

		param.mode = GPIOx_Pn_F6_EINT;
		param.driving = GPIO_DRIVING_LEVEL_1;
		param.pull = pull;
		HAL_GPIO_Init(GPIO_PORT_A, g_pins[index].pin, &param);
#if !PLATFORM_XR809
		HAL_PRCM_SetWakeupDebClk0(0);
		HAL_PRCM_SetWakeupIOxDebSrc(g_pins[index].wakeup, 0);
		HAL_PRCM_SetWakeupIOxDebounce(g_pins[index].wakeup, 1);
#endif
		HAL_Wakeup_SetIO(g_pins[index].wakeup, !edge, pull);
		//ADDLOG_INFO(LOG_FEATURE_GENERAL, "Waking up from PA%i, pull: %i, edge: %i", g_pins[index].pin, pull, !edge);
	}
}
#endif

void setGPIActive(int index, int active, int falling) {
	if (active) {
		if (index >= 32)
			g_gpio_index_map[1] |= (1 << (index - 32));
		else
			g_gpio_index_map[0] |= (1 << index);
	}
	else {
		if (index >= 32)
			g_gpio_index_map[1] &= ~(1 << (index - 32));
		else
			g_gpio_index_map[0] &= ~(1 << index);
	}
	if (falling) {
		if (index >= 32)
			g_gpio_edge_map[1] |= (1 << (index - 32));
		else
			g_gpio_edge_map[0] |= (1 << index);
	}
	else {
		if (index >= 32)
			g_gpio_edge_map[1] &= ~(1 << (index - 32));
		else
			g_gpio_edge_map[0] &= ~(1 << index);
	}
}
#if ENABLE_DEEPSLEEP
void PINS_BeginDeepSleepWithPinWakeUp(unsigned int wakeUpTime) {
	int i;
	int value;
	int falling;

	// door input always uses opposite level for wakeup
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (
			g_cfg.pins.roles[i] == IOR_DigitalInput
			|| g_cfg.pins.roles[i] == IOR_DigitalInput_n
			|| g_cfg.pins.roles[i] == IOR_DigitalInput_NoPup
			|| g_cfg.pins.roles[i] == IOR_DigitalInput_NoPup_n) {
			// added per request
			// https://www.elektroda.pl/rtvforum/viewtopic.php?p=20543190#20543190
			// forcing a certain edge for both states helps on some door sensors, somehow
			// 0 means always wake up on rising edge, 1 means on falling, 2 means if state is high, use falling edge, if low, use rising
			if (g_defaultWakeEdge == NULL || g_defaultWakeEdge[i] == 2) {
				value = HAL_PIN_ReadDigitalInput(i);
				if (value) {
					// on falling edge wake up
					falling = 1;
				}
				else {
					// on rising edge wake up
					falling = 0;
				}
			}
			else {
				falling = g_defaultWakeEdge[i];
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
			setGPIActive(i, 1, falling);
// #endif
		}
	}
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "Index map: %i, edge: %i", g_gpio_index_map[0], g_gpio_edge_map[0]);
#ifdef PLATFORM_BEKEN_NEW
	PS_DEEP_CTRL_PARAM params;
	params.gpio_index_map = g_gpio_index_map[0];
	params.gpio_edge_map = g_gpio_edge_map[0];
	params.sleep_mode = MANUAL_MODE_IDLE;
	if(wakeUpTime)
	{
		params.wake_up_way = PS_DEEP_WAKEUP_GPIO | PS_DEEP_WAKEUP_RTC;
		params.sleep_time = wakeUpTime;
	}
	else
	{
		params.wake_up_way = PS_DEEP_WAKEUP_GPIO;
	}
	bk_enter_deep_sleep_mode(&params);
#elif PLATFORM_BEKEN
	// NOTE: this function:
	// void bk_enter_deep_sleep(UINT32 gpio_index_map,UINT32 gpio_edge_map)
	// On BK7231T, will overwrite HAL pin settings, and depending on edge map,
	// will set a internal pullup or internall pulldown
#ifdef PLATFORM_BK7231T
	extern void deep_sleep_wakeup_with_gpio(UINT32 gpio_index_map, UINT32 gpio_edge_map);
	deep_sleep_wakeup_with_gpio(g_gpio_index_map[0], g_gpio_edge_map[0]);
#else
	extern void bk_enter_deep_sleep(UINT32 g_gpio_index_map, UINT32 g_gpio_edge_map);
	extern void deep_sleep_wakeup(const UINT32* g_gpio_index_map,
		const UINT32* g_gpio_edge_map, const UINT32* sleep_time);
	if (wakeUpTime) {
		deep_sleep_wakeup(&g_gpio_index_map[0],
			&g_gpio_edge_map[0],
			&wakeUpTime);
	}
	else {
		bk_enter_deep_sleep(g_gpio_index_map[0], g_gpio_edge_map[0]);
	}
#endif
#elif PLATFORM_XRADIO
	if(wakeUpTime)
	{
		HAL_Wakeup_SetTimer_mS(wakeUpTime * DS_MS_TO_S);
	}
	pm_enter_mode(DEEP_SLEEP);
#elif PLATFORM_ESPIDF
//	if(wakeUpTime)
//	{
//		esp_sleep_enable_timer_wakeup(timeMS * 1000000);
//	}
//#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
//	esp_deep_sleep_start();
//#else
//	ADDLOG_ERROR(LOG_FEATURE_GENERAL, "%s, doesn't support gpio deep sleep, entering light sleep.", PLATFORM_MCU_NAME);
//	delay_ms(10);
//	esp_sleep_enable_gpio_wakeup();
//	esp_light_sleep_start();
//#endif
#elif PLATFORM_BL602
	uint8_t wkup = HBN_WAKEUP_GPIO_NONE;
	HBN_GPIO_INT_Trigger_Type edge = HBN_GPIO_INT_TRIGGER_ASYNC_RISING_EDGE;
	uint8_t g7 = (g_gpio_index_map[0] >> 7) & 1;
	uint8_t g8 = (g_gpio_index_map[0] >> 8) & 1;
	uint8_t gf7 = (g_gpio_edge_map[0] >> 7) & 1;
	uint8_t gf8 = (g_gpio_edge_map[0] >> 8) & 1;
	if(g7) wkup |= HBN_WAKEUP_GPIO_7;
	if(g8) wkup |= HBN_WAKEUP_GPIO_8;
	// only one edge setting
	if(gf8) edge = HBN_GPIO_INT_TRIGGER_ASYNC_FALLING_EDGE;
	if(gf7) edge = HBN_GPIO_INT_TRIGGER_ASYNC_FALLING_EDGE;

	HBN_APP_CFG_Type cfg = {
		.useXtal32k = 0,
		.sleepTime = wakeUpTime,
		.gpioWakeupSrc = wkup,
		.gpioTrigType = edge,
		.flashCfg = bl_flash_get_flashCfg(),
		.hbnLevel = wakeUpTime > 0 ? HBN_LEVEL_1 : HBN_LEVEL_3,
		.ldoLevel = HBN_LDO_LEVEL_1P10V,
	};
	HBN_Clear_IRQ(HBN_INT_GPIO7);
	HBN_Clear_IRQ(HBN_INT_GPIO8);
	HBN_Mode_Enter(&cfg);
#endif
}
#endif // ENABLE_DEEPSLEEP


#ifdef PLATFORM_BEKEN
#ifdef BEKEN_PIN_GPI_INTERRUPTS
// TODO: EXAMPLE of edge based interrupt handling
// NOT YET ENABLED

// this will be from hal_bk....
// causes button read.
extern void BUTTON_TriggerRead();


// NOTE: ISR!!!!
// triggers one-shot timer to fire in 1ms
// from hal_main_bk7231.c
// THIS IS AN ISR.
void PIN_IntHandler(unsigned char index) {
	BUTTON_TriggerRead();
}

// this will be from hal_bk....
// ensures that we will get called in 50ms or less.
extern void BUTTON_TriggerRead_quick();

// called in PIN_ticks to carry on polling for 20 polls after last button is active
void PIN_TriggerPoll() {
	BUTTON_TriggerRead_quick();
}
#endif
#endif
extern int g_pwmFrequency;
unsigned short g_counterDeltas[PLATFORM_GPIO_MAX];
void PIN_InterruptHandler(int gpio) {
	g_counterDeltas[gpio]++;
}
static void PIN_ProcessNewPinRole(int index, int role) {
	if (g_enable_pins) {
		if (!role) // no role no processing
			return;
		int driverIndex = g_pinIORoleDriver[role];
		if (driverIndex) { // let driver take care of pins
			DRV_SendRequest(driverIndex, OBKF_AcquirePin, index);
			return;
		}

		int falling = 0;

		// init new role
		switch (role)
		{
		case IOR_IRRecv:
			falling = 1;
			// add to active inputs
			setGPIActive(index, 1, falling);
			break;
#if ENABLE_DRIVER_BRIDGE
		case IOR_BridgeForward:
		case IOR_BridgeReverse:
		{
			int channelIndex;
			int channelValue;

			channelIndex = PIN_GetPinChannelForPinIndex(index);
			channelValue = g_channelValues[channelIndex];

			HAL_PIN_Setup_Output(index);
			HAL_PIN_SetOutputValue(index, 0);
		}
		break;
#endif // ENABLE_DRIVER_BRIDGE
		case IOR_ADC_Button:
		case IOR_ADC:
			// init ADC for given pin
#if PLATFORM_XRADIO
			OBK_HAL_ADC_Init(index);
#else
			HAL_ADC_Init(index);
#endif
			break; 
		case IOR_Counter_f:
			HAL_PIN_Setup_Input_Pullup(index);
			HAL_AttachInterrupt(index, INTERRUPT_FALLING, PIN_InterruptHandler);
			break;
		case IOR_Counter_r:
			HAL_PIN_Setup_Input_Pullup(index);
			HAL_AttachInterrupt(index, INTERRUPT_RISING, PIN_InterruptHandler);
			break;
		case IOR_PWM_n:
		case IOR_PWM_ScriptOnly:
		case IOR_PWM_ScriptOnly_n:
		case IOR_PWM:
		{
			int channelIndex;
			float channelValue;

			channelIndex = PIN_GetPinChannelForPinIndex(index);
			channelValue = g_channelValuesFloats[channelIndex];

			//100hz to 20000hz according to tuya code
#define PWM_FREQUENCY_SLOW 600 //Slow frequency for LED Drivers requiring slower PWM Freq

			int useFreq;
			useFreq = g_pwmFrequency;
			//Use slow pwm if user has set checkbox in webif
			if (CFG_HasFlag(OBK_FLAG_SLOW_PWM))
				useFreq = PWM_FREQUENCY_SLOW;

			HAL_PIN_PWM_Start(index, useFreq);

			if (role == IOR_PWM_n
				|| role == IOR_PWM_ScriptOnly_n) {
				// inversed PWM
				HAL_PIN_PWM_Update(index, 100.0f - channelValue);
			}
			else {
				HAL_PIN_PWM_Update(index, channelValue);
			}
		}
		break;

		default:
			break;
		}
	}
}

static void PIN_addUsedPin(int pinIndex) {
		ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "%s - Added entry for pin %i", __func__, pinIndex);
		registeredPinDetails[g_registeredPinCount] = pinIndex;
		g_registeredPinCount++;
}

void PIN_SetupPins() {
	for (int pinIndex = 0; pinIndex < PLATFORM_GPIO_MAX; pinIndex++) {
		int role = g_cfg.pins.roles[pinIndex];
		if (role) { // only process pins with a role
			PIN_addUsedPin(pinIndex);
			int driverIndex = g_pinIORoleDriver[role];
			if (driverIndex) // let driver take care of pins
				DRV_SendRequest(driverIndex, OBKF_AcquirePin, pinIndex);
			else
				PIN_ProcessNewPinRole(pinIndex, role);
		}
	}

#ifdef PLATFORM_BEKEN
#ifdef BEKEN_PIN_GPI_INTERRUPTS
	// TODO: EXAMPLE of edge based interrupt handling
	// NOT YET ENABLED
	for (int i = 0; i < 32; i++) {
		if (g_gpio_index_map[0] & (1 << i)) {
			uint32_t mode = GPIO_INT_LEVEL_RISING;
			if (g_gpio_edge_map[0] & (1 << i)) {
				mode = GPIO_INT_LEVEL_FALLING;
			}
			if (g_cfg.pins.roles[i] == IOR_IRRecv) {
				// not yet implemented
				//gpio_int_enable(i, mode, IR_GPI_IntHandler);
			}
			else {
				gpio_int_enable(i, mode, PIN_IntHandler);
			}
		}
		else {
			gpio_int_disable(i);
		}
	}
#endif
#endif
#ifdef ENABLE_DRIVER_DHT
	// TODO: better place to call?
	DHT_OnPinsConfigChanged();
#endif
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "PIN_SetupPins pins have been set up.\r\n");
}

int PIN_GetPinRoleForPinIndex(int index) {
	return g_cfg.pins.roles[index];
}
int PIN_GetPinChannelForPinIndex(int index) {
	return g_cfg.pins.channels[index];
}
int PIN_CountPinsWithRoleOrRole(int role, int role2) {
	int i;
	int r = 0;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] == role)
			r++;
		else if (g_cfg.pins.roles[i] == role2)
			r++;
	}
	return r;
}
int PIN_CountPinsWithRole(int role) {
	int i;
	int r = 0;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] == role)
			r++;
	}
	return r;
}
int PIN_FindPinIndexForRole(int role, int defaultIndexToReturnIfNotFound) {
	int i;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] == role)
			return i;
	}
	return defaultIndexToReturnIfNotFound;
}
int PIN_GetPinChannel2ForPinIndex(int index) {
	return g_cfg.pins.channels2[index];
}
// return number of channels used for a role
// taken from code in http_fnc.c
int PIN_IOR_NofChan(int role) {
	int driverIndex = g_pinIORoleDriver[role];
	if (driverIndex) {
		return DRV_SendRequest(driverIndex, OBKF_NoOfChannels, role);
	}
	// For button, is relay index to toggle on double click
	if (IS_PIN_DHT_ROLE(role) || IS_PIN_TEMP_HUM_SENSOR_ROLE(role) || IS_PIN_AIR_SENSOR_ROLE(role)){
			return 2;
	}
	// Some roles don't need any channels
	if (role == IOR_SGP_CLK || role == IOR_SHT3X_CLK || role == IOR_CHT83XX_CLK
			|| role == IOR_BL0937_CF || role == IOR_BL0937_CF1 || role == IOR_BL0937_SEL
			|| role == IOR_BL0937_SEL_n || role == IOR_RCRecv || role == IOR_RCRecv_nPup
			|| (role >= IOR_IRRecv && role <= IOR_DHT11)
			|| (role >= IOR_SM2135_DAT && role <= IOR_BP1658CJ_CLK)
			|| (role == IOR_HLW8112_SCSN)
			|| (role == IOR_TuyaMCU)) {
			return 0;
	}
	// all others have 1 channel
	return 1;
}
// TODO: Remove
void RAW_SetPinValue(int index, int iVal) {
	if (index < 0 || index >= PLATFORM_GPIO_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_CFG, "RAW_SetPinValue: Pin index %i out of range <0,%i).", index, PLATFORM_GPIO_MAX);
		return;
	}
	if (g_enable_pins) {
		HAL_PIN_SetOutputValue(index, iVal);
	}
}

#define PIN_UART1_RXD 10
#define PIN_UART1_TXD 11
#define PIN_UART2_RXD 1
#define PIN_UART2_TXD 0

void CHANNEL_SetFirstChannelByTypeEx(int requiredType, int newVal, int ausemovingaverage) {
	int i;

	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_GetType(i) == requiredType) {
			CHANNEL_Set_Ex(i, newVal, 0, ausemovingaverage);
			return;
		}
	}
}

int CHANNEL_FindIndexForPinType(int requiredType) {
	int i;
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] == requiredType) {
			return g_cfg.pins.channels[i];
		}
	}
	return -1;
}

int CHANNEL_FindIndexForPinType2(int requiredType, int requiredType2) {
	int i;
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] == requiredType || g_cfg.pins.roles[i] == requiredType2) {
			return g_cfg.pins.channels[i];
		}
	}
	return -1;
}
int CHANNEL_FindIndexForType(int requiredType) {
	int i;
	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_GetType(i) == requiredType) {
			return i;
		}
	}
	return -1;
}
void CHANNEL_SetFirstChannelByType(int requiredType, int newVal) {
	CHANNEL_SetFirstChannelByTypeEx(requiredType, newVal, 0);
}
void CHANNEL_DoSpecialToggleAll() {
	int anyEnabled, i;

	anyEnabled = 0;

	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_IsPowerRelayChannel(i) == false)
			continue;
		if (g_channelValues[i] > 0) {
			anyEnabled = true;
		}
	}
	for (i = 0; i < CHANNEL_MAX; i++) {
		if (CHANNEL_IsPowerRelayChannel(i)) {
			CHANNEL_Set(i, !anyEnabled, 0);
		}
	}
}

static void PIN_ProcessOldPinRole(int index) {
	if (g_enable_pins) {
		int role = g_cfg.pins.roles[index];
		if (!role) // no role no processing
			return;
		int driverIndex = g_pinIORoleDriver[role];
		if (driverIndex) { // let driver take care of pins
			DRV_SendRequest(driverIndex, OBKF_ReleasePin, index);
			return;
		}
		// remove from active inputs
		setGPIActive(index, 0, 0);
		switch (role)
		{
		case IOR_PWM_n:
		case IOR_PWM_ScriptOnly:
		case IOR_PWM_ScriptOnly_n:
		case IOR_PWM:
			HAL_PIN_PWM_Stop(index);
			break;
		case IOR_ADC_Button:
		case IOR_ADC:
			HAL_ADC_Deinit(index);
			break;
		case IOR_Counter_f:
		case IOR_Counter_r:
			HAL_DetachInterrupt(index);
			break;

		default:
			break;
		}
	}
}

static void PIN_remUsedPin(int index) {
	int usedIndex;
	for (usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		if (registeredPinDetails[usedIndex] != index)
			continue;
		break;
	}
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "%s - Removed entry for pin index %i", __func__, usedIndex);
	for (usedIndex++; usedIndex < g_registeredPinCount; usedIndex++) {
		registeredPinDetails[usedIndex - 1] = registeredPinDetails[usedIndex];
	}
	registeredPinDetails[g_registeredPinCount] = 0;
	g_registeredPinCount--;
}

void PIN_SetPinRoleForPinIndex(int index, int role) {
	int oldRole = g_cfg.pins.roles[index];
	if (!(oldRole || role)) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "%s - What a waste of time that was!", __func__);
		return;
	}

	if (oldRole)
		PIN_ProcessOldPinRole(index);

	if (oldRole != role) {
		g_cfg.pins.roles[index] = role;
		g_cfg_pendingChanges++;
	}

	if (role)
		PIN_ProcessNewPinRole(index, role);

	// used pin allocation processing
	if (oldRole == role)
		return;

	if (!oldRole)
		PIN_addUsedPin(index);
	else if (!role)
		PIN_remUsedPin(index);
}

uint32_t PIN_registeredPinIndex(uint32_t usedIndex) {
	return registeredPinDetails[usedIndex];
}

uint32_t PIN_getDriverForRole(uint32_t pinRole) {
	return g_pinIORoleDriver[pinRole];
}
int* PIN_pinIORoleDriver() {
	return g_pinIORoleDriver;
}
void Channel_SaveInFlashIfNeeded(int ch) {
	// save, if marked as save value in flash (-1)
	if (g_cfg.startChannelValues[ch] == -1) {
		//ADDLOG_INFO(LOG_FEATURE_GENERAL, "Channel_SaveInFlashIfNeeded: Channel %i is being saved to flash, state %i", ch, g_channelValues[ch]);
		HAL_FlashVars_SaveChannel(ch, g_channelValues[ch]);
	}
	else {
		//ADDLOG_INFO(LOG_FEATURE_GENERAL, "Channel_SaveInFlashIfNeeded: Channel %i is not saved to flash, state %i", ch, g_channelValues[ch]);
	}
}
// all items on the channel are processed
static void Channel_OnChanged(int ch, int prevValue, int iFlags) {
	int iVal;
	int bOn;

	//bOn = BIT_CHECK(g_channelStates,ch);
	iVal = g_channelValues[ch];
	g_channelValuesFloats[ch] = (float)iVal;
	bOn = iVal > 0;

#if ENABLE_I2C
	I2C_OnChannelChanged(ch, iVal);
#endif

#ifndef OBK_DISABLE_ALL_DRIVERS
	DRV_OnChannelChanged(ch, iVal);
#endif

#if ENABLE_DRIVER_TUYAMCU
	TuyaMCU_OnChannelChanged(ch, iVal);
#endif
#if ENABLE_DRIVER_GIRIERMCU
	GirierMCU_OnChannelChanged(ch, iVal);
#endif
	for (int usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		int pinIndex = registeredPinDetails[usedIndex];
		if (g_cfg.pins.channels[pinIndex] == ch) {
			if (g_cfg.pins.roles[pinIndex] == IOR_Relay || g_cfg.pins.roles[pinIndex] == IOR_LED)
				RAW_SetPinValue(pinIndex, bOn);
			else if (g_cfg.pins.roles[pinIndex] == IOR_Relay_n || g_cfg.pins.roles[pinIndex] == IOR_LED_n)
				RAW_SetPinValue(pinIndex, !bOn);
			else if (g_cfg.pins.roles[pinIndex] == IOR_PWM || g_cfg.pins.roles[pinIndex] == IOR_PWM_ScriptOnly)
				HAL_PIN_PWM_Update(pinIndex, iVal);
			else if (g_cfg.pins.roles[pinIndex] == IOR_PWM_n || g_cfg.pins.roles[pinIndex] == IOR_PWM_ScriptOnly_n)
				HAL_PIN_PWM_Update(pinIndex, 100 - iVal);
		}
	}
#if ENABLE_MQTT
	if ((iFlags & CHANNEL_SET_FLAG_SKIP_MQTT) == 0) {
		if (CHANNEL_ShouldBePublished(ch)) {
			MQTT_ChannelPublish(ch, 0);
		}
	}
#endif
	// Simple event - it just says that there was a change
	EventHandlers_FireEvent(CMD_EVENT_CHANNEL_ONCHANGE, ch);
	// more advanced events - change FROM value TO value
	EventHandlers_ProcessVariableChange_Integer(CMD_EVENT_CHANGE_CHANNEL0 + ch, prevValue, iVal);
	//ADDLOG_ERROR(LOG_FEATURE_GENERAL,"CHANNEL_OnChanged: Channel index %i startChannelValues %i\n\r",ch,g_cfg.startChannelValues[ch]);

	Channel_SaveInFlashIfNeeded(ch);
}

// TODO: think on channels and pins and drivers
void CFG_ApplyChannelStartValues() {
	int i, iValue;
	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);

	for (i = 0; i < CHANNEL_MAX; i++) {
		iValue = g_cfg.startChannelValues[i];
		if (iValue == -1) {
			g_channelValuesFloats[i] = g_channelValues[i] = HAL_FlashVars_GetChannelValue(i);
			//ADDLOG_INFO(LOG_FEATURE_GENERAL, "CFG_ApplyChannelStartValues: Channel %i is being set to REMEMBERED state %i", i, g_channelValues[i]);
		}
		else {
			g_channelValuesFloats[i] = g_channelValues[i] = iValue;
			//ADDLOG_INFO(LOG_FEATURE_GENERAL, "CFG_ApplyChannelStartValues: Channel %i is being set to constant state %i", i, g_channelValues[i]);
		}
	}
}
int ChannelType_GetDecimalPlaces(int type) {
	int pl;

	int div = ChannelType_GetDivider(type);
	int cur = 10;
	for (pl = 0; pl < 6; pl++) {
		if (div < cur)
			return pl + 1;
		cur *= 10;
	}
	return pl;
}
int ChannelType_GetDivider(int type) {
	switch (type)
	{
	case ChType_Humidity_div10:
	case ChType_Temperature_div10:
	case ChType_Voltage_div10:
	case ChType_Power_div10:
	case ChType_Frequency_div10:
	case ChType_ReadOnly_div10:
	case ChType_Current_div10:
		return 10;
	case ChType_Frequency_div100:
	case ChType_Current_div100:
	case ChType_EnergyTotal_kWh_div100:
	case ChType_Voltage_div100:
	case ChType_PowerFactor_div100:
	case ChType_Pressure_div100:
	case ChType_Temperature_div100:
	case ChType_Power_div100:
	case ChType_ReadOnly_div100:
	case ChType_Ph:
		return 100;
	case ChType_PowerFactor_div1000:
	case ChType_EnergyTotal_kWh_div1000:
	case ChType_EnergyExport_kWh_div1000:
	case ChType_EnergyToday_kWh_div1000:
	case ChType_EnergyImport_kWh_div1000:
	case ChType_Current_div1000:
	case ChType_LeakageCurrent_div1000:
	case ChType_ReadOnly_div1000:
	case ChType_Frequency_div1000:
		return 1000;
	case ChType_Temperature_div2:
		return 2;
	}
	return 1;
}
const char *ChannelType_GetUnit(int type) {
	switch (type)
	{
	case ChType_BatteryLevelPercent:
	case ChType_Humidity:
	case ChType_Humidity_div10:
		return "%";
	case ChType_Temperature_div100:
	case ChType_Temperature_div10:
	case ChType_Temperature_div2:
	case ChType_Temperature:
		return "C";
	case ChType_Voltage_div100:
	case ChType_Voltage_div10:
		return "V";
	case ChType_Power:
	case ChType_Power_div10:
	case ChType_Power_div100:
		return "W";
	case ChType_Frequency_div10:
	case ChType_Frequency_div100:
	case ChType_Frequency_div1000:
		return "Hz";
	case ChType_LeakageCurrent_div1000:
	case ChType_Current_div1000:
	case ChType_Current_div100:
	case ChType_Current_div10:
		return "A";
	case ChType_EnergyTotal_kWh_div1000:
	case ChType_EnergyExport_kWh_div1000:
	case ChType_EnergyToday_kWh_div1000:
	case ChType_EnergyTotal_kWh_div100:
	case ChType_EnergyImport_kWh_div1000:
		return "kWh";
	case ChType_PowerFactor_div1000:
	case ChType_PowerFactor_div100:
		return "";
	case ChType_Pressure_div100:
		return "hPa";
	case ChType_ReactivePower:
		return "vAr";
	case ChType_Illuminance:
		return "Lux";
	case ChType_Ph:
		return "Ph";
	case ChType_Orp:
		return "mV";
	case ChType_Tds:
		return "ppm";
	}
	return "";
}
const char *ChannelType_GetTitle(int type) {
	switch (type)
	{
	case ChType_BatteryLevelPercent:
		return "Battery";
	case ChType_Humidity:
	case ChType_Humidity_div10:
		return "Humidity";
	case ChType_Temperature_div100:
	case ChType_Temperature_div10:
	case ChType_Temperature_div2:
	case ChType_Temperature:
		return "Temperature";
	case ChType_Voltage_div100:
	case ChType_Voltage_div10:
		return "Voltage"; 
	case ChType_Power:
	case ChType_Power_div10:
	case ChType_Power_div100:
		return "Power";
	case ChType_Frequency_div10:
	case ChType_Frequency_div100:
	case ChType_Frequency_div1000:
		return "Frequency";
	case ChType_Current_div1000:
	case ChType_Current_div100:
	case ChType_Current_div10:
		return "Current";
	case ChType_LeakageCurrent_div1000:
		return "Leakage"; 
	case ChType_EnergyTotal_kWh_div1000:
	case ChType_EnergyTotal_kWh_div100:
		return "EnergyTotal";
	case ChType_EnergyExport_kWh_div1000:
		return "EnergyExport";
	case ChType_EnergyToday_kWh_div1000:
		return "EnergyToday";
	case ChType_EnergyImport_kWh_div1000:
		return "EnergyImport";
	case ChType_PowerFactor_div1000:
	case ChType_PowerFactor_div100:
		return "PowerFactor";
	case ChType_Pressure_div100:
		return "Pressure";
	case ChType_ReactivePower:
		return "ReactivePower";
	case ChType_Illuminance:
		return "Illuminance";
	case ChType_Ph:
		return "Ph Water Quality";
	case ChType_Orp:
		return "Orp Water Quality";
	case ChType_Tds:
		return "TDS Water Quality";
	case ChType_ReadOnly:
	case ChType_ReadOnly_div10:
	case ChType_ReadOnly_div100:
	case ChType_ReadOnly_div1000:
		return "ReadOnly:";
	}
	return "";
}
float CHANNEL_GetFinalValue(int channel) {
	float dVal;

	dVal = CHANNEL_Get(channel);
	dVal /= ChannelType_GetDivider(CHANNEL_GetType(channel));

	return dVal;
}
float CHANNEL_GetFloat(int ch) {
	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_Get: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return 0;
	}
	return g_channelValuesFloats[ch];
}
int CHANNEL_Get(int ch) {
	if (ch >= SPECIAL_CHANNEL_FLASHVARS_FIRST && ch <= SPECIAL_CHANNEL_FLASHVARS_LAST) {
		return HAL_FlashVars_GetChannelValue(ch - SPECIAL_CHANNEL_FLASHVARS_FIRST);
	}
	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_Get: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return 0;
	}
	return g_channelValues[ch];
}
void CHANNEL_ClearAllChannels() {
	int i;

	for (i = 0; i < CHANNEL_MAX; i++) {
		CHANNEL_Set(i, 0, CHANNEL_SET_FLAG_SILENT);
	}
}

void CHANNEL_Set_FloatPWM(int ch, float fVal, int iFlags) {
	int i;
	float prevValue = g_channelValuesFloats[ch];

	g_channelValues[ch] = (int)fVal;
	g_channelValuesFloats[ch] = fVal;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.channels[i] == ch) {
			if (g_cfg.pins.roles[i] == IOR_PWM || g_cfg.pins.roles[i] == IOR_PWM_ScriptOnly) {
				HAL_PIN_PWM_Update(i, fVal);
			}
			else if (g_cfg.pins.roles[i] == IOR_PWM_n || g_cfg.pins.roles[i] == IOR_PWM_ScriptOnly_n) {
				HAL_PIN_PWM_Update(i, 100.0f - fVal);
			}
		}
	}
	// TODO: support float
	EventHandlers_FireEvent(CMD_EVENT_CHANNEL_ONCHANGE, ch);
	EventHandlers_ProcessVariableChange_Integer(CMD_EVENT_CHANGE_CHANNEL0 + ch, prevValue, fVal);
}
void CHANNEL_SetSmart(int ch, float fVal, int iFlags) {
	if (ch < 0 || ch >= CHANNEL_MAX)
		return;
	int divider = ChannelType_GetDivider(g_cfg.pins.channelTypes[ch]);
	int divided = fVal * divider;
	CHANNEL_Set(ch, divided, iFlags);
}

void CHANNEL_Set_Ex(int ch, int iVal, int iFlags, int ausemovingaverage) {
	int prevValue;
	int bForce;
	int bSilent;
	bForce = iFlags & CHANNEL_SET_FLAG_FORCE;
	bSilent = iFlags & CHANNEL_SET_FLAG_SILENT;

	if (ch >= SPECIAL_CHANNEL_FLASHVARS_FIRST && ch <= SPECIAL_CHANNEL_FLASHVARS_LAST) {
		HAL_FlashVars_SaveChannel(ch - SPECIAL_CHANNEL_FLASHVARS_FIRST, iVal);
		return;
	}
	if (ch < 0 || ch >= CHANNEL_MAX) {
		//if(bMustBeSilent==0) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "%s: Channel index %i is out of range <0,%i)", __func__, ch, CHANNEL_MAX);
		//}
		return;
	}
	prevValue = g_channelValues[ch];
	if (bForce == 0) {
		if (prevValue == iVal) {
			if (bSilent == 0) {
				ADDLOG_INFO(LOG_FEATURE_GENERAL, "%s: No change in channel %i (still set to %i) - ignoring", __func__, ch, prevValue);
			}
			return;
		}
	}
	if (bSilent == 0) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "%s: Channel %i has changed to %i (flags %i)", __func__, ch, iVal, iFlags);
	}
	#ifdef ENABLE_BL_MOVINGAVG
	//ADDLOG_INFO(LOG_FEATURE_GENERAL, "CHANNEL_Set debug channel %i has changed to %i (flags %i)\n\r", ch, iVal, iFlags);
	if (ausemovingaverage) {
		iVal=XJ_MovingAverage_int(prevValue, iVal);
	}
	#endif
	g_channelValues[ch] = iVal;

	Channel_OnChanged(ch, prevValue, iFlags);
}
void CHANNEL_Set(int ch, int iVal, int iFlags) {
	CHANNEL_Set_Ex(ch, iVal, iFlags, 0);
}
char *g_channelPingPongs = 0;

void CHANNEL_AddClamped(int ch, int iDelta, int min, int max, int bWrapInsteadOfClamp) {
	// we want to support special channel indexes, so it's better to use GET/SET interface
	// Special channel indexes are used to access things like dimmer, led colors, etc
	int newVal;;

	if (bWrapInsteadOfClamp == 3) {
		if (g_channelPingPongs) {
			g_channelPingPongs[ch] *= -1;
		}
		return;
	} else if (bWrapInsteadOfClamp == 2) {
		// ping-pong logic
		if (g_channelPingPongs == 0) {
			g_channelPingPongs = (char*)malloc(CHANNEL_MAX);
			memset(g_channelPingPongs, 1, CHANNEL_MAX);
		}
		int prevVal = CHANNEL_Get(ch);
		newVal = prevVal + iDelta * g_channelPingPongs[ch];
		if (prevVal == min && newVal < min) {
			g_channelPingPongs[ch] *= -1;
		}
		else if (prevVal == max && newVal > max) {
			g_channelPingPongs[ch] *= -1;
		}
		newVal = prevVal + iDelta * g_channelPingPongs[ch];
		if (newVal > max) {
			newVal = max;
		}
		else if (newVal < min) {
			newVal = min;
		}
	} else if (bWrapInsteadOfClamp) {
		newVal = CHANNEL_Get(ch) + iDelta;
		if (newVal > max)
			newVal = min;
		if (newVal < min)
			newVal = max;
	}
	else {
		newVal = CHANNEL_Get(ch) + iDelta;
		if (newVal > max)
			newVal = max;
		if (newVal < min)
			newVal = min;
	}

	ADDLOG_INFO(LOG_FEATURE_GENERAL, 
		"CHANNEL_AddClamped channel %i has changed to %i\n\r", ch, newVal);

	CHANNEL_Set(ch, newVal, 0);
}
void CHANNEL_Add(int ch, int iVal) {
#if 0
	int prevValue;
	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_Add: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return;
	}
	prevValue = g_channelValues[ch];
	g_channelValues[ch] = g_channelValues[ch] + iVal;

	ADDLOG_INFO(LOG_FEATURE_GENERAL, "CHANNEL_Add channel %i has changed to %i\n\r", ch, g_channelValues[ch]);

	Channel_OnChanged(ch, prevValue, 0);
#else
	// we want to support special channel indexes, so it's better to use GET/SET interface
	// Special channel indexes are used to access things like dimmer, led colors, etc
	iVal = iVal + CHANNEL_Get(ch);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "CHANNEL_Add channel %i has changed to %i\n\r", ch, iVal);
	CHANNEL_Set(ch, iVal, 0);
#endif
}

int CHANNEL_FindMaxValueForChannel(int ch) {
	int i;
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		// is pin tied to this channel?
		if (g_cfg.pins.channels[i] == ch) {
			// is it PWM?
			if (g_cfg.pins.roles[i] == IOR_PWM || g_cfg.pins.roles[i] == IOR_PWM_ScriptOnly) {
				return 100;
			}
			if (g_cfg.pins.roles[i] == IOR_PWM_n || g_cfg.pins.roles[i] == IOR_PWM_ScriptOnly_n) {
				return 100;
			}
		}
	}
	if (g_cfg.pins.channelTypes[ch] == ChType_Dimmer)
		return 100;
	if (g_cfg.pins.channelTypes[ch] == ChType_Dimmer256)
		return 256;
	if (g_cfg.pins.channelTypes[ch] == ChType_Dimmer1000)
		return 1000;
	return 1;
}
// PWMs are toggled between 0 and 100 (0% and 100% PWM)
// Relays and everything else is toggled between 0 (off) and 1 (on)
void CHANNEL_Toggle(int ch) {
	int prev;

	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_Toggle: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return;
	}
	prev = g_channelValues[ch];
	if (g_channelValues[ch] == 0)
		g_channelValues[ch] = CHANNEL_FindMaxValueForChannel(ch);
	else
		g_channelValues[ch] = 0;

	Channel_OnChanged(ch, prev, 0);
}
int CHANNEL_HasChannelPinWithRoleOrRole(int ch, int iorType, int iorType2) {
	int i;

	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_HasChannelPinWithRole: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return 0;
	}
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.channels[i] == ch) {
			if (g_cfg.pins.roles[i] == iorType)
				return 1;
			else if (g_cfg.pins.roles[i] == iorType2)
				return 1;
		}
	}
	return 0;
}
int CHANNEL_HasChannelPinWithRole(int ch, int iorType) {
	int i;

	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_HasChannelPinWithRole: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return 0;
	}
	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.channels[i] == ch) {
			if (g_cfg.pins.roles[i] == iorType)
				return 1;
		}
	}
	return 0;
}
bool CHANNEL_Check(int ch) {
	if (ch < 0 || ch >= CHANNEL_MAX) {
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "CHANNEL_Check: Channel index %i is out of range <0,%i)\n\r", ch, CHANNEL_MAX);
		return 0;
	}
	if (g_channelValues[ch] > 0)
		return 1;
	return 0;
}

bool CHANNEL_IsInUse(int ch) {
	int i;

	if (g_cfg.pins.channelTypes[ch] != ChType_Default) {
		return true;
	}

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		if (g_cfg.pins.roles[i] != IOR_None) {
			int NofC=PIN_IOR_NofChan(g_cfg.pins.roles[i]);
			if (NofC>=1 && g_cfg.pins.channels[i] == ch) {
				return true;
			}
			if (NofC>=2 && g_cfg.pins.channels2[i] == ch) {
				return true;
			}
		}
	}
#if (ENABLE_DRIVER_DS1820_FULL)
#include "driver/drv_ds1820_full.h"
	return ds18b20_used_channel(ch);
#endif
	return false;
}

// if any pin on the channel is relay say yes
bool CHANNEL_IsPowerRelayChannel(int ch) {
	for (int usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		int pinIndex = registeredPinDetails[usedIndex];
		if (g_cfg.pins.channels[pinIndex] == ch) {
			int role = g_cfg.pins.roles[pinIndex];
			// NOTE: do not include Battery relay
			if (role == IOR_Relay || role == IOR_Relay_n) {
				return true;
			}
#if ENABLE_DRIVER_BRIDGE
			// Also allow toggling Bridge channel
			// https://www.elektroda.com/rtvforum/viewtopic.php?p=20906463#20906463
			if (role == IOR_BridgeForward || role == IOR_BridgeReverse) {
				return true;
			}
#endif // ENABLE_DRIVER_BRIDGE
		}
	}
	return false;
}
// TODO: Again think about channels
bool CHANNEL_ShouldBePublished(int ch) {
	for (int i = 0; i < g_registeredPinCount; i++) {
		int pinIndex = registeredPinDetails[i];
		int role = PIN_GetPinRoleForPinIndex(pinIndex);
		int driverIndex = g_pinIORoleDriver[role];
		if (g_cfg.pins.channels[pinIndex] == ch) {
			if (driverIndex) {
				return DRV_SendRequest(driverIndex, OBKF_ShouldPublish, role);
			} else if (role == IOR_ADC
				|| role == IOR_CHT83XX_DAT || role == IOR_SHT3X_DAT
				|| role == IOR_DigitalInput || role == IOR_DigitalInput_n
				|| IS_PIN_AIR_SENSOR_ROLE(role)
				|| IS_PIN_DHT_ROLE(role)
				|| role == IOR_DigitalInput_NoPup || role == IOR_DigitalInput_NoPup_n) {
				return true;
			}
		} else if (g_cfg.pins.channels2[pinIndex] == ch) {
			if (IS_PIN_DHT_ROLE(role))
				return true;
			// SGP, CHT8305 and SHT3X uses secondary channel for humidity
			if (role == IOR_CHT83XX_DAT || role == IOR_SHT3X_DAT || IS_PIN_AIR_SENSOR_ROLE(role))
				return true;
		}
	}
	if (g_cfg.pins.channelTypes[ch] != ChType_Default) {
		return true;
	}
#ifdef ENABLE_DRIVER_TUYAMCU
	// publish if channel is used by TuyaMCU or Girier (no pin role set), for example door sensor state with power saving V0 protocol
	// Not enabled by default, you have to set OBK_FLAG_TUYAMCU_ALWAYSPUBLISHCHANNELS flag
	if (CFG_HasFlag(OBK_FLAG_TUYAMCU_ALWAYSPUBLISHCHANNELS) && TuyaMCU_IsChannelUsedByTuyaMCU(ch)) {
		return true;
	}
#endif
#ifdef ENABLE_DRIVER_GIRIERMCU
	if (CFG_HasFlag(OBK_FLAG_TUYAMCU_ALWAYSPUBLISHCHANNELS) && GirierMCU_IsChannelUsedByGirierMCU(ch)) {
		return true;
	}
#endif

	if (CFG_HasFlag(OBK_FLAG_MQTT_PUBLISH_ALL_CHANNELS)) {
		return true;
	}
	return false;
}

void PIN_ticks(void* param)
{
	static uint32_t g_time = 0, g_last_time = 0;
	uint32_t t_diff = QUICK_TMR_DURATION;
	int value;

	for (int usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		int pinIndex = registeredPinDetails[usedIndex];
		if (g_counterDeltas[pinIndex]) {
			// TODO: disable interrupts now so it won't get called in meantime?
			CHANNEL_Add(g_cfg.pins.channels[pinIndex], g_counterDeltas[pinIndex]);
			g_counterDeltas[pinIndex] = 0;
		}
	}

#if defined(PLATFORM_BEKEN) || defined(WINDOWS)
	g_time = rtos_get_time();
	t_diff = g_time - g_last_time;
#else
	g_time += QUICK_TMR_DURATION;
#endif
	// cope with wrap
	if (t_diff > 0x4000) {
		t_diff = ((g_time + 0x4000) - (g_last_time + 0x4000));
	}
	g_last_time = g_time;

}

const char* g_channelTypeNames[] = {
	"Default",
	"Error",
	"Temperature",
	"Humidity",
	// most likely will not be used
	// but it's humidity value times 10
	// TuyaMCU sends 225 instead of 22.5%
	"Humidity_div10",
	"Temperature_div10",
	"Toggle",
	// 0-100 range
	"Dimmer",
	"LowMidHigh",
	"TextField",
	"ReadOnly",
	// off (0) and 3 speeds
	"OffLowMidHigh",
	// off (0) and 5 speeds
	"OffLowestLowMidHighHighest",
	// only 5 speeds
	"LowestLowMidHighHighest",
	// like dimmer, but 0-255
	"Dimmer256",
	// like dimmer, but 0-1000
	"Dimmer1000",
	// for TuyaMCU power metering
	//NOTE: not used for BL0937 etc
	"Frequency_div100",
	"Voltage_div10",
	"Power",
	"Current_div100",
	"ActivePower",
	"PowerFactor_div1000",
	"ReactivePower",
	"EnergyTotal_kWh_div1000",
	"EnergyExport_kWh_div1000",
	"EnergyToday_kWh_div1000",
	"Current_div1000",
	"EnergyTotal_kWh_div100",
	"OpenClosed",
	"OpenClosed_Inv",
	"BatteryLevelPercent",
	"OffDimBright",
	"LowMidHighHighest",
	"OffLowMidHighHighest",
	"Custom",
	"Power_div10",
	"ReadOnlyLowMidHigh",
	"SmokePercent",
	"Illuminance",
	"Toggle_Inv",
	"OffOnRemember",
	"Voltage_div100",
	"Temperature_div2",
	"TimerSeconds",
	"Frequency_div10",
	"PowerFactor_div100",
	"Pressure_div100",
	"Temperature_div100",
	"LeakageCurrent_div1000",
	"Power_div100",
	"Motion",
	"ReadOnly_div10",
	"ReadOnly_div100",
	"ReadOnly_div1000",
	"Ph",
	"Orp",
	"Tds",
	"Motion_n",
	"Frequency_div1000",
	"OpenStopClose",
	"Percent",
	"StopUpDown",
	"EnergyImport_kWh_div1000",
	"Enum",
	"ReadOnlyEnum",
	"Current_div10",
	"error",
	"error",
	"error",
	"error",
};
// setChannelType 3 LowMidHigh
int CHANNEL_ParseChannelType(const char* s) {
	int i;
	for (i = 0; i < ChType_Max; i++) {
		if (!stricmp(g_channelTypeNames[i], s)) {
			return i;
		}
	}
	return ChType_Error;
}
static commandResult_t CMD_setButtonHoldRepeat(const void* context, const char* cmd, const char* args, int cmdFlags) {


	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() < 1) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "This command requires 1 argument - timeRepeat - current %i",
			g_cfg.buttonHoldRepeat);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}


	CFG_SetButtonRepeatPressTime(Tokenizer_GetArgInteger(0));

	CFG_Save_IfThereArePendingChanges();

	ADDLOG_INFO(LOG_FEATURE_GENERAL, "Times set, %i. Config autosaved to flash.",
		g_cfg.buttonHoldRepeat
	);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "If something is wrong, you can restore default %i",
		CFG_DEFAULT_BTN_REPEAT);
	return CMD_RES_OK;
}
// SetButtonTimes [ValLongPress] [ValShortPress] [ValRepeat]
// Each value is times 100ms, so: SetButtonTimes 2 1 1 means 200ms long press, 100ms short and 100ms repeat
static commandResult_t CMD_SetButtonTimes(const void* context, const char* cmd, const char* args, int cmdFlags) {


	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() < 3) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "This command requires 3 arguments - timeLong, timeShort, timeRepeat - current %i %i %i",
			g_cfg.buttonLongPress, g_cfg.buttonShortPress, g_cfg.buttonHoldRepeat);
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	CFG_SetButtonLongPressTime(Tokenizer_GetArgInteger(0));

	CFG_SetButtonShortPressTime(Tokenizer_GetArgInteger(1));

	CFG_SetButtonRepeatPressTime(Tokenizer_GetArgInteger(2));

	CFG_Save_IfThereArePendingChanges();

	ADDLOG_INFO(LOG_FEATURE_GENERAL, "Times set, %i %i %i. Config autosaved to flash.",
		g_cfg.buttonLongPress, g_cfg.buttonShortPress, g_cfg.buttonHoldRepeat
	);
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "If something is wrong, you can restore defaults - %i %i %i",
		CFG_DEFAULT_BTN_LONG, CFG_DEFAULT_BTN_SHORT, CFG_DEFAULT_BTN_REPEAT);
	return 0;
}
static commandResult_t CMD_ShowChannelValues(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int i;

	for (i = 0; i < CHANNEL_MAX; i++) {
		if (g_channelValues[i] > 0) {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "Channel %i value is %i", i, g_channelValues[i]);
		}
	}

	return CMD_RES_OK;
}
static commandResult_t CMD_SetChannelType(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int channel;
	const char* type;
	int typeCode;

	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() < 2) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "This command requires 2 arguments");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
	channel = Tokenizer_GetArgInteger(0);
	type = Tokenizer_GetArg(1);

	typeCode = CHANNEL_ParseChannelType(type);
	if (typeCode == ChType_Error) {

		ADDLOG_INFO(LOG_FEATURE_GENERAL, "Channel %i type not set because %s is not a known type", channel, type);
		return CMD_RES_BAD_ARGUMENT;
	}

	CHANNEL_SetType(channel, typeCode);

	ADDLOG_INFO(LOG_FEATURE_GENERAL, "Channel %i type changed to %s", channel, type);
	return CMD_RES_OK;
}
#if ALLOW_SSID2
// setStartupSSIDChannel [-1 or RetainChannelIndex]
static commandResult_t CMD_setStartupSSIDChannel(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() >= 1) {
		int fval = Tokenizer_GetArgInteger(0);
		if ((fval < -1) || (fval >= MAX_RETAIN_CHANNELS - 1)) {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSIDChannel value error: %i Allowed values (-1, 0..%i)", fval, MAX_RETAIN_CHANNELS - 1);
			return CMD_RES_BAD_ARGUMENT;
		}
		g_StartupSSIDRetainChannel = fval;
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSIDChannel changed to %i", g_StartupSSIDRetainChannel);
	}
	else {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSIDChannel is %i", g_StartupSSIDRetainChannel);
	}
	return CMD_RES_OK;
}
// setStartupSSID [0/1]  
// Sets startup SSID - 0=SSID1 1=SSID2 - which SSID will be used after reboot. 
// for this to work, setStartupSSIDChannel and SSID2 must be set
static commandResult_t CMD_setStartupSSID(const void* context, const char* cmd, const char* args, int cmdFlags) {

	Tokenizer_TokenizeString(args, 0);

	int fold = FV_GetStartupSSID_StoredValue(0);
	if (Tokenizer_GetArgsCount() >= 1) {
		int fval = Tokenizer_GetArgInteger(0);
		if ((fval < 0) || (fval >1)) {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSID value error: %i Allowed values (0, 1)", fval);
			return CMD_RES_BAD_ARGUMENT;
		}
		if (g_StartupSSIDRetainChannel<0) {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "Cannot set StartupSSID, StartupSSIDChannel is not set.");
			return CMD_RES_BAD_ARGUMENT;
		}
		if (!(fval==fold)) {
			FV_UpdateStartupSSIDIfChanged_StoredValue(fval);//update flash only when changed
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSID changed to %i", fval);
		} else {
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSID unchanged %i", fval);
		}
	} else {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "StartupSSID is %i", fold);
	}
	return CMD_RES_OK;
}
#endif
/// @brief Computes the Relay and PWM count.
/// @param pwmCount Number of PWM channels.
void PIN_get_Relay_PWM_Count(int* pwmCount, int* dInputCount) {
	int pwmBits;
	if (pwmCount) {
		(*pwmCount) = 0;
	}
	if (dInputCount) {
		(*dInputCount) = 0;
	}

	// if we have two PWMs on single channel, count it once
	pwmBits = 0;

	for (int usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		int pinIndex = registeredPinDetails[usedIndex];
		int role = PIN_GetPinRoleForPinIndex(pinIndex);
		switch (role) {
		case IOR_PWM:
		case IOR_PWM_n:
			// if we have two PWMs on single channel, count it once
			BIT_SET(pwmBits, g_cfg.pins.channels[pinIndex]);
			//(*pwmCount)++;
			break;
		case IOR_PWM_ScriptOnly:
		case IOR_PWM_ScriptOnly_n:
			// DO NOT COUNT SCRIPTONLY PWM HERE!
			// As in title - it's only for scripts. 
			// It should not generate lights!
			break;
		case IOR_DigitalInput:
		case IOR_DigitalInput_n:
		case IOR_DigitalInput_NoPup:
		case IOR_DigitalInput_NoPup_n:
// #if ENABLE_DRIVER_DOORSENSOR
// 		case IOR_DoorSensor:
// 		case IOR_DoorSensor_NoPup:
// 		case IOR_DoorSensor_pd:
// #endif
			if (dInputCount) {
				(*dInputCount)++;
			}
			break;
		default:
			break;
		}
	}
	if (pwmCount) {
		// if we have two PWMs on single channel, count it once
		for (int i = 0; i < 32; i++) {
			if (BIT_CHECK(pwmBits, i)) {
				(*pwmCount)++;
			}
		}
	}
}


int h_isChannelPWM(int tg_ch) {
	int i;
	int role;
	int ch;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		ch = PIN_GetPinChannelForPinIndex(i);
		if (tg_ch != ch)
			continue;
		role = PIN_GetPinRoleForPinIndex(i);
		if (role == IOR_PWM || role == IOR_PWM_n) {
			return true;
		}
		// DO NOT COUNT SCRIPTONLY PWM HERE!
		// As in title - it's only for scripts. 
		// It should not generate lights!
		//if (role == IOR_PWM_ScriptOnly) {
		//	return true;
		//}
	}
	return false;
}
// if any item on the channel is relay
int h_isChannelRelay(int tg_ch) {
	int role;

	for (int usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
		int pinIndex = registeredPinDetails[usedIndex];
		int ch = PIN_GetPinChannelForPinIndex(pinIndex);
		if (tg_ch != ch)
			continue;
		role = PIN_GetPinRoleForPinIndex(pinIndex);
		if (role == IOR_Relay || role == IOR_Relay_n || role == IOR_LED || role == IOR_LED_n) {
			return true;
		}
#if ENABLE_DRIVER_BRIDGE
		if ((role == IOR_BridgeForward) || (role == IOR_BridgeReverse))
		{
			return true;
		}
#endif // ENABLE_DRIVER_BRIDGE
	}
	return false;
}
int h_isChannelDigitalInput(int tg_ch) {
	int i;
	int role;

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		int ch = PIN_GetPinChannelForPinIndex(i);
		if (tg_ch != ch)
			continue;
		role = PIN_GetPinRoleForPinIndex(i);
		if (role == IOR_DigitalInput || role == IOR_DigitalInput_n || role == IOR_DigitalInput_NoPup || role == IOR_DigitalInput_NoPup_n) {
			return true;
		}
	}
	return false;
}
static commandResult_t showgpi(const void* context, const char* cmd, const char* args, int cmdFlags)
{
	int i;
	unsigned int value[2] = { 0, 0 };

	for (i = 0; i < PLATFORM_GPIO_MAX; i++) {
		int val = 0;

		val = HAL_PIN_ReadDigitalInput(i);

		if (i >= 32)
			value[1] |= ((val & 1) << (i - 32));
		else
			value[0] |= ((val & 1) << i);
	}
	ADDLOG_INFO(LOG_FEATURE_GENERAL, "GPIs are 0x%08lX%08lX", value[1], value[0]);
	return CMD_RES_OK;
}

#ifdef ENABLE_BL_MOVINGAVG
static int movingavg_cnt = 0;
static commandResult_t CMD_setMovingAvg(const void* context, const char* cmd,
	const char* args, int cmdFlags) {
	Tokenizer_TokenizeString(args, 0);
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	int fval = Tokenizer_GetArgInteger(0);
	if (fval < 0) {
		ADDLOG_ERROR(LOG_FEATURE_ENERGYMETER, "%s",
			CMD_GetResultString(CMD_RES_BAD_ARGUMENT));
		return CMD_RES_BAD_ARGUMENT;
	}
	ADDLOG_INFO(LOG_FEATURE_ENERGYMETER, "MovingAvg=%i", fval);	
	movingavg_cnt = fval;
	return CMD_RES_OK;
}

float XJ_MovingAverage_float(float aprevvalue, float aactvalue) {
	//if aprevvalue not set, return actual 
	if (aprevvalue <= 0) return aactvalue;
	if (movingavg_cnt <= 1) return aactvalue;
	//if aprevvalue set, calculate simple average value
	float res = (((movingavg_cnt - 1) * aprevvalue + aactvalue) / movingavg_cnt);
	//ADDLOG_DEBUG(LOG_FEATURE_ENERGYMETER, "MovAvg p: %.2f a: %.2f r: %.2f", aprevvalue, aactvalue, res);
	return res;
}

int XJ_MovingAverage_int(int aprevvalue, int aactvalue) {
	//if aprevvalue not set, return actual 
	if (aprevvalue <= 0) return aactvalue;
	if (movingavg_cnt <= 1) return aactvalue;
	//if aprevvalue set, calculate simple average value
	int res = (((movingavg_cnt - 1) * aprevvalue + aactvalue) / movingavg_cnt);
	//ADDLOG_DEBUG(LOG_FEATURE_ENERGYMETER, "MovAvg p: %i a: %i r: %i", aprevvalue, aactvalue, res);
	return res;
}
#endif

void PIN_AddCommands(void)
{
	//cmddetail:{"name":"showgpi","args":"NULL",
	//cmddetail:"descr":"log stat of all GPIs",
	//cmddetail:"fn":"showgpi","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("showgpi", showgpi, NULL);
	//cmddetail:{"name":"setChannelType","args":"[ChannelIndex][TypeString]",
	//cmddetail:"descr":"Sets a custom type for channel. Types are mostly used to determine how to display channel value on GUI",
	//cmddetail:"fn":"CMD_SetChannelType","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setChannelType", CMD_SetChannelType, NULL);
	//cmddetail:{"name":"showChannelValues","args":"",
	//cmddetail:"descr":"log channel values",
	//cmddetail:"fn":"CMD_ShowChannelValues","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("showChannelValues", CMD_ShowChannelValues, NULL);
	//cmddetail:{"name":"setButtonTimes","args":"[ValLongPress][ValShortPress][ValRepeat]",
	//cmddetail:"descr":"Each value is times 100ms, so: SetButtonTimes 2 1 1 means 200ms long press, 100ms short and 100ms repeat",
	//cmddetail:"fn":"CMD_SetButtonTimes","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setButtonTimes", CMD_SetButtonTimes, NULL);
	//cmddetail:{"name":"setButtonHoldRepeat","args":"[Value]",
	//cmddetail:"descr":"Sets just the hold button repeat time, given value is times 100ms, so write 1 for 100ms, 2 for 200ms, etc",
	//cmddetail:"fn":"CMD_setButtonHoldRepeat","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setButtonHoldRepeat", CMD_setButtonHoldRepeat, NULL);
#ifdef ENABLE_BL_MOVINGAVG
	//cmddetail:{"name":"setMovingAvg","args":"MovingAvg",
	//cmddetail:"descr":"Moving average value for power and current. <=1 disable, >=2 count of avg values. The setting is temporary and need to be set at startup.",
	//cmddetail:"fn":"CMD_setMovingAvg","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setMovingAvg", CMD_setMovingAvg, NULL);
#endif
#if ALLOW_SSID2
	//cmddetail:{"name":"setStartupSSIDChannel","args":"[Value]",
	//cmddetail:"descr":"Sets retain channel number to store last used SSID, 0..MAX_RETAIN_CHANNELS-1, -1 to disable. Suggested channel number is 7 (MAXMAX_RETAIN_CHANNELS-5)",
	//cmddetail:"fn":"CMD_setStartupSSIDChannel","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setStartupSSIDChannel", CMD_setStartupSSIDChannel, NULL);
	//cmddetail:{"name":"setStartupSSID","args":"[Value]",
	//cmddetail:"descr":"Sets startup SSID, 0 (SSID0) 1 (SSID1)",
	//cmddetail:"fn":"CMD_setStartupSSID","file":"new_pins.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("setStartupSSID", CMD_setStartupSSID, NULL);
#endif
}
