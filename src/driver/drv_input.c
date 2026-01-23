// Basic Input Driver

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

#define EVENT_CB(ev)   

// loaded from config, they are now configurable
uint32_t BTN_SHORT_MS;
uint32_t BTN_LONG_MS;
uint32_t BTN_HOLD_REPEAT_MS;

uint32_t g_lastValidState;
uint32_t g_driverPins;
uint32_t g_driverIndex;

#define BTN_DEBOUNCE_MS         50

void (*g_doubleClickCallback)(uint32_t pinIndex) = 0;

typedef enum {
	BTN_PRESS_DOWN = 0,
	BTN_PRESS_UP,
	BTN_PRESS_REPEAT,
	BTN_LONG_RRESS_START,
	BTN_LONG_PRESS_HOLD,
	BTN_SINGLE_CLICK, // must be in order
	BTN_DOUBLE_CLICK,
	BTN_TRIPLE_CLICK,
	BTN_QUADRUPLE_CLICK,
	BTN_5X_CLICK,
	BTN_number_of_event,
	BTN_NONE_PRESS
} BTN_PRESS_EVT;

typedef struct pinButton_ {
	uint32_t ticks;
	uint32_t holdRepeatTicks;

	uint32_t event;
	uint32_t repeat;

	uint32_t state;

	uint32_t  debounce_cnt;

	bool active_level;
	bool button_level;
} pinButton_s;

pinButton_s g_buttons[PLATFORM_GPIO_MAX];

void Button_OnPressRelease(uint32_t pinIndex) {
	if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
		ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
		return;
	}
	// fire event - button on pin <index> was released
	EventHandlers_FireEvent(CMD_EVENT_PIN_ONRELEASE, pinIndex);
}

// pinIndex is used to assign a handle that is used to call a function
// that reads the GPIO that we could have gotten a lot easier in the calling
// function. So this is what we will do.

// called from quick tick only
static uint32_t PIN_Input_Handler(uint32_t pinIndex, uint32_t pinRole, pinButton_s* button, uint32_t timeSinceLast) {
	/*-----------------State machine-------------------*/
	switch (button->state) {
	case 0:
		if (button->button_level == button->active_level) {  //start press down
			button->event = BTN_PRESS_DOWN;
			EVENT_CB(BTN_PRESS_DOWN);
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "%i Button_OnInitialPressDown", pinIndex);

			if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
				ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
			} else {
				EventHandlers_FireEvent(CMD_EVENT_PIN_ONPRESS, pinIndex);

				// so-called SetOption13 - instant reaction to touch instead of waiting for release
				if (CFG_HasFlag(OBK_FLAG_BTN_INSTANTTOUCH)) {
					if (pinRole == IOR_Button_ToggleAll || pinRole == IOR_Button_ToggleAll_n)
					{
						CHANNEL_DoSpecialToggleAll();
					} else if (pinRole == IOR_Button_ScriptOnly || pinRole == IOR_Button_ScriptOnly_n)
					{
						return;
					} else {
						CHANNEL_Toggle(g_cfg.pins.channels[pinIndex]);
					}
				}
			}
			button->ticks = 0;
			button->repeat = 1;
			button->state = 1;
		}
		else {
			button->event = BTN_NONE_PRESS;
		}
		break;

	case 1:
		if (button->button_level != button->active_level) { //released press up
			button->event = BTN_PRESS_UP;
			EVENT_CB(BTN_PRESS_UP);
			Button_OnPressRelease(pinIndex);
			button->ticks = 0;
			button->state = 2;

		}
		else if (button->ticks > BTN_LONG_MS) {
			button->event = BTN_LONG_RRESS_START;
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "%i Button_OnLongPressHoldStart", pinIndex);
			if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
				ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
			} else {
				// fire event - button on pin <index> was held
				EventHandlers_FireEvent(CMD_EVENT_PIN_ONHOLDSTART, pinIndex);
			}
			EVENT_CB(BTN_LONG_RRESS_START);
			button->state = 5;
		}
		break;

	case 2:
		if ((button->button_level == button->active_level) && (button->repeat < 5)) { //press down again
			button->event = BTN_PRESS_DOWN;
			EVENT_CB(BTN_PRESS_DOWN);
			button->repeat++;
			EVENT_CB(BTN_PRESS_REPEAT); // repeat hit
			button->ticks = 0;
			button->state = 3;
		} else if ((button->ticks > BTN_SHORT_MS) || (button->repeat == 5)) { //released timeout
			ADDLOG_INFO(LOG_FEATURE_GENERAL, "%i key press %ix", pinIndex, button->repeat);
			button->event = BTN_SINGLE_CLICK + button->repeat - 1;
			if (button->repeat == 1) { // do event_cb before child lock
				EVENT_CB(BTN_SINGLE_CLICK);
				if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
					ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
				} else {
					// fire event - button on pin <index> was clicked
					EventHandlers_FireEvent(button->event, pinIndex);
					// so-called SetOption13 - instant reaction to touch instead of waiting for release
					// first click toggles FIRST CHANNEL linked to this button
					if (!CFG_HasFlag(OBK_FLAG_BTN_INSTANTTOUCH)) {
						if (pinRole == IOR_Button_ToggleAll || pinRole == IOR_Button_ToggleAll_n) {
							CHANNEL_DoSpecialToggleAll();
						} else if (pinRole == IOR_Button_ScriptOnly || pinRole == IOR_Button_ScriptOnly_n) {
							// do nothing
						} else {
							CHANNEL_Toggle(g_cfg.pins.channels[pinIndex]);
						}
					}
				}
			} else {
				if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) { // do child lock first
					ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
				} else {
					if (button->repeat == 2) {
						if (pinRole == IOR_Button_ToggleAll || pinRole == IOR_Button_ToggleAll_n) {
							CHANNEL_DoSpecialToggleAll();
						} else {
							// fire event - button on pin <index> was dbclicked
							EventHandlers_FireEvent(button->event, pinIndex);

							if (pinRole == IOR_Button || pinRole == IOR_Button_n) {
								// double click toggles SECOND CHANNEL linked to this button
								CHANNEL_Toggle(g_cfg.pins.channels2[pinIndex]);
							}
							if (g_doubleClickCallback != 0) {
								g_doubleClickCallback(pinIndex);
							}
						}
					} else {
						EventHandlers_FireEvent(button->event, pinIndex);
					}
				}
			}
			button->state = 0;
		}
		break;

	case 3:
		if (button->button_level != button->active_level) { //released press up
			button->event = BTN_PRESS_UP;
			EVENT_CB(BTN_PRESS_UP);
			Button_OnPressRelease(pinIndex);
			if (button->ticks < BTN_SHORT_MS) {
				button->ticks = 0;
				button->state = 2; //repeat press
			}
			else {
				button->state = 0;
			}
		}
		break;

	case 5:
		if (button->button_level == button->active_level) {
			//continue hold trigger
			button->event = BTN_LONG_PRESS_HOLD;
			button->holdRepeatTicks += timeSinceLast;
			if (button->holdRepeatTicks > BTN_HOLD_REPEAT_MS) {
				ADDLOG_INFO(LOG_FEATURE_GENERAL, "%i Button_OnLongPressHold\r\n", pinIndex);
				if (CFG_HasFlag(OBK_FLAG_BUTTON_DISABLE_ALL)) {
					ADDLOG_INFO(LOG_FEATURE_GENERAL, "Child lock!");
				} else {
					// fire event - button on pin <index> was held
					EventHandlers_FireEvent(CMD_EVENT_PIN_ONHOLD, pinIndex);
				}
				button->holdRepeatTicks = 0;
			}
			EVENT_CB(BTN_LONG_PRESS_HOLD);
		}
		else { //releasd
			button->event = BTN_PRESS_UP;
			EVENT_CB(BTN_PRESS_UP);
			Button_OnPressRelease(pinIndex);
			button->state = 0; //reset
		}
		break;
	}
}

uint32_t g_counterDeltas[PLATFORM_GPIO_MAX];
void PIN_InterruptHandler(int gpio) {
	g_counterDeltas[gpio]++;
}

// basic input quick tick timer function
void Input_quickTick(uint32_t timeSinceLast) {
	if (!g_driverPins || !g_enable_pins)
		return;

	uint32_t usedIndex;
	uint32_t driverPins = g_driverPins;
	for (usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		static uint32_t g_times[PLATFORM_GPIO_MAX];
		static uint32_t g_times2[PLATFORM_GPIO_MAX];

		uint32_t debounceMS = CFG_HasFlag(OBK_FLAG_BTN_INSTANTTOUCH) ? 100 : 250;
		uint32_t pinRole = PIN_GetPinRoleForPinIndex(pinIndex);
		bool pinValue = HAL_PIN_ReadDigitalInput(pinIndex);
		switch (pinRole) {
		case IOR_Counter_f:
		case IOR_Counter_r:
			if (g_counterDeltas[pinIndex]) {
				// TODO: disable interrupts now so it won't get called in meantime?
				CHANNEL_Add(PIN_GetPinChannelForPinIndex(pinIndex), g_counterDeltas[pinIndex]);
				g_counterDeltas[pinIndex] = 0;
			}
			break;

		case IOR_Button_n:
		case IOR_Button_ToggleAll_n:
		case IOR_Button_ScriptOnly_n:
		case IOR_SmartButtonForLEDs_n:
			pinValue = !pinValue;
		case IOR_Button:
		case IOR_Button_ToggleAll:
		case IOR_Button_ScriptOnly:
		case IOR_SmartButtonForLEDs:
			//ticks counter working..
			if (g_buttons[pinIndex].state)
				g_buttons[pinIndex].ticks += timeSinceLast;

			/*------------button debounce handle---------------*/
			if (pinValue != g_buttons[pinIndex].button_level) {
				if (g_buttons[pinIndex].debounce_cnt >= BTN_DEBOUNCE_MS) {
					g_buttons[pinIndex].button_level = pinValue;
					g_buttons[pinIndex].debounce_cnt = 0;
				} else {
					g_buttons[pinIndex].debounce_cnt += timeSinceLast;
				}
			} else {
				g_buttons[pinIndex].debounce_cnt = 0;
			}

			PIN_Input_Handler(pinIndex, pinRole, &g_buttons[pinIndex], timeSinceLast);
			break;
		}
		// clear processed pin and exit if no more left to process
		BIT_CLEAR(driverPins, pinIndex);
	}
}

static uint32_t Input_noOfChannels(uint32_t pinRole) {
	switch (pinRole) {
	case IOR_Button:
	case IOR_Button_n:
		return 2;
	
	case IOR_Button_ToggleAll:
	case IOR_Button_ToggleAll_n:
		return 0;

	default:
		return 1;
	}
}

static bool Input_activatePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_Button_n:
	case IOR_Button_ToggleAll_n:
	case IOR_Button_ScriptOnly_n:
	case IOR_SmartButtonForLEDs_n:
		setGPIActive(pinIndex, 1, 1);
		HAL_PIN_Setup_Input_Pullup(pinIndex);
		g_buttons[pinIndex].event = BTN_NONE_PRESS;
		g_buttons[pinIndex].button_level = false;
		break;

	case IOR_Button:
	case IOR_Button_ToggleAll:
	case IOR_Button_ScriptOnly:
	case IOR_SmartButtonForLEDs:
		setGPIActive(pinIndex, 1, 0);
		HAL_PIN_Setup_Input_Pullup(pinIndex);
		g_buttons[pinIndex].event = BTN_NONE_PRESS;
		g_buttons[pinIndex].button_level = true;
		break;

	case IOR_Counter_f:
		HAL_PIN_Setup_Input_Pullup(pinIndex);
		HAL_AttachInterrupt(pinIndex, INTERRUPT_FALLING, PIN_InterruptHandler);
		break;

	case IOR_Counter_r:
		HAL_PIN_Setup_Input_Pullup(pinIndex);
		HAL_AttachInterrupt(pinIndex, INTERRUPT_RISING, PIN_InterruptHandler);
		break;

	default:
		return false;
	}
	BIT_SET(g_driverPins, pinIndex);
	return true;
}

static void Input_ReleasePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_Counter_f:
	case IOR_Counter_r:
		HAL_DetachInterrupt(pinIndex);
		break;
	case IOR_Button:
	case IOR_Button_ToggleAll:
	case IOR_Button_ScriptOnly:
	case IOR_SmartButtonForLEDs:
	case IOR_Button_n:
	case IOR_Button_ToggleAll_n:
	case IOR_Button_ScriptOnly_n:
	case IOR_SmartButtonForLEDs_n:
		setGPIActive(pinIndex, 0, 0);
		memset(&g_buttons[pinIndex], 0, sizeof(pinButton_s));
		break;
	}
	BIT_CLEAR(g_driverPins, pinIndex);
}

static void Input_StopDriver() {
	for (uint32_t usedIndex = 0; g_driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(g_driverPins, pinIndex))
			continue;
		Input_ReleasePin(pinIndex);
	}
}

static void Input_init() {
	BTN_SHORT_MS = (g_cfg.buttonShortPress * 100);
	BTN_LONG_MS = (g_cfg.buttonLongPress * 100);
	BTN_HOLD_REPEAT_MS = (g_cfg.buttonHoldRepeat * 100);
	ADDLOGF_TIMING("%i - %s", xTaskGetTickCount(), __func__);
}

// framework request function
uint32_t Input_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		g_driverIndex = PIN_pinIORoleDriver()[IOR_Button] = PIN_pinIORoleDriver()[IOR_Button_n] \
		              = PIN_pinIORoleDriver()[IOR_Button_ToggleAll] = PIN_pinIORoleDriver()[IOR_Button_ToggleAll_n] \
		              = PIN_pinIORoleDriver()[IOR_Button_ScriptOnly] = PIN_pinIORoleDriver()[IOR_Button_ScriptOnly_n] \
		              = PIN_pinIORoleDriver()[IOR_SmartButtonForLEDs] = PIN_pinIORoleDriver()[IOR_SmartButtonForLEDs_n] \
		              = PIN_pinIORoleDriver()[IOR_Counter_f] = PIN_pinIORoleDriver()[IOR_Counter_r] \
		              = arg;
		ADDLOG_DEBUG(LOG_FEATURE_DRV, "%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return Input_activatePin(arg);

	case OBKF_ReleasePin:
		Input_ReleasePin(arg);
		break;

	case OBKF_Stop:
		Input_StopDriver();
		break;
		
	case OBKF_ShouldPublish:
		return false;

	case OBKF_NoOfChannels:
		return Input_noOfChannels(arg);

	case OBKF_Init:
		Input_init();
		break;

	default:
		break;
	}

	return true;
}

void Input_SetGenericDoubleClickCallback(void (*cb)(uint32_t pinIndex)) {
	g_doubleClickCallback = cb;
}

bool Input_isButton(uint32_t channelIndex) {
	if (!g_driverPins)
		return false;
	
	uint32_t driverPins = g_driverPins;
	for (uint32_t usedIndex = 0; usedIndex < g_registeredPinCount; usedIndex++) {
	uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
	if (!BIT_CHECK(driverPins, pinIndex))
		continue;
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_Button:
	case IOR_Button_n:
		return true;
	}
	if (!BIT_CLEAR(driverPins, pinIndex))
		break;
	}
	return false;
}
