
//
// Generic TuyaMCU information
//
/*
There are two versions of TuyaMCU that I am aware of.
TuyaMCU version 3, the one that is supported by Tasmota and documented here:

TuyaMCU version 0, aka low power protocol, documented here:
(Tuya IoT Development PlatformProduct DevelopmentLow-Code Development (MCU)Wi-Fi for Low-PowerSerial Port Protocol)
https://developer.tuya.com/en/docs/iot/tuyacloudlowpoweruniversalserialaccessprotocol?id=K95afs9h4tjjh
*/

#include "../obk_config.h"
#if ENABLE_DRIVER_TUYAMCU

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
#include "../quicktick.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../hal/hal_wifi.h"
#include "../mqtt/new_mqtt.h"
#include "drv_uart.h"
#include "drv_public.h"
#include <time.h>
#include "drv_deviceclock.h"
#include "../rgb2hsv.h"

#define LOG_FEATURE LOG_FEATURE_TUYAMCU

#define TUYA_CMD_HEARTBEAT     0x00
#define TUYA_CMD_QUERY_PRODUCT 0x01
#define TUYA_CMD_MCU_CONF      0x02
#define TUYA_CMD_WIFI_STATE    0x03
#define TUYA_CMD_WIFI_RESET    0x04
#define TUYA_CMD_WIFI_SELECT   0x05
#define TUYA_CMD_SET_DP        0x06
#define TUYA_CMD_STATE         0x07
#define TUYA_CMD_QUERY_STATE   0x08
#define TUYA_CMD_SET_TIME      0x1C
#define TUYA_CMD_WEATHERDATA   0x21
#define TUYA_CMD_SET_RSSI      0x24
#define TUYA_CMD_NETWORK_STATUS 0x2B
#define TUYA_CMD_REPORT_STATUS_RECORD_TYPE		0x34 

#define TUYA_NETWORK_STATUS_SMART_CONNECT_SETUP 0x00
#define TUYA_NETWORK_STATUS_AP_MODE             0x01
#define TUYA_NETWORK_STATUS_NOT_CONNECTED       0x02
#define TUYA_NETWORK_STATUS_CONNECTED_TO_ROUTER 0x03
#define TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD  0x04
#define TUYA_NETWORK_STATUS_LOW_POWER_MODE      0x05

#define OBKTM_FLAG_DPCACHE 1
#define OBKTM_FLAG_NOREAD 2

//=============================================================================
//dp data point type
//=============================================================================
typedef enum {
	DP_TYPE_RAW,
	DP_TYPE_BOOL,
	DP_TYPE_VALUE,
	DP_TYPE_STRING,
	DP_TYPE_ENUM,
	DP_TYPE_BITMAP
} tuyaDP_Type_t;

typedef enum {
	DU_RESP_SUCCESS,
	DU_RESP_STRANDED,
	DU_RESP_FAILURE,
	DU_RESP_INVALID
} tuyaDU_Resp_t;

// Subtypes of raw - added for OpenBeken - not Tuya standard
#define         DP_TYPE_RAW_DDS238Packet        200
#define			DP_TYPE_RAW_TAC2121C_VCP		201
#define			DP_TYPE_RAW_TAC2121C_YESTERDAY	202
#define			DP_TYPE_RAW_TAC2121C_LASTMONTH	203
#define			DP_TYPE_PUBLISH_TO_MQTT			204
#define			DP_TYPE_RAW_VCPPfF				205
#define			DP_TYPE_RAW_V2C3P3				206

const char* TuyaMCU_getDataTypeString(tuyaDP_Type_t dpType) {
	switch (dpType) {
	case DP_TYPE_RAW:
		return "raw";
	case DP_TYPE_BOOL:
		return "bool";
	case DP_TYPE_VALUE:
		return "val";
	case DP_TYPE_STRING:
		return "str";
	case DP_TYPE_ENUM:
		return "enum";
	case DP_TYPE_BITMAP:
		return "bitmap";
	default:
		return "error";
	}
}

typedef struct tuyaDP_s {
	// internal Tuya variable index
	byte dpId;
	// data point type (one of the DP_TYPE_xxx defines)
	tuyaDP_Type_t dpType;
	// store last value to avoid sending it again
	int prevValue;
	struct tuyaDP_s* next;
} tuyaDP_t;

tuyaDP_t* g_tuyaDPs = 0;

// serial baud rate used to communicate with the TuyaMCU
// common baud rates are 9600 bit/s and 115200 bit/s
const uint32_t g_baudRate = 9600;

bool self_processing_mode = true;

// dynamic will increase based on need
// for V3 door sensor the largest packet is 62 bytes
#define TUYAMCU_BUFFER_SIZE		128

#define MIN_TUYAMCU_PACKET_SIZE (2+1+1+2+1)

uint32_t g_driverIndex;
uint32_t g_driverPins;


byte *g_tuyaMCUTXBuffer;
uint32_t g_tuyaMCUTXBufferSize = TUYAMCU_BUFFER_SIZE;
byte *g_tuyaMCURXBuffer;
uint32_t g_tuyaMCURXBufferSize = TUYAMCU_BUFFER_SIZE;
char *g_productinfo;

/* Tuya communication status logic details */
// Heartbeat status
bool g_heartbeat_valid;
// Query production information
bool g_product_information_valid;
// MCU configuration status
bool g_mcuconfig_valid;
// Inform on wifi status
bool g_wifi_state;

uint32_t g_version;

/* Tuya command wait logic details */
// Last command sent to TuyaMCU incremented by one
uint32_t TuyaMCU_waitingToHearBack;
// Default value to wait to hear back
#define TUYAMCU_HEARBACK_TIMEOUT 3
// Number of seconds we wait till giving up
uint32_t TuyaMCU_stillWaitingToHearBack = TUYAMCU_HEARBACK_TIMEOUT;
// Reset command wait logic
static void TuyaMCU_waitingToHearBackReset() {
	TuyaMCU_waitingToHearBack = 0;
	TuyaMCU_stillWaitingToHearBack = TUYAMCU_HEARBACK_TIMEOUT;
}

/* MQTT queued logic counters */
// Number of MQTT items queued
uint32_t TuyaMCU_queuedMQTT;
// Timeout in seconds from start before failing MQTT queued
// Make sure Tuya will not turn off module before this time
uint32_t TuyaMCU_MQTTConnectTimeout = 60;

/* Tuya ACK delay logic counters */
// Delay before sending the last data unit acknowledgement
uint32_t TuyaMCU_ackDelay;
// Acknowledgement delay left
uint32_t TuyaMCU_ackDelayLeft;

static tuyaDP_t* TuyaMCU_findDefForID(int dpId) {
	tuyaDP_t* curDP = g_tuyaDPs;
	while (curDP) {
		if (curDP->dpId == dpId)
			return curDP;
		curDP = curDP->next;
	}
	return 0;
}

static tuyaDP_t* TuyaMCU_saveDpId(int dpId, int dpType, int iVal) {
	tuyaDP_t* curDP = TuyaMCU_findDefForID(dpId);
	if (!curDP) {
		curDP = (tuyaDP_t*)malloc(sizeof(tuyaDP_t));
		curDP->next = g_tuyaDPs;
		g_tuyaDPs = curDP;
	}
	curDP->dpId = dpId;
	curDP->dpType = dpType;
	curDP->prevValue = iVal;
	return curDP;
}

// header version command lenght data checksum
// 55AA     00      00      0000   xx   00

// talks to TuyaMCU, puts what they said in a buffer and returns
// how much they said
static uint32_t TuyaMCU_listenToTuya() {
	uint32_t howMuchTheyNeedToSay = MIN_TUYAMCU_PACKET_SIZE;
	uint32_t howMuchTheySaid = UART_GetDataSize();
	uint32_t whatWeDidnotWantToHear = 0;
	uint32_t a, b;

	if (howMuchTheySaid < MIN_TUYAMCU_PACKET_SIZE)
		return 0;

	// skip garbage data (should not happen)
	while (howMuchTheySaid >= MIN_TUYAMCU_PACKET_SIZE) {
		a = UART_GetByte(0);
		b = UART_GetByte(1);
		if (a == 0x55 && b == 0xAA) // we have a header
			break;
		
		UART_ConsumeBytes(1);
		whatWeDidnotWantToHear++;
		howMuchTheySaid--;
	}
	if (whatWeDidnotWantToHear > 0) {
		ADDLOGF_WARN("Consumed %i unwanted non-header byte in Tuya MCU buffer\n", whatWeDidnotWantToHear);
		if (howMuchTheySaid < MIN_TUYAMCU_PACKET_SIZE)
			return 0;
	}

	howMuchTheyNeedToSay += UART_GetByte(4) >> 8 | UART_GetByte(5);
	if (howMuchTheySaid < howMuchTheyNeedToSay) {
		ADDLOGF_DEBUG("Still need %i bytes to complete message", howMuchTheyNeedToSay - howMuchTheySaid);
		return 0;
	}
	// can packet fit into the buffer?
	if (g_tuyaMCURXBufferSize < howMuchTheyNeedToSay) {
		g_tuyaMCURXBuffer = realloc(g_tuyaMCURXBuffer, howMuchTheyNeedToSay);
		g_tuyaMCURXBufferSize = howMuchTheyNeedToSay;
	}

	uint32_t checkCheckSum = 0;
	for (uint32_t i = 0; i < howMuchTheyNeedToSay - 1; i++) {
		g_tuyaMCURXBuffer[i] = UART_GetByte(i);
		checkCheckSum += g_tuyaMCURXBuffer[i];
	}
	uint32_t sentCheckSum = UART_GetByte(howMuchTheyNeedToSay - 1);
	// consume whole packet (but don't touch next one, if any)
	UART_ConsumeBytes(howMuchTheyNeedToSay);
	checkCheckSum &= 0x000000FF;
	if (checkCheckSum != sentCheckSum) {
		ADDLOGF_ERROR("Consumed %i bytes bad checksum, expected %i and got checksum %i",
			howMuchTheyNeedToSay, sentCheckSum, checkCheckSum);
		
		for (uint32_t i = 0; i < howMuchTheyNeedToSay - 4; i += 4)
			ADDLOGF_ERROR("%x %x %x %x", g_tuyaMCURXBuffer[i], g_tuyaMCURXBuffer[i+1], g_tuyaMCURXBuffer[i+2], g_tuyaMCURXBuffer[i+3]);
		TuyaMCU_waitingToHearBackReset();
		return 0;
	}

	return howMuchTheyNeedToSay;
}

// append header, len, everything, checksum
static void TuyaMCU_talkToTuya(byte cmdType, byte* data, int payload_len) {
	int i;
	
	byte check_sum = (0xFF + cmdType + (payload_len >> 8) + (payload_len & 0xFF));

	UART_SendByte(0x55);
	UART_SendByte(0xAA);
	UART_SendByte(0x00);         // version 00
	UART_SendByte(cmdType);         // version 00
	UART_SendByte(payload_len >> 8);      // following data length (Hi)
	UART_SendByte(payload_len & 0xFF);    // following data length (Lo)
	for (i = 0; i < payload_len; i++) {
		byte b = data[i];
		check_sum += b;
		UART_SendByte(b);
	}
	UART_SendByte(check_sum);
	TuyaMCU_waitingToHearBack = cmdType + 1;
	ADDLOGF_DEBUG("%s - Sent command %i, with payload len %i",
		__func__, cmdType, payload_len);
	if (payload_len>=2)
		ADDLOGF_DEBUG("%s - Bytes 0x%X 0x%X", __func__, data[0], data[1]);

}
static void TuyaMCU_ParseQueryProductInformation(const byte* prodInfo, int prodInfoLen) {
	g_productinfo = (char*)malloc(prodInfoLen);
	if (g_productinfo) {
		memcpy(g_productinfo, prodInfo, prodInfoLen - 1);
		g_productinfo[prodInfoLen - 1] = 0;
		ADDLOGF_DEBUG("ParseQueryProductInformation: %i bytes, received %s",
			prodInfoLen, g_productinfo);
	} else {
		ADDLOGF_ERROR("%s - unable to allocate space for product info %i bytes", prodInfoLen);
	}
}

static OBK_Publish_Result TuyaMCU_PublishDPToMQTT(int dpId, tuyaDP_Type_t dataType, int sectorLen, const byte *data) {
#if ENABLE_MQTT
	char sName[32];
	int32_t iVal;

	switch (dataType) {
	case DP_TYPE_BOOL:
	case DP_TYPE_ENUM:
		iVal = data[0];
		if (MQTT_IsReady()) {
			sprintf(sName, "tm/%s/%i", TuyaMCU_getDataTypeString(dataType), dpId);
			return MQTT_PublishMain_StringInt(sName, iVal, 0);
		} else 	{
			char sValue[4];   // channel value as a string
			sprintf(sValue, "%i", (int)iVal);
			sprintf(sName, "tm/%s/%i/get", TuyaMCU_getDataTypeString(dataType), dpId);
			MQTT_QueuePublish(CFG_GetMQTTClientId(), sName, sValue, 0); // queue the publishing
			TuyaMCU_queuedMQTT++;
			return OBK_PUBLISH_OK;
		}
	default:
		return OBK_PUBLISH_WAS_NOT_REQUIRED;
	}
#endif
}
static tuyaDU_Resp_t TuyaMCU_parseStateMessage(const byte* data, int len) {
	tuyaDP_t* mapping;
	uint32_t dpId;
	tuyaDP_Type_t dataType;
	uint32_t sectorLen;
	uint32_t iVal = 0;

	uint32_t ofs = 0;

	OBK_Publish_Result ret = OBK_PUBLISH_OK;

	while (ofs + 4 < len) {
		dpId = data[ofs];
		dataType = data[ofs + 1];
		sectorLen = data[ofs + 2] << 8 | data[ofs + 3];
		ADDLOGF_DEBUG("ParseState: id %i type %i-%s len %i, first byte %i",
			dpId, dataType, TuyaMCU_getDataTypeString(dataType), sectorLen, data[ofs + 3 + sectorLen]);
		switch (dataType) {
		case DP_TYPE_BOOL:
		case DP_TYPE_ENUM:
			if (sectorLen == 1)
				iVal = data[ofs + 4];
			else
				return DU_RESP_INVALID;
			break;
		default:
			return DU_RESP_INVALID;
		}

		uint32_t queuedMQTT = TuyaMCU_queuedMQTT;
		mapping = TuyaMCU_findDefForID(dpId);
		if (!mapping) {
			mapping = TuyaMCU_saveDpId(dpId, dataType, iVal);
			ret = TuyaMCU_PublishDPToMQTT(dpId, dataType, sectorLen, data + ofs + 4);
		} else if (mapping->prevValue != iVal) {
			mapping->prevValue = iVal;
			ret = TuyaMCU_PublishDPToMQTT(dpId, dataType, sectorLen, data + ofs + 4);
		}
		if (queuedMQTT != TuyaMCU_queuedMQTT)
			return DU_RESP_STRANDED;
			
		if (OBK_PUBLISH_OK != ret)
			return DU_RESP_FAILURE;
		ofs += (4 + sectorLen);
	}

	if (TuyaMCU_ackDelay) {
		// we wiil reset the delay no matter what, but we
		// can only have one stranded at a time
		if (!TuyaMCU_ackDelayLeft) {
			TuyaMCU_ackDelayLeft = TuyaMCU_ackDelay;
			return DU_RESP_STRANDED;
		} else {
			TuyaMCU_ackDelayLeft = TuyaMCU_ackDelay;
			return DU_RESP_SUCCESS;
		}
	}
	
	return DU_RESP_SUCCESS;
}
static void TuyaMCU_sendDataUnitResponse(tuyaDU_Resp_t result) {
	byte reply[2] = { 0x0B, 0x00 };
	reply[1] = result;
	TuyaMCU_talkToTuya(TUYA_CMD_REPORT_STATUS_RECORD_TYPE, reply, 2);
}
static void TuyaMCU_parseReportStatusType(const byte *value, int len) {
	tuyaDU_Resp_t ret;
	uint32_t subcommand = value[0];
	ADDLOGF_DEBUG("command %i, subcommand %i\n", TUYA_CMD_REPORT_STATUS_RECORD_TYPE, subcommand);
	switch (subcommand) {
	case 0x0B:
		ret = TuyaMCU_parseStateMessage(value + 9, len - 9);
		TuyaMCU_sendDataUnitResponse(ret);
		return;

	default:
		// Unknown subcommand, ignore
		return;
	}
}
static void TuyaMCU_tuyaSaidWhat(int howMuchTheySaid) {
	uint32_t whatWeHeard = g_tuyaMCURXBuffer[3];

	g_version = g_tuyaMCURXBuffer[2];
	if (TuyaMCU_waitingToHearBack && TuyaMCU_waitingToHearBack - 1 != whatWeHeard)
		ADDLOGF_WARN("%s - Rec'd command response (%i), but sent command %i", __func__, whatWeHeard, TuyaMCU_waitingToHearBack - 1);
	TuyaMCU_waitingToHearBackReset();
	ADDLOGF_TIMING("%i - %s - ProcessIncoming[v=%i]: cmd %i, len %i", xTaskGetTickCount(), __func__, g_version, whatWeHeard, howMuchTheySaid);
	switch (whatWeHeard) {
	case TUYA_CMD_HEARTBEAT:
		g_heartbeat_valid = true;
		break;
	case TUYA_CMD_QUERY_PRODUCT:
		TuyaMCU_ParseQueryProductInformation(g_tuyaMCURXBuffer + 6, howMuchTheySaid - 6);
		g_product_information_valid = true;
		break;
	case TUYA_CMD_MCU_CONF:
		g_mcuconfig_valid = true;
		if (howMuchTheySaid == MIN_TUYAMCU_PACKET_SIZE) {
			self_processing_mode = true;
		} else if (howMuchTheySaid == MIN_TUYAMCU_PACKET_SIZE + 2) {
			self_processing_mode = false;
			ADDLOGF_INFO("IMPORTANT!!! mcu conf pins: %i %i", (int)(g_tuyaMCURXBuffer[6]), (int)(g_tuyaMCURXBuffer[7]));
		}
		ADDLOGF_INFO("ProcessIncoming: TUYA_CMD_MCU_CONF, TODO!");
		break;
	case TUYA_CMD_WIFI_STATE:
		g_wifi_state = true;
		ADDLOGF_DEBUG("%s - rec'd wifi state response", __func__);
		break;
	case TUYA_CMD_WIFI_RESET:
		ADDLOGF_INFO("ProcessIncoming: 0x04 replying");
		// added for https://www.elektroda.com/rtvforum/viewtopic.php?p=21095905#21095905
		TuyaMCU_talkToTuya(0x04, 0, 0);
		break;
	case TUYA_CMD_WIFI_SELECT:
		// TUYA_CMD_WIFI_SELECT
		// it should have 1 payload byte, AP mode or EZ mode, but does it make difference for us?
		g_openAP = 1;
	case TUYA_CMD_REPORT_STATUS_RECORD_TYPE:
		TuyaMCU_parseReportStatusType(g_tuyaMCURXBuffer + 6, howMuchTheySaid - 6);
		break;
	default:
		ADDLOGF_INFO("ProcessIncoming: unhandled type %i", whatWeHeard);
		break;
	}
}
commandResult_t Cmd_TuyaMCU_SetBatteryAckDelay(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int delay;

	Tokenizer_TokenizeString(args, 0);
	Tokenizer_CheckArgsCountAndPrintWarning(args, 1);

	delay = Tokenizer_GetArgInteger(0);

	if (!Tokenizer_IsArgInteger(0)) {
		ADDLOGF_INFO("SetBatteryAckDelay: requires 1 argument [delay in seconds]\n");
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	if (delay < 0) {
		ADDLOGF_INFO("SetBatteryAckDelay: delay must be positive\n");
		return CMD_RES_BAD_ARGUMENT;
	}

	if (delay > 60) {
		ADDLOGF_INFO("SetBatteryAckDelay: delay too long, max 60 seconds\n");
		return CMD_RES_BAD_ARGUMENT;
	}

	TuyaMCU_ackDelay = delay;

	return CMD_RES_OK;
}

// call from second or quick tick timers
// returns true when it sent information to the TuyaMCU
// no processing after state is updated
// Devices are powered by the TuyaMCU, transmit information and get turned off
// Use the minimal amount of communications
static void TuyaMCU_runBattery() {
	// Send a heartbeak if the quick tick thread has not sent a command
	if (!g_heartbeat_valid) {
		if (TuyaMCU_waitingToHearBack != TUYA_CMD_HEARTBEAT + 1) {
			ADDLOGF_EXTRADEBUG("Will send TUYA_CMD_HEARTBEAT.\n");
			TuyaMCU_talkToTuya(TUYA_CMD_HEARTBEAT, NULL, 0);
		}
		return;
	}

	/*
		Don't send heartbeats just work on product information but
		if we are waiting to hear back don't send a new one
	*/
	if (!g_product_information_valid) {
		if (TuyaMCU_waitingToHearBack != TUYA_CMD_QUERY_PRODUCT + 1) {
			ADDLOGF_EXTRADEBUG("Will send TUYA_CMD_QUERY_PRODUCT.\n");
			/* Request production information */
			TuyaMCU_talkToTuya(TUYA_CMD_QUERY_PRODUCT, NULL, 0);
		}
		return;
	}

	if (!g_mcuconfig_valid) {
		if (TuyaMCU_waitingToHearBack != TUYA_CMD_MCU_CONF + 1) {
			ADDLOGF_EXTRADEBUG("Will send TUYA_CMD_MCU_CONF.\n");
			TuyaMCU_talkToTuya(TUYA_CMD_MCU_CONF, NULL, 0);
		}
		return;
	}

	/*
		As soon as we have product information tell Tuya we are connected
		so we get the sensor details as soon as possible
	*/
	if (!g_wifi_state) {
		if (TuyaMCU_waitingToHearBack != TUYA_CMD_WIFI_STATE + 1) {
			ADDLOGF_TIMING("%i - %s - Sending TuyaMCU we are connected to cloud", xTaskGetTickCount(), __func__);
			uint8_t state = TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD;
			TuyaMCU_talkToTuya(TUYA_CMD_WIFI_STATE, &state, 1);
		}
		return;
	}
}
/*
	Send heartbeats if queued and nothing else is going on
	Waiting to hear timeout
	MQTT queued timeout
	Delayed data unit acknowledgement
*/
void TuyaMCU_onEverySecond() {
	// If we haven't heard back reduce the counter and fail the next time if 
	// it turns 0
	if (TuyaMCU_waitingToHearBack && !TuyaMCU_stillWaitingToHearBack--)
		TuyaMCU_waitingToHearBackReset();

	// MQTT Timeout to fail dpIds before Tuya turns module off
	if (!MQTT_IsReady() && TuyaMCU_MQTTConnectTimeout)
		TuyaMCU_MQTTConnectTimeout--;
	// if I queued something and it it still there do nothing
	// continue and fail items if timeout reaches zero
	if (TuyaMCU_queuedMQTT && MQTT_hasQueued() && TuyaMCU_MQTTConnectTimeout)
		return;

	// I timed out so send a failure message to Tuya
	if (TuyaMCU_queuedMQTT && !TuyaMCU_MQTTConnectTimeout)
		while (TuyaMCU_queuedMQTT)
			if (TuyaMCU_queuedMQTT-- || !TuyaMCU_ackDelayLeft) // more queued or no delay
				TuyaMCU_sendDataUnitResponse(DU_RESP_FAILURE);

	// process delayed logic
	static bool wasDelaying;
	if (TuyaMCU_ackDelayLeft) {
		wasDelaying = true;
		TuyaMCU_ackDelayLeft--;
	} else if (wasDelaying) {
		wasDelaying = false;
		TuyaMCU_sendDataUnitResponse(DU_RESP_SUCCESS);
	}
}
/*
	Processes incoming messages
	Sends out commands
	Responds to data units that have been queued
 */
void TuyaMCU_quickTick() {
	// process any received information
	uint32_t howMuchTheySaid;

	// while they are saying something listen to what they said
	while (howMuchTheySaid = TuyaMCU_listenToTuya())
		TuyaMCU_tuyaSaidWhat(howMuchTheySaid);

	TuyaMCU_runBattery();

	// if I queued something and it it still there do nothing
	if (TuyaMCU_queuedMQTT && MQTT_hasQueued())
		return;

	// process queued item logic
	while (TuyaMCU_queuedMQTT)
		if (TuyaMCU_queuedMQTT-- || !TuyaMCU_ackDelayLeft) // more queued or no delay
			TuyaMCU_sendDataUnitResponse(DU_RESP_SUCCESS);

}
static void TuyaMCU_init() {
	if (!g_tuyaMCURXBuffer)
		g_tuyaMCURXBuffer = (byte*)malloc(g_tuyaMCURXBufferSize);
	if (!g_tuyaMCUTXBuffer)
		g_tuyaMCUTXBuffer = (byte*)malloc(g_tuyaMCUTXBufferSize);

	UART_InitUART(g_baudRate, 0, false);
	UART_InitReceiveRingBuffer(1024);

	//cmddetail:{"name":"tuyaMcu_setBatteryAckDelay","args":"[ackDelay]",
	//cmddetail:"descr":"Defines the delay before the ACK is sent to TuyaMCU sending the device to sleep. Default value is 0 seconds.",
	//cmddetail:"fn":"Cmd_TuyaMCU_SetBatteryAckDelay","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_setBatteryAckDelay", Cmd_TuyaMCU_SetBatteryAckDelay, NULL);

	ADDLOGF_TIMING("%i - %s - Initialization of TuyaMCU", xTaskGetTickCount(), __func__);

	/* Request production information */
	TuyaMCU_talkToTuya(TUYA_CMD_HEARTBEAT, NULL, 0);
}

static bool TuyaMCU_activatePin(uint32_t pinIndex) {
	switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
	case IOR_TuyaMCU_RX:
	case IOR_TuyaMCU_TX:
		BIT_SET(g_driverPins, pinIndex);
		break;

	default:
		return false;
	}
	return true;
}

static void TuyaMCU_releasePin(uint32_t pinIndex) {
	BIT_CLEAR(g_driverPins, pinIndex);
}

static void TuyaMCU_stopDriver() {
	tuyaDP_t *curDp, *nxtDp;
	g_driverPins = 0;

	// free the tuyaDP_t linked list
	curDp = g_tuyaDPs;
	while (curDp) {
		nxtDp = curDp->next;
		free(curDp);
		curDp = nxtDp;
	}
	g_tuyaDPs = NULL;

	// free the g_tuyaMCURXBuffer
	free(g_tuyaMCURXBuffer);
	g_tuyaMCURXBuffer = NULL;
	g_tuyaMCURXBufferSize = TUYAMCU_BUFFER_SIZE;

	// free the g_tuyaMCURXBuffer
	free(g_tuyaMCUTXBuffer);
	g_tuyaMCUTXBuffer = NULL;
	g_tuyaMCUTXBufferSize = TUYAMCU_BUFFER_SIZE;

	// free g_productinfo
	free(g_productinfo);
	g_productinfo = NULL;
}

// framework request function
uint32_t TuyaMCU_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_PinRoles:
		// replace with real values for this driver
		g_driverIndex = PIN_pinIORoleDriver()[IOR_TuyaMCU_RX] = PIN_pinIORoleDriver()[IOR_TuyaMCU_TX] \
		              = arg;
		ADDLOGF_DEBUG("%s - Driver index %i", __func__, g_driverIndex);
		break;
	
	case OBKF_AcquirePin:
		return TuyaMCU_activatePin(arg);

	case OBKF_ReleasePin:
		TuyaMCU_releasePin(arg);
		break;

	case OBKF_Stop:
		TuyaMCU_stopDriver();
		break;
		
	case OBKF_ShouldPublish:
	case OBKF_NoOfChannels:
		return false;

	case OBKF_Init:
		TuyaMCU_init();
		break;

	default:
		break;
	}

	return true;
}

// lookup channel and determine is a TuyaMCU pin is assigned to it
bool TuyaMCU_isTuyaMCU(uint32_t channelIndex) {
	uint32_t driverPins = g_driverPins;

	for (uint32_t usedIndex = 0; driverPins && (usedIndex < g_registeredPinCount); usedIndex++) {
		uint32_t pinIndex = PIN_registeredPinIndex(usedIndex);
		if (!BIT_CHECK(driverPins, pinIndex))
			continue; // not my pin
		if (PIN_GetPinChannelForPinIndex(pinIndex) != channelIndex)
			continue; // channel not assigned to pin
		switch (PIN_GetPinRoleForPinIndex(pinIndex)) {
		case IOR_TuyaMCU_RX:
		case IOR_TuyaMCU_TX:
			return true;
		}
		BIT_CLEAR(driverPins, pinIndex);
	}
	return false;
}

void TuyaMCU_appendHTML(http_request_t* request, int bPreState)
{
	if (bPreState) {
		return;
	}

	static uint32_t waitingToHearBack;
	// wait at least one full second before saying we have no response
	if (TuyaMCU_waitingToHearBack || waitingToHearBack)
		hprintf255(request, "<h2>TuyaMCU: Offline</h2>");
	else
		hprintf255(request, "<h2>TuyaMCU: Online</h2>");

	if (TuyaMCU_waitingToHearBack)
		waitingToHearBack = 1;
	else if (waitingToHearBack)
		waitingToHearBack--;

	tuyaDP_t* tuyaDP = g_tuyaDPs;
	while (tuyaDP) {
		hprintf255(request, "<h2>dpId=%i, type=%s, value=%i</h2>", tuyaDP->dpId, TuyaMCU_getDataTypeString(tuyaDP->dpType), tuyaDP->prevValue);
		tuyaDP = tuyaDP->next;
	}

	if (g_product_information_valid)
		hprintf255(request, "<h4>TuyaMCU: %s</h4>", g_productinfo);

	if (!MQTT_IsReady() && TuyaMCU_MQTTConnectTimeout)
		hprintf255(request, "<h4>MQTT: Timeout in %is</h4>", TuyaMCU_MQTTConnectTimeout);
	else if (!TuyaMCU_MQTTConnectTimeout)
		hprintf255(request, "<h4>MQTT: Timed out</h4>");

	if (TuyaMCU_ackDelayLeft)
		hprintf255(request, "<h4>Delayed ACK in %is</h4>", TuyaMCU_ackDelayLeft);
}

int TuyaMCU_appendJSON(int id, void* request, jsonCb_t printer) {
	int iCnt = 0;
	tuyaDP_t* curDP;

	printer(request, "[");

	curDP = g_tuyaDPs;
	while (curDP) {
		if (id == -1 || id == curDP->dpId) {
			if (iCnt++)
				printer(request, ",");
			printer(request, "{\"id\":%i,\"type\":%i,\"data\":%i", curDP->dpId, curDP->dpType, curDP->prevValue);
		}
		curDP = curDP->next;
	}


	printer(request, "]");
	return 0;
}

#endif // ENABLE_DRIVER_TUYAMCU
