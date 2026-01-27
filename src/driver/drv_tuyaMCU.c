
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
#define         DP_TYPE_RAW                     0x00        //RAW type
#define         DP_TYPE_BOOL                    0x01        //bool type
#define         DP_TYPE_VALUE                   0x02        //value type
#define         DP_TYPE_STRING                  0x03        //string type
#define         DP_TYPE_ENUM                    0x04        //enum type
#define         DP_TYPE_BITMAP                  0x05        //fault type

// Subtypes of raw - added for OpenBeken - not Tuya standard
#define         DP_TYPE_RAW_DDS238Packet        200
#define			DP_TYPE_RAW_TAC2121C_VCP		201
#define			DP_TYPE_RAW_TAC2121C_YESTERDAY	202
#define			DP_TYPE_RAW_TAC2121C_LASTMONTH	203
#define			DP_TYPE_PUBLISH_TO_MQTT			204
#define			DP_TYPE_RAW_VCPPfF				205
#define			DP_TYPE_RAW_V2C3P3				206

const char* TuyaMCU_GetDataTypeString(int dpId) {
	if (DP_TYPE_RAW == dpId)
		return "raw";
	if (DP_TYPE_BOOL == dpId)
		return "bool";
	if (DP_TYPE_VALUE == dpId)
		return "val";
	if (DP_TYPE_STRING == dpId)
		return "str";
	if (DP_TYPE_ENUM == dpId)
		return "enum";
	if (DP_TYPE_BITMAP == dpId)
		return "bitmap";
	return "error";
}

typedef struct tuyaMCUMapping_s {
	// internal Tuya variable index
	byte dpId;
	// data point type (one of the DP_TYPE_xxx defines)
	byte dpType;
	// store last value to avoid sending it again
	int prevValue;
	struct tuyaMCUMapping_s* next;
} tuyaMCUMapping_t;

tuyaMCUMapping_t* g_tuyaMappings = 0;

// serial baud rate used to communicate with the TuyaMCU
// common baud rates are 9600 bit/s and 115200 bit/s
const uint32_t g_baudRate = 9600;

bool self_processing_mode = true;
bool state_updated;
uint32_t g_tuyaMCUBatteryAckDelay = 0;

// dynamic will increase based on need
// for V3 door sensor the largest packet is 62 bytes
#define TUYAMCU_BUFFER_SIZE		128

#define MIN_TUYAMCU_PACKET_SIZE (2+1+1+2+1)

byte *g_tuyaMCUTXBuffer;
uint32_t g_tuyaMCUTXBufferSize = TUYAMCU_BUFFER_SIZE;
byte *g_tuyaMCURXBuffer;
uint32_t g_tuyaMCURXBufferSize = TUYAMCU_BUFFER_SIZE;
uint32_t g_waitToSendRespounse;

uint32_t g_driverIndex;
uint32_t g_driverPins;

bool product_information_valid;
bool wifi_state;
bool mqtt_state;

uint32_t g_version;
char *g_productinfo;

uint32_t g_waitingToHearBack;

tuyaMCUMapping_t* TuyaMCU_FindDefForID(int dpId) {
	tuyaMCUMapping_t* cur = g_tuyaMappings;
	while (cur) {
		if (cur->dpId == dpId)
			return cur;
		cur = cur->next;
	}
	return 0;
}

tuyaMCUMapping_t* TuyaMCU_saveDpId(int dpId, int dpType) {
	tuyaMCUMapping_t* cur = TuyaMCU_FindDefForID(dpId);
	if (!cur) {
		cur = (tuyaMCUMapping_t*)malloc(sizeof(tuyaMCUMapping_t));
		cur->next = g_tuyaMappings;
		g_tuyaMappings = cur;
	}
	cur->dpId = dpId;
	cur->dpType = dpType;
	cur->prevValue = 0;
	return cur;
}

// header version command lenght data checksum
// 55AA     00      00      0000   xx   00

// talks to TuyaMCU, puts what they said in a buffer and returns
// how much they said
int TuyaMCU_listenToThem() {
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
		return g_waitingToHearBack = 0;
	}

	return howMuchTheyNeedToSay;
}

// append header, len, everything, checksum
void TuyaMCU_SendCommandWithData(byte cmdType, byte* data, int payload_len) {
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
	g_waitingToHearBack = cmdType;
	ADDLOGF_DEBUG("%s - Sent command %i, with payload len %i",
		__func__, cmdType, payload_len);
	if (payload_len>2)
		ADDLOGF_DEBUG("%s - Bytes 0x%X 0x%X", __func__, data[0], data[1]);

}
void TuyaMCU_ParseQueryProductInformation(const byte* prodInfo, int prodInfoLen) {
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

int http_obk_json_dps(int id, void* request, jsonCb_t printer) {
	int i;
	int iCnt = 0;
	char tmp[8];
	tuyaMCUMapping_t* cur;

	printer(request, "[");

	cur = g_tuyaMappings;
	while (cur) {
		if (id == -1 || id == cur->dpId) {
			if (iCnt) {
				printer(request, ",");
			}
			iCnt++;
			printer(request, "{\"id\":%i,\"type\":%i,\"data\":", cur->dpId, cur->dpType);
			printer(request, "0}", 0);
		}
		cur = cur->next;
	}


	printer(request, "]");
	return 0;
}

static void TuyaMCU_PublishDPToMQTT(int dpId, int dataType, int sectorLen, const byte *data) {
#if ENABLE_MQTT
	char sName[32];
	int strLen;
	char *s;
	int index;

	// really it's just +1 for NULL character but let's keep more space
	strLen = sectorLen * 2 + 16;
	if (g_tuyaMCUTXBufferSize < strLen) {
		g_tuyaMCUTXBuffer = realloc(g_tuyaMCUTXBuffer, strLen);
		g_tuyaMCUTXBufferSize = strLen;
	}
	s = (char*)g_tuyaMCUTXBuffer;
	*s = 0;

	sprintf(sName, "tm/%s/%i", TuyaMCU_GetDataTypeString(dataType), dpId);
	switch (dataType) 
	{
	case DP_TYPE_BOOL:
	case DP_TYPE_VALUE:
	case DP_TYPE_ENUM:
		if (sectorLen == 4){
			index = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
		} else if (sectorLen == 2) {
			index = data[1] << 8 | data[0];
		} else if (sectorLen == 1) {
			index = (int)data[0];
		} else {
			index = 0;
		}
		sprintf(s, "%i", index);
		break;
	case DP_TYPE_STRING:
		memcpy(s, data, sectorLen);
		s[sectorLen] = 0;
		break;
	case DP_TYPE_RAW:
	case DP_TYPE_BITMAP:
		// convert payload bytes string like FFAABB in s
		index = 0;
		if (0) {
			*s = '0';
			s++; index++;
			*s = 'x';
			s++; index++;
		}
		for (int i = 0; i < sectorLen; i++) {
			// convert each byte to two hexadecimal characters
			s[index++] = "0123456789ABCDEF"[(*data >> 4) & 0xF];
			s[index++] = "0123456789ABCDEF"[*data & 0xF];
			data++;
		}
		s[index] = 0; // null-terminate the string
		break;
		
	}
	if (*s == 0)
		return;

	MQTT_PublishMain_StringString(sName, s, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
#endif
}
void TuyaMCU_ParseStateMessage(const byte* data, int len) {
	tuyaMCUMapping_t* mapping;
	int dpId, dataType, sectorLen;
	int iVal = 0;

	dpId = data[0];
	dataType = data[1];
	sectorLen = data[2] << 8 | data[3];

	ADDLOGF_DEBUG("ParseState: id %i type %i-%s len %i\n",
		dpId, dataType, TuyaMCU_GetDataTypeString(dataType), sectorLen);

	mapping = TuyaMCU_FindDefForID(dpId);
	if (!mapping)
		mapping = TuyaMCU_saveDpId(dpId, dataType);

	if (sectorLen == 1) {
		iVal = (int)data[4];
	} else if (sectorLen == 4) {
		iVal = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
	}
	if (mapping->prevValue != iVal) {
		TuyaMCU_PublishDPToMQTT(dpId, dataType, sectorLen, data + 4);
		mapping->prevValue = iVal;
	}
}
static void TuyaMCU_ParseReportStatusType(const byte *value, int len) {
	byte reply[2] = { 0x00, 0x00 };
	uint32_t subcommand = value[0];
	reply[0] = subcommand;
	ADDLOGF_DEBUG("command %i, subcommand %i\n", TUYA_CMD_REPORT_STATUS_RECORD_TYPE, subcommand);
	switch (subcommand) {
	case 0x0B:
		// TuyaMCU version 3 equivalent packet to version 0 0x08 packet
		// This packet includes first DateTime (skip past), then DataUnits
		TuyaMCU_ParseStateMessage(value + 9, len - 9);
		state_updated = true;
		break;

	default:
		// Unknown subcommand, ignore
		return;
	}
	TuyaMCU_SendCommandWithData(TUYA_CMD_REPORT_STATUS_RECORD_TYPE, reply, 2);
}
void TuyaMCU_theySaidWhat(int howMuchTheySaid) {
	uint32_t whatWeHeard = g_tuyaMCURXBuffer[3];

	g_version = g_tuyaMCURXBuffer[2];
	if (g_waitingToHearBack && g_waitingToHearBack != whatWeHeard)
		ADDLOGF_WARN("%s - Rec'd command response (%i), but sent command %i", __func__, whatWeHeard, g_waitingToHearBack);
	g_waitingToHearBack = 0;
	ADDLOGF_TIMING("%i - %s - ProcessIncoming[v=%i]: cmd %i, len %i", xTaskGetTickCount(), __func__, g_version, whatWeHeard, howMuchTheySaid);
	switch (whatWeHeard) {
	case TUYA_CMD_HEARTBEAT:
		break;
	case TUYA_CMD_QUERY_PRODUCT:
		TuyaMCU_ParseQueryProductInformation(g_tuyaMCURXBuffer + 6, howMuchTheySaid - 6);
		product_information_valid = true;
		break;
	case TUYA_CMD_MCU_CONF:
		if (howMuchTheySaid == MIN_TUYAMCU_PACKET_SIZE) {
			self_processing_mode = true;
		} else if (howMuchTheySaid == MIN_TUYAMCU_PACKET_SIZE + 2) {
			self_processing_mode = false;
			ADDLOGF_INFO("IMPORTANT!!! mcu conf pins: %i %i", (int)(g_tuyaMCURXBuffer[6]), (int)(g_tuyaMCURXBuffer[7]));
		}
		ADDLOGF_INFO("ProcessIncoming: TUYA_CMD_MCU_CONF, TODO!");
		break;
	case TUYA_CMD_WIFI_STATE:
		ADDLOGF_DEBUG("%s - rec'd wifi state response", __func__);
		break;
	case TUYA_CMD_WIFI_RESET:
		ADDLOGF_INFO("ProcessIncoming: 0x04 replying");
		// added for https://www.elektroda.com/rtvforum/viewtopic.php?p=21095905#21095905
		TuyaMCU_SendCommandWithData(0x04, 0, 0);
		break;
	case TUYA_CMD_WIFI_SELECT:
		// TUYA_CMD_WIFI_SELECT
		// it should have 1 payload byte, AP mode or EZ mode, but does it make difference for us?
		g_openAP = 1;
	case TUYA_CMD_REPORT_STATUS_RECORD_TYPE:
		TuyaMCU_ParseReportStatusType(g_tuyaMCURXBuffer + 6, howMuchTheySaid - 6);
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

	g_tuyaMCUBatteryAckDelay = delay;

	return CMD_RES_OK;
}

// call from second or quick tick timers
// returns true when it sent information to the TuyaMCU
// no processing after state is updated
// Devices are powered by the TuyaMCU, transmit information and get turned off
// Use the minimal amount of communications
bool TuyaMCU_runBattery() {
	/* Don't worry about connection after state is updated device will be turned off */
	if (state_updated || g_waitingToHearBack)
		return false;

	/* Don't send heartbeats just work on product information */
	if (!product_information_valid) {
		ADDLOGF_EXTRADEBUG("Will send TUYA_CMD_QUERY_PRODUCT.\n");
		/* Request production information */
		TuyaMCU_SendCommandWithData(TUYA_CMD_QUERY_PRODUCT, NULL, 0);
		return true;
	}
	/* No query state. Will be updated when connected to wifi/mqtt */
	/* One way process device will be turned off after publishing */
	if (Main_HasWiFiConnected()) {
		if (!wifi_state) {
			ADDLOGF_TIMING("%i - %s - Sending TuyaMCU we are connected to router", xTaskGetTickCount(), __func__);
			const uint8_t state = TUYA_NETWORK_STATUS_CONNECTED_TO_ROUTER;
			TuyaMCU_SendCommandWithData(TUYA_CMD_WIFI_STATE, &state, 1);
			wifi_state = true;
			mqtt_state = false;
			return true;
		}
		if (MQTT_IsReady() && !mqtt_state) { // use MQTT function, main function isn't updated soon enough
			ADDLOGF_TIMING("%i - %s - Sending TuyaMCU we are connected to cloud", xTaskGetTickCount(), __func__);
			const uint8_t state = TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD;
			TuyaMCU_SendCommandWithData(TUYA_CMD_WIFI_STATE, &state, 1);
			mqtt_state = true;
			return true;
		}
	}
	return false;
}

void TuyaMCU_quickTick() {
	// process any received information
	uint32_t howMuchTheySaid;

	if (g_waitToSendRespounse)
		return;

	while (howMuchTheySaid = TuyaMCU_listenToThem())
			TuyaMCU_theySaidWhat(howMuchTheySaid);

	// start with a delay because during init we sent a
	// production request.
	static int timer_battery = 100;
	if (timer_battery > 0) {
		timer_battery -= g_deltaTimeMS;
	} else if (TuyaMCU_runBattery()) {
		timer_battery = 100;
	}
}

static void TuyaMCU_Shutdown() {
	tuyaMCUMapping_t *tmp, *nxt;

	// free the tuyaMCUMapping_t linked list
	tmp = g_tuyaMappings;
	while (tmp) {
		nxt = tmp->next;
		// free rawData if allocated
		free(tmp);
		tmp = nxt;
	}
	g_tuyaMappings = NULL;

	// free the g_tuyaMCURXBuffer
	if (g_tuyaMCURXBuffer) {
		free(g_tuyaMCURXBuffer);
		g_tuyaMCURXBuffer = NULL;
		g_tuyaMCURXBufferSize = 0;
	}

	// free the g_tuyaMCURXBuffer
	if (g_tuyaMCUTXBuffer) {
		free(g_tuyaMCUTXBuffer);
		g_tuyaMCUTXBuffer = NULL;
		g_tuyaMCUTXBufferSize = 0;
	}
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
	TuyaMCU_SendCommandWithData(TUYA_CMD_QUERY_PRODUCT, NULL, 0);

}

// Door sensor with TuyaMCU version 0 (not 3), so all replies have x00 and not 0x03 byte
// fakeTuyaPacket 55AA0008000C00010101010101030400010223
// fakeTuyaPacket 55AA0008000C00020202020202010100010123
// https://developer.tuya.com/en/docs/iot/tuyacloudlowpoweruniversalserialaccessprotocol?id=K95afs9h4tjjh
/// 55AA 00 08 000C 00 02 0202 020202010100010123
/// head vr id size dp dt dtln ????
// https://github.com/esphome/feature-requests/issues/497
/// 55AA 00 08 000C 00 02 02 02 02 02 02 01 01 0001 01 23
/// head vr id size FL YY MM DD HH MM SS ID TP SIZE VL CK
/// 55AA 00 08 000C 00 01 01 01 01 01 01 03 04 0001 02 23
// TP = 0x01    bool    1   Value range: 0x00/0x01.
// TP = 0x04    enum    1   Enumeration type, ranging from 0 to 255.

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
	TuyaMCU_Shutdown();
	g_driverPins = 0;
}

static bool TuyaMCU_shouldPublish(uint32_t pinRole) {
	return false;
}

static uint32_t TuyaMCU_noOfChannels(uint32_t pinRole) {
	return 0;
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
		return TuyaMCU_shouldPublish(arg);

	case OBKF_NoOfChannels:
		return TuyaMCU_noOfChannels(arg);

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

void TuyaMCU_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState)
{
	if (bPreState) {
		return;
	}
	tuyaMCUMapping_t* mapping;

	if (product_information_valid)
		hprintf255(request, "<h3>%s</h3>", g_productinfo);

	mapping = g_tuyaMappings;
	while (mapping) {
		hprintf255(request, "<h2>dpId=%i, type=%s, value=%i</h2>", mapping->dpId, TuyaMCU_GetDataTypeString(mapping->dpType), mapping->prevValue);
		mapping = mapping->next;
	}
}

#endif // ENABLE_DRIVER_TUYAMCU
