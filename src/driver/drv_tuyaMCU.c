
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
#include "drv_public.h"
#include "drv_tuyaMCU.h"
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

static byte g_tuyaMCUConfirmationsToSend_0x08 = 0;
static byte g_tuyaMCUConfirmationsToSend_0x05 = 0;
static byte g_resetWiFiEvents = 0;

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
	// store last channel value to avoid sending it again
	int prevValue;
	struct tuyaMCUMapping_s* next;
} tuyaMCUMapping_t;

tuyaMCUMapping_t* g_tuyaMappings = 0;

/**
 * Dimmer range
 *
 * Map OpenBK7231T_App's dimmer range of 0..100 to the dimmer range used by TuyMCU.
 * Use tuyaMCU_setDimmerrange command to set range used by TuyaMCU.
 */
 // minimum dimmer value as reported by TuyaMCU dimmer
static int g_dimmerRangeMin = 0;
// maximum dimmer value as reported by TuyaMCU dimmer
static int g_dimmerRangeMax = 100;

// serial baud rate used to communicate with the TuyaMCU
// common baud rates are 9600 bit/s and 115200 bit/s
const uint32_t g_baudRate = 9600;

static bool heartbeat_valid = false;
static int heartbeat_timer = 0;
static int heartbeat_counter = 0;
static bool product_information_valid = false;
// ?? it's unused atm
//static char *prod_info = NULL;
static bool working_mode_valid = false;
static bool wifi_state_valid = false;
static bool wifi_state = false;
static int wifi_state_timer = 0;
static bool self_processing_mode = true;
static bool state_updated = false;
static int g_sendQueryStatePackets = 0;
static int g_tuyaMCUBatteryAckDelay = 0;

// wifistate to send when not online
// See: https://imgur.com/a/mEfhfiA
static byte g_defaultTuyaMCUWiFiState = 0x00;

#define TUYAMCU_BUFFER_SIZE		256

static byte *g_tuyaMCUpayloadBuffer = 0;
static int g_tuyaMCUpayloadBufferSize = 0;

static bool g_tuyaMCU_batteryPoweredMode = false;

typedef struct tuyaMCUPacket_s {
	byte *data;
	int size;
	int allocated;
	struct tuyaMCUPacket_s *next;
} tuyaMCUPacket_t;

tuyaMCUPacket_t *tm_emptyPackets = 0;
tuyaMCUPacket_t *tm_sendPackets = 0;

uint32_t g_driverIndex;
uint32_t g_driverPins;

tuyaMCUPacket_t *TUYAMCU_AddToQueue(int len) {
	tuyaMCUPacket_t *toUse;
	if (tm_emptyPackets) {
		toUse = tm_emptyPackets;
		tm_emptyPackets = toUse->next;

		if (len > toUse->allocated) {
			toUse->data = realloc(toUse->data, len);
			toUse->allocated = len;
		}
	}
	else {
		toUse = malloc(sizeof(tuyaMCUPacket_t));
		int toAlloc = 128;
		if (len > toAlloc)
			toAlloc = len;
		toUse->allocated = toAlloc;
		toUse->data = malloc(toUse->allocated);
	}
	toUse->size = len;
	if (tm_sendPackets == 0) {
		tm_sendPackets = toUse;
	}
	else {
		tuyaMCUPacket_t *p = tm_sendPackets;
		while (p->next) {
			p = p->next;
		}
		p->next = toUse;
	}
	toUse->next = 0;
	return toUse;
}
bool TUYAMCU_SendFromQueue() {
	tuyaMCUPacket_t *toUse;
	if (tm_sendPackets == 0)
		return false;
	toUse = tm_sendPackets;
	tm_sendPackets = toUse->next;

	UART_SendByte(0x55);
	UART_SendByte(0xAA);
	UART_SendByte(0x00);
	for (int i = 0; i < toUse->size; i++) {
		UART_SendByte(toUse->data[i]);
	}

	toUse->next = tm_emptyPackets;
	tm_emptyPackets = toUse;
	return true;
}

tuyaMCUMapping_t* TuyaMCU_FindDefForID(int dpId) {
	tuyaMCUMapping_t* cur;

	cur = g_tuyaMappings;
	while (cur) {
		if (cur->dpId == dpId)
			return cur;
		cur = cur->next;
	}
	return 0;
}

tuyaMCUMapping_t* TuyaMCU_MapIDToChannel(int dpId, int dpType) {
	tuyaMCUMapping_t* cur;

	cur = TuyaMCU_FindDefForID(dpId);

	if (cur == 0) {
		cur = (tuyaMCUMapping_t*)malloc(sizeof(tuyaMCUMapping_t));
		cur->next = g_tuyaMappings;
		g_tuyaMappings = cur;
	}
	cur->dpId = dpId;
	cur->dpType = dpType;
	cur->prevValue = 0;
	return cur;
}

// now you can detect TuyaMCU faults with event handler
// addChangeHandler MissedHeartbeats > 4 setChannel 0 1
void TuyaMCU_SetHeartbeatCounter(int v) {
	EventHandlers_ProcessVariableChange_Integer(CMD_EVENT_MISSEDHEARTBEATS, heartbeat_counter, v);
	heartbeat_counter = v;
}

// header version command lenght data checksum
// 55AA     00      00      0000   xx   00

#define MIN_TUYAMCU_PACKET_SIZE (2+1+1+2+1)
// talks to TuyaMCU, puts what they said in a buffer and returns
// how much they said
int TuyaMCU_talkToThem(byte* theySaid, uint32_t mostWeCanHear) {
	uint32_t howMuchTheyNeedToSay;
	uint32_t howMuchTheySaid = UART_GetDataSize();
	uint32_t whatWeDidnotWantToHear = 0;
	byte a, b, version, command, lena, lenb;

	if (howMuchTheySaid < MIN_TUYAMCU_PACKET_SIZE)
		return 0;

	// skip garbage data (should not happen)
	while (howMuchTheySaid >= MIN_TUYAMCU_PACKET_SIZE) {
		a = UART_GetByte(0);
		b = UART_GetByte(1);
		if (a == 0x55 && b == 0xAA)
			break;
		
		UART_ConsumeBytes(1);
		whatWeDidnotWantToHear++;
		howMuchTheySaid--;
	}
	if (whatWeDidnotWantToHear > 0) {
		ADDLOGF_WARN("Consumed %i unwanted non-header byte in Tuya MCU buffer\n", whatWeDidnotWantToHear);
	}
	if (howMuchTheySaid < MIN_TUYAMCU_PACKET_SIZE)
		return 0;

	version = UART_GetByte(2);
	command = UART_GetByte(3);
	lena = UART_GetByte(4); // hi
	lenb = UART_GetByte(5); // lo
	howMuchTheyNeedToSay = lenb | lena >> 8;
	// now check if we have received whole packet
	howMuchTheyNeedToSay += MIN_TUYAMCU_PACKET_SIZE; // header 2 bytes, version, command, lenght, chekcusm
	if (howMuchTheySaid < howMuchTheyNeedToSay) {
		ADDLOGF_DEBUG("Still need %i bytes to complete message", howMuchTheyNeedToSay - howMuchTheySaid);
		return 0;
	}
	
	// can packet fit into the buffer?
	if (mostWeCanHear < howMuchTheyNeedToSay) {
		ADDLOGF_ERROR("TuyaMCU packet too large, %i > %i\n", howMuchTheyNeedToSay, mostWeCanHear);
		UART_ConsumeBytes(howMuchTheyNeedToSay);
		return 0;
	}

	for (uint32_t i = 0; i < howMuchTheyNeedToSay; i++)
		theySaid[i] = UART_GetByte(i);
	// consume whole packet (but don't touch next one, if any)
	UART_ConsumeBytes(howMuchTheyNeedToSay);
	return howMuchTheyNeedToSay;
}

// append header, len, everything, checksum
void TuyaMCU_SendCommandWithData(byte cmdType, byte* data, int payload_len) {
	int i;
	
	byte check_sum = (0xFF + cmdType + (payload_len >> 8) + (payload_len & 0xFF));

	if (CFG_HasFlag(OBK_FLAG_TUYAMCU_USE_QUEUE)) {
		tuyaMCUPacket_t *p = TUYAMCU_AddToQueue(payload_len + 4);
		p->data[0] = cmdType;
		p->data[1] = payload_len >> 8;
		p->data[2] = payload_len & 0xFF;
		memcpy(p->data + 3, data, payload_len);
		for (i = 0; i < payload_len; i++) {
			byte b = data[i];
			check_sum += b;
		}
		p->data[3+payload_len] = check_sum;
	} else {
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
	}
}
int TuyaMCU_AppendStateInternal(byte *buffer, int bufferMax, int currentLen, uint8_t id, int8_t type, void* value, int dataLen) {
	if (currentLen + 4 + dataLen >= bufferMax) {
		ADDLOGF_ERROR("Tuya buff overflow");
		return 0;
	}
	buffer[currentLen + 0] = id;
	buffer[currentLen + 1] = type;
	buffer[currentLen + 2] = 0x00;
	buffer[currentLen + 3] = dataLen;
	memcpy(buffer + (currentLen + 4), value, dataLen);
	return currentLen + 4 + dataLen;
}
void TuyaMCU_SendStateInternal(uint8_t id, uint8_t type, void* value, int dataLen)
{
	uint16_t payload_len = 0;
	
	payload_len = TuyaMCU_AppendStateInternal(g_tuyaMCUpayloadBuffer, g_tuyaMCUpayloadBufferSize,
		payload_len, id, type, value, dataLen);

	TuyaMCU_SendCommandWithData(TUYA_CMD_SET_DP, g_tuyaMCUpayloadBuffer, payload_len);
}
void TuyaMCU_SendTwoVals(byte idA, int valA, byte idB, int valB)
{
	byte swap[4];
	uint16_t payload_len = 0;

	swap[0] = ((byte*)&valA)[3];
	swap[1] = ((byte*)&valA)[2];
	swap[2] = ((byte*)&valA)[1];
	swap[3] = ((byte*)&valA)[0];
	payload_len = TuyaMCU_AppendStateInternal(g_tuyaMCUpayloadBuffer, g_tuyaMCUpayloadBufferSize,
		payload_len, idA, DP_TYPE_VALUE, swap, 4);
	swap[0] = ((byte*)&valB)[3];
	swap[1] = ((byte*)&valB)[2];
	swap[2] = ((byte*)&valB)[1];
	swap[3] = ((byte*)&valB)[0];
	payload_len = TuyaMCU_AppendStateInternal(g_tuyaMCUpayloadBuffer, g_tuyaMCUpayloadBufferSize,
		payload_len, idB, DP_TYPE_VALUE, swap, 4);

	TuyaMCU_SendCommandWithData(TUYA_CMD_SET_DP, g_tuyaMCUpayloadBuffer, payload_len);
}
void TuyaMCU_SendState(uint8_t id, uint8_t type, uint8_t* value)
{
	byte swap[4];

	switch (type) {
	case DP_TYPE_BOOL:
	case DP_TYPE_ENUM:
		TuyaMCU_SendStateInternal(id, type, value, 1);
		break;
	case DP_TYPE_VALUE:
		swap[0] = value[3];
		swap[1] = value[2];
		swap[2] = value[1];
		swap[3] = value[0];
		TuyaMCU_SendStateInternal(id, type, swap, 4);
		break;
	case DP_TYPE_STRING:
		TuyaMCU_SendStateInternal(id, type, value, strlen((const char*)value));
		break;
	case DP_TYPE_RAW:
		break;
	}
}

void TuyaMCU_SendBool(uint8_t id, bool value)
{
	TuyaMCU_SendState(id, DP_TYPE_BOOL, (uint8_t*)&value);
}

void TuyaMCU_SendValue(uint8_t id, uint32_t value)
{
	TuyaMCU_SendState(id, DP_TYPE_VALUE, (uint8_t*)(&value));
}

void TuyaMCU_SendEnum(uint8_t id, uint32_t value)
{
	TuyaMCU_SendState(id, DP_TYPE_ENUM, (uint8_t*)(&value));
}

void TuyaMCU_SendString(uint8_t id, char data[]) {
	uint16_t len = strlen(data);
	uint16_t payload_len = 4 + len;
	g_tuyaMCUpayloadBuffer[0] = id;
	g_tuyaMCUpayloadBuffer[1] = DP_TYPE_STRING;
	g_tuyaMCUpayloadBuffer[2] = len >> 8;
	g_tuyaMCUpayloadBuffer[3] = len & 0xFF;

	for (uint16_t i = 0; i < len; i++) {
		g_tuyaMCUpayloadBuffer[4 + i] = data[i];
	}

	TuyaMCU_SendCommandWithData(TUYA_CMD_SET_DP, g_tuyaMCUpayloadBuffer, payload_len);
}

// See this post to find about about arguments of WiFi state
// https://www.elektroda.com/rtvforum/viewtopic.php?p=20483899#20483899
commandResult_t Cmd_TuyaMCU_Set_DefaultWiFiState(const void* context, const char* cmd, const char* args, int cmdFlags) {

	g_defaultTuyaMCUWiFiState = atoi(args);

	return CMD_RES_OK;
}
//battery-powered water sensor with TyuaMCU request to get somo response
// uartSendHex 55AA0001000000 - this will get reply:
//Info:TuyaMCU:TuyaMCU_ParseQueryProductInformation: received {"p":"j53rkdu55ydc0fkq","v":"1.0.0"}
//
// and this will get states: 0x55 0xAA 0x00 0x02 0x00 0x01 0x04 0x06
// uartSendHex 55AA000200010406
/*Info:MAIN:Time 143, free 88864, MQTT 1, bWifi 1, secondsWithNoPing -1, socks 2/38

Info:TuyaMCU:TUYAMCU received: 55 AA 00 08 00 0C 00 02 02 02 02 02 02 01 04 00 01 00 25

Info:TuyaMCU:TuyaMCU_theySaidWhat: processing V0 command 8 with 19 bytes

*/

int TuyaMCU_ParseDPType(const char *dpTypeString) {
	int dpType;
	if (!stricmp(dpTypeString, "bool")) {
		dpType = DP_TYPE_BOOL;
	}
	else if (!stricmp(dpTypeString, "val")) {
		dpType = DP_TYPE_VALUE;
	}
	else if (!stricmp(dpTypeString, "str")) {
		dpType = DP_TYPE_STRING;
	}
	else if (!stricmp(dpTypeString, "enum")) {
		dpType = DP_TYPE_ENUM;
	}
	else if (!stricmp(dpTypeString, "raw")) {
		dpType = DP_TYPE_RAW;
	}
	else if (!stricmp(dpTypeString, "RAW_DDS238")) {
		// linkTuyaMCUOutputToChannel 6 RAW_DDS238
		dpType = DP_TYPE_RAW_DDS238Packet;
	}
	else if (!stricmp(dpTypeString, "RAW_TAC2121C_VCP")) {
		// linkTuyaMCUOutputToChannel 6 RAW_TAC2121C_VCP
		dpType = DP_TYPE_RAW_TAC2121C_VCP;
	}
	else if (!stricmp(dpTypeString, "RAW_V2C3P3")) {
		dpType = DP_TYPE_RAW_V2C3P3;
	}
	else if (!stricmp(dpTypeString, "RAW_VCPPfF")) {
		dpType = DP_TYPE_RAW_VCPPfF;
	}
	else if (!stricmp(dpTypeString, "RAW_TAC2121C_Yesterday")) {
		dpType = DP_TYPE_RAW_TAC2121C_YESTERDAY;
	}
	else if (!stricmp(dpTypeString, "RAW_TAC2121C_LastMonth")) {
		dpType = DP_TYPE_RAW_TAC2121C_LASTMONTH;
	}
	else if (!stricmp(dpTypeString, "MQTT")) {
		// linkTuyaMCUOutputToChannel 6 MQTT
		dpType = DP_TYPE_PUBLISH_TO_MQTT;
	}
	else {
		if (strIsInteger(dpTypeString)) {
			dpType = atoi(dpTypeString);
		}
		else {
			ADDLOGF_INFO("%s is not a valid var type\n", dpTypeString);
			return DP_TYPE_VALUE;
		}
	}
	return dpType;
}

void TuyaMCU_Send(byte* data, int size) {
	int i;
	unsigned char check_sum;

	check_sum = 0;
	for (i = 0; i < size; i++) {
		byte b = data[i];
		check_sum += b;
		UART_SendByte(b);
	}
	UART_SendByte(check_sum);

	ADDLOGF_INFO("\nWe sent %i bytes to Tuya MCU\n", size + 1);
}

commandResult_t TuyaMCU_SendHeartbeat(const void* context, const char* cmd, const char* args, int cmdFlags) {
	TuyaMCU_SendCommandWithData(TUYA_CMD_HEARTBEAT, NULL, 0);

	return CMD_RES_OK;
}

commandResult_t TuyaMCU_SendQueryProductInformation(const void* context, const char* cmd, const char* args, int cmdFlags) {
	TuyaMCU_SendCommandWithData(TUYA_CMD_QUERY_PRODUCT, NULL, 0);

	return CMD_RES_OK;
}
commandResult_t TuyaMCU_SendQueryState(const void* context, const char* cmd, const char* args, int cmdFlags) {
	TuyaMCU_SendCommandWithData(TUYA_CMD_QUERY_STATE, NULL, 0);

	return CMD_RES_OK;
}

void TuyaMCU_SendStateRawFromString(int dpId, const char *args) {
	int cur = 0;
	byte buffer[64];

	while (*args) {
		if (cur >= sizeof(buffer)) {
			ADDLOGF_ERROR("Tuya raw buff overflow");
			return;
		}
		buffer[cur] = CMD_ParseOrExpandHexByte(&args);
		cur++;
	}
	TuyaMCU_SendStateInternal(dpId, DP_TYPE_RAW, buffer, cur);
}
const char *STR_FindArg(const char *s, int arg) {
	while (1) {
		while (isspace((int)*s)) {
			if (*s == 0)
				return "";
			s++;
		}
		arg--;
		if (arg < 0) {
			return s;
		}
		while (isspace((int)*s) == false) {
			if (*s == 0)
				return "";
			s++;
		}
	}
	return 0;
}
// tuyaMcu_sendState id type value
// send boolean true
// tuyaMcu_sendState 25 1 1
// send boolean false
// tuyaMcu_sendState 25 1 0
// send value 87
// tuyaMcu_sendState 25 2 87
// send string 
// tuyaMcu_sendState 25 3 ff0000646464ff 
// send raw 
// tuyaMcu_sendState 25 0 ff$CH3$00646464ff 
// tuyaMcu_sendState 2 0 0404010C$CH9$$CH10$$CH11$FF04FEFF0031
// send enum (type 4)
// tuyaMcu_sendState 14 4 2
commandResult_t TuyaMCU_SendStateCmd(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int dpId;
	int dpType;
	int value;
	const char *valStr;

	Tokenizer_TokenizeString(args, 0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 3)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}

	dpId = Tokenizer_GetArgInteger(0);
	dpType = TuyaMCU_ParseDPType(Tokenizer_GetArg(1)); ;
	if (dpType == DP_TYPE_STRING) {
		valStr = Tokenizer_GetArg(2);
		TuyaMCU_SendState(dpId, DP_TYPE_STRING, (uint8_t*)valStr);
	}
	else if (dpType == DP_TYPE_RAW) {
		//valStr = Tokenizer_GetArg(2);
		valStr = STR_FindArg(args, 2);
		TuyaMCU_SendStateRawFromString(dpId, valStr);
	}
	else {
		value = Tokenizer_GetArgInteger(2);
		TuyaMCU_SendState(dpId, dpType, (uint8_t*)&value);
	}

	return CMD_RES_OK;
}

commandResult_t TuyaMCU_SendMCUConf(const void* context, const char* cmd, const char* args, int cmdFlags) {
	TuyaMCU_SendCommandWithData(TUYA_CMD_MCU_CONF, NULL, 0);

	return CMD_RES_OK;
}

void Tuya_SetWifiState(uint8_t state)
{
	TuyaMCU_SendCommandWithData(TUYA_CMD_WIFI_STATE, &state, 1);
}
void TuyaMCU_SendNetworkStatus()
{
	uint8_t state = TUYA_NETWORK_STATUS_NOT_CONNECTED;
	if (Main_IsOpenAccessPointMode() != 0) {
		state = TUYA_NETWORK_STATUS_AP_MODE;
	}
	else if (Main_HasWiFiConnected() != 0) {
		if (g_cfg.mqtt_host[0] == 0) {
			// if not MQTT configured at all
			state = TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD;
		} else {
			// otherwise, wait for MQTT connection
			state = Main_HasMQTTConnected() != 0 ? TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD : TUYA_NETWORK_STATUS_CONNECTED_TO_ROUTER;
		}
	}
	// allow override
	if (state < g_defaultTuyaMCUWiFiState) {
		state = g_defaultTuyaMCUWiFiState;
	}
	ADDLOGF_DEBUG("SendNetworkStatus: sending status 0x%X to MCU \n", state);
	TuyaMCU_SendCommandWithData(0x2B, &state, 1);
}
void TuyaMCU_ParseQueryProductInformation(const byte* data, int len) {
	char name[256];
	int useLen;

	useLen = len - 1;
	if (useLen > sizeof(name) - 1)
		useLen = sizeof(name) - 1;
	memcpy(name, data, useLen);
	name[useLen] = 0;

	ADDLOGF_DEBUG("ParseQueryProductInformation: received %s\n", name);
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

void TuyaMCU_PublishDPToMQTT(int dpId, int dataType, int sectorLen, const byte *data, int ofs) {
	char sName[32];
	int strLen;
	char *s;
	const byte *payload;
	int index;

	// really it's just +1 for NULL character but let's keep more space
	strLen = sectorLen * 2 + 16;
	if (g_tuyaMCUpayloadBufferSize < strLen) {
		g_tuyaMCUpayloadBuffer = realloc(g_tuyaMCUpayloadBuffer, strLen);
		g_tuyaMCUpayloadBufferSize = strLen;
	}
	s = (char*)g_tuyaMCUpayloadBuffer;
	*s = 0;

	sprintf(sName, "tm/%s/%i", TuyaMCU_GetDataTypeString(dataType), dpId);
	payload = data + ofs;
	switch (dataType) 
	{
	case DP_TYPE_BOOL:
	case DP_TYPE_VALUE:
	case DP_TYPE_ENUM:
		if (sectorLen == 4){
			index = payload[0] << 24 | payload[1] << 16 | payload[2] << 8 | payload[3];
		} else if (sectorLen == 2) {
			index = payload[1] << 8 | payload[0];
		} else if (sectorLen == 1) {
			index = (int)payload[0];
		} else {
			index = 0;
		}
		sprintf(s, "%i", index);
		break;
	case DP_TYPE_STRING:
		memcpy(s, payload, sectorLen);
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
			s[index++] = "0123456789ABCDEF"[(*payload >> 4) & 0xF];
			s[index++] = "0123456789ABCDEF"[*payload & 0xF];
			payload++;
		}
		s[index] = 0; // null-terminate the string
		break;
		
	}
	if (*s == 0)
		return;

#if ENABLE_MQTT
	MQTT_PublishMain_StringString(sName, s, OBK_PUBLISH_FLAG_FORCE_REMOVE_GET);
#endif
}
void TuyaMCU_ParseStateMessage(const byte* data, int len) {
	tuyaMCUMapping_t* mapping;
	int ofs = 0;
	int dpId, dataType, sectorLen;
	int iVal;

	while (ofs + 4 < len) {
		dpId = data[ofs];
		dataType = data[ofs + 1];
		sectorLen = data[ofs + 2] << 8 | data[ofs + 3];

		ADDLOGF_DEBUG("ParseState: id %i type %i-%s len %i\n",
			dpId, dataType, TuyaMCU_GetDataTypeString(dataType), sectorLen);

		mapping = TuyaMCU_FindDefForID(dpId);
		if (!mapping)
			mapping = TuyaMCU_MapIDToChannel(dpId, dataType);

		if (sectorLen == 1) {
			iVal = (int)data[ofs + 4];
			ADDLOGF_DEBUG("ParseState: byte %i\n", iVal);
		} else if (sectorLen == 4) {
			iVal = data[ofs + 4] << 24 | data[ofs + 5] << 16 | data[ofs + 6] << 8 | data[ofs + 7];
			ADDLOGF_DEBUG("ParseState: int32 %i\n", iVal);
		}
		if (mapping->prevValue != iVal) {
			TuyaMCU_PublishDPToMQTT(dpId, dataType, sectorLen, data, ofs + 4);
			mapping->prevValue = iVal;
		}

		// size of header (type, datatype, len 2 bytes) + data sector size
		ofs += (4 + sectorLen);
	}
}
void TuyaMCU_ParseReportStatusType(const byte *value, int len) {
	int subcommand = value[0];
	ADDLOGF_DEBUG("command %i, subcommand %i\n", TUYA_CMD_REPORT_STATUS_RECORD_TYPE, subcommand);
	byte reply[2] = { 0x00, 0x00 };
	reply[0] = subcommand;
	switch (subcommand)
	{
	case 0x04:
		// query: 55 AA 03 34 00 01 04 3B
		// reply: 55 aa 00 34 00 02 04 00 39
		// No processing, just reply
		break;
	
	case 0x0B:
		// TuyaMCU version 3 equivalent packet to version 0 0x08 packet
		// This packet includes first DateTime (skip past), then DataUnits
		TuyaMCU_ParseStateMessage(value + 9, len - 9);
		state_updated = true;
		g_sendQueryStatePackets = 0;
		break;

	default:
		// Unknown subcommand, ignore
		return;
	}
	TuyaMCU_SendCommandWithData(TUYA_CMD_REPORT_STATUS_RECORD_TYPE, reply, 2);
}
void TuyaMCU_theySaidWhat(const byte* theySaid, int howMuchTheySaid) {
	int checkLen;
	int i;
	byte checkCheckSum;
	byte cmd;
	byte version;

	if (theySaid[0] != 0x55 || theySaid[1] != 0xAA) {
		ADDLOGF_INFO("ProcessIncoming: discarding packet with bad ident and len %i\n", howMuchTheySaid);
		return;
	}
	version = theySaid[2];
	checkLen = theySaid[5] | theySaid[4] >> 8;
	checkLen = checkLen + 2 + 1 + 1 + 2 + 1;
	if (checkLen != howMuchTheySaid) {
		ADDLOGF_INFO("ProcessIncoming: discarding packet bad expected len, expected %i and got len %i\n", checkLen, howMuchTheySaid);
		return;
	}
	checkCheckSum = 0;
	for (i = 0; i < howMuchTheySaid - 1; i++) {
		checkCheckSum += theySaid[i];
	}
	if (checkCheckSum != theySaid[howMuchTheySaid - 1]) {
		ADDLOGF_INFO("ProcessIncoming: discarding packet bad expected checksum, expected %i and got checksum %i\n", (int)theySaid[howMuchTheySaid - 1], (int)checkCheckSum);
		return;
	}
	cmd = theySaid[3];
	ADDLOGF_TIMING("%i - %s - ProcessIncoming[v=%i]: cmd %i, len %i", xTaskGetTickCount(), __func__, version, cmd, howMuchTheySaid);
	switch (cmd)
	{
	case TUYA_CMD_HEARTBEAT:
		heartbeat_valid = true;
		TuyaMCU_SetHeartbeatCounter(0);
		break;
	case TUYA_CMD_MCU_CONF:
		working_mode_valid = true;
		int dataCount;
		// https://github.com/openshwprojects/OpenBK7231T_App/issues/291
		// header	ver	TUYA_CMD_MCU_CONF	LENGHT							Chksum
		// Pushing
		// 55 AA	01	02					00 03	FF 01 01			06 
		// 55 AA	01	02					00 03	FF 01 00			05 
		// Rotating down
		// 55 AA	01	02					00 05	01 24 02 01 0A		39 
		// 55 AA	01	02					00 03	01 09 00			0F 
		// Rotating up
		// 55 AA	01	02					00 05	01 24 01 01 0A		38 
		// 55 AA	01	02					00 03	01 09 01			10 
		dataCount = theySaid[5];
		if (dataCount == 0)
		{
			self_processing_mode = true;
		}
		else if (dataCount == 2)
		{
			self_processing_mode = false;
			ADDLOGF_INFO("IMPORTANT!!! mcu conf pins: %i %i",
				(int)(theySaid[6]), (int)(theySaid[7]));
		}
		if (5 + dataCount + 2 != howMuchTheySaid) {
			ADDLOGF_INFO("ProcessIncoming: TUYA_CMD_MCU_CONF had wrong data lenght?");
		}
		else {
			ADDLOGF_INFO("ProcessIncoming: TUYA_CMD_MCU_CONF, TODO!");
		}
		break;
	case TUYA_CMD_WIFI_STATE:
		wifi_state_valid = true;
		break;
	case TUYA_CMD_WIFI_SELECT:
		// TUYA_CMD_WIFI_SELECT
		// it should have 1 payload byte, AP mode or EZ mode, but does it make difference for us?
		g_openAP = 1;

	case TUYA_CMD_WIFI_RESET:
		ADDLOGF_INFO("ProcessIncoming: 0x04 replying");
		// added for https://www.elektroda.com/rtvforum/viewtopic.php?p=21095905#21095905
		TuyaMCU_SendCommandWithData(0x04, 0, 0);
		break;
	case 0x22:
		{
			TuyaMCU_ParseStateMessage(theySaid + 6, howMuchTheySaid - 6);

			byte data23[1] = { 1 };
			ADDLOGF_INFO("ProcessIncoming: 0x22 replying");
			// For example, the module returns 55 aa 00 23 00 01 01 24
			TuyaMCU_SendCommandWithData(0x23, data23, 1);
		}
		break;

		
	case TUYA_CMD_STATE:
		TuyaMCU_ParseStateMessage(theySaid + 6, howMuchTheySaid - 6);
		state_updated = true;
		g_sendQueryStatePackets = 0;
		break;
	case TUYA_CMD_QUERY_PRODUCT:
		TuyaMCU_ParseQueryProductInformation(theySaid + 6, howMuchTheySaid - 6);
		product_information_valid = true;
		break;
	case TUYA_CMD_REPORT_STATUS_RECORD_TYPE:
		TuyaMCU_ParseReportStatusType(theySaid + 6, howMuchTheySaid - 6);
		break;
	case TUYA_CMD_NETWORK_STATUS:
		ADDLOGF_INFO("ProcessIncoming: (test for S09 calendar/IR device) received TUYA_CMD_NETWORK_STATUS 0x2B ");
		TuyaMCU_SendNetworkStatus();
		break;
	default:
		ADDLOGF_INFO("ProcessIncoming: unhandled type %i", cmd);
		break;
	}
	EventHandlers_FireEvent(CMD_EVENT_TUYAMCU_PARSED, cmd);
}
// tuyaMcu_sendCmd 0x30 000000
// This will send 55 AA 00 30 00 03 00 00 00 32
// 
commandResult_t TuyaMCU_SendUserCmd(const void* context, const char* cmd, const char* args, int cmdFlags) {
	byte packet[128];
	int c = 0;

	Tokenizer_TokenizeString(args, 0);

	int command = Tokenizer_GetArgInteger(0);
	//XJIKKA 20250327 tuyaMcu_sendCmd without second param bug
	if (Tokenizer_GetArgsCount() >= 2) {
		const char* s = Tokenizer_GetArg(1);
		if (s) {
			while (*s) {
				byte b;
				b = CMD_ParseOrExpandHexByte(&s);

				if (sizeof(packet) > c + 1) {
					packet[c] = b;
					c++;
				}
			}
		}
	}
	TuyaMCU_SendCommandWithData(command,packet, c);
	return CMD_RES_OK;
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

void TuyaMCU_BatteryPoweredMode(bool enable) {
	g_tuyaMCU_batteryPoweredMode = enable;
}

commandResult_t Cmd_TuyaMCU_BatteryPoweredMode(const void* context, const char* cmd, const char* args, int cmdFlags) {
	int enable = 1;

	Tokenizer_TokenizeString(args, 0);

	if (Tokenizer_GetArgsCount() > 0) {
		enable = Tokenizer_GetArgInteger(0);
	}
	ADDLOGF_INFO("TuyaMCU power saving %s\n", enable ? "enabled" : "disabled");
	TuyaMCU_BatteryPoweredMode(enable != 0);

	return CMD_RES_OK;
}

// call from second or quick tick timers
// returns true when it sent information to the TuyaMCU
// no processing after state is updated
// Devices are powered by the TuyaMCU, transmit information and get turned off
// Use the minimal amount of communications
bool TuyaMCU_RunBattery() {
	/* Don't worry about connection after state is updated device will be turned off */
	if (!state_updated) {
		/* Don't send heartbeats just work on product information */
		heartbeat_valid = true;
		if (product_information_valid == false) {
			ADDLOGF_EXTRADEBUG("Will send TUYA_CMD_QUERY_PRODUCT.\n");
			/* Request production information */
			TuyaMCU_SendCommandWithData(TUYA_CMD_QUERY_PRODUCT, NULL, 0);
			return true;
		} else {
			/* Don't bother with MCU config */
			working_mode_valid = true;
			/* No query state. Will be updated when connected to wifi/mqtt */
			/* One way process device will be turned off after publishing */
			if (Main_HasWiFiConnected()) {
				if (!wifi_state) {
					ADDLOGF_TIMING("%i - %s - Sending TuyaMCU we are connected to router", xTaskGetTickCount(), __func__);
					Tuya_SetWifiState(TUYA_NETWORK_STATUS_CONNECTED_TO_ROUTER);
					wifi_state = true;
					wifi_state_timer = false;
					return true;
				} else if (MQTT_IsReady()) { // use MQTT function, main function isn't updated soon enough
					if (!wifi_state_timer) {
						ADDLOGF_TIMING("%i - %s - Sending TuyaMCU we are connected to cloud", xTaskGetTickCount(), __func__);
						Tuya_SetWifiState(TUYA_NETWORK_STATUS_CONNECTED_TO_CLOUD);
						wifi_state_timer = true;
						return true;
					}
				}
			}
		}
	}
	return false;
}

int timer_send = 0;
void TuyaMCU_quickTick() {
	// process any received information
	uint32_t weHaveContact;
	do {
		byte theySaid[192];
		weHaveContact = TuyaMCU_talkToThem(theySaid, sizeof(theySaid));
		if (weHaveContact)
			TuyaMCU_theySaidWhat(theySaid, weHaveContact);
	} while (weHaveContact);

	if (g_tuyaMCU_batteryPoweredMode) {
		static int timer_battery = 0;
		if (timer_battery > 0) {
			timer_battery -= g_deltaTimeMS;
		} else if (TuyaMCU_RunBattery()) {
			timer_battery = 100;
		}
	}

	if (timer_send > 0) {
		timer_send -= g_deltaTimeMS;
	}
	else {
		if (TUYAMCU_SendFromQueue()) {
			timer_send = 100;
		}
	}
}

static SemaphoreHandle_t g_mutex = 0;

static void TuyaMCU_Shutdown() {
	tuyaMCUMapping_t *tmp, *nxt;
	tuyaMCUPacket_t *packet, *next_packet;

	// free the tuyaMCUMapping_t linked list
	tmp = g_tuyaMappings;
	while (tmp) {
		nxt = tmp->next;
		// free rawData if allocated
		free(tmp);
		tmp = nxt;
	}
	g_tuyaMappings = NULL;

	// free the tuyaMCUpayloadBuffer
	if (g_tuyaMCUpayloadBuffer) {
		free(g_tuyaMCUpayloadBuffer);
		g_tuyaMCUpayloadBuffer = NULL;
		g_tuyaMCUpayloadBufferSize = 0;
	}

	// free the tm_emptyPackets queue
	packet = tm_emptyPackets;
	while (packet) {
		next_packet = packet->next;
		if (packet->data) {
			free(packet->data);
			packet->data = NULL;
			packet->allocated = 0;
			packet->size = 0;
		}
		free(packet);
		packet = next_packet;
	}
	tm_emptyPackets = NULL;

	// free the tm_sendPackets queue
	packet = tm_sendPackets;
	while (packet) {
		next_packet = packet->next;
		if (packet->data) {
			free(packet->data);
			packet->data = NULL;
			packet->allocated = 0;
			packet->size = 0;
		}
		free(packet);
		packet = next_packet;
	}
	tm_sendPackets = NULL;

	// free the mutex
	if (g_mutex) {
		//vSemaphoreDelete(g_mutex);
		g_mutex = NULL;
	}
}
static void TuyaMCU_init() {
	g_resetWiFiEvents = 0;
	g_tuyaMCUConfirmationsToSend_0x05 = 0;
	g_tuyaMCUConfirmationsToSend_0x08 = 0;
	if (g_tuyaMCUpayloadBuffer == 0) {
		g_tuyaMCUpayloadBufferSize = TUYAMCU_BUFFER_SIZE;
		g_tuyaMCUpayloadBuffer = (byte*)malloc(TUYAMCU_BUFFER_SIZE);
	}

	UART_InitUART(g_baudRate, 0, false);
	UART_InitReceiveRingBuffer(1024);
	//cmddetail:{"name":"tuyaMcu_sendHeartbeat","args":"",
	//cmddetail:"descr":"Send heartbeat to TuyaMCU",
	//cmddetail:"fn":"TuyaMCU_SendHeartbeat","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendHeartbeat", TuyaMCU_SendHeartbeat, NULL);
	//cmddetail:{"name":"tuyaMcu_sendQueryState","args":"",
	//cmddetail:"descr":"Send query state command. No arguments needed.",
	//cmddetail:"fn":"TuyaMCU_SendQueryState","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendQueryState", TuyaMCU_SendQueryState, NULL);
	//cmddetail:{"name":"tuyaMcu_sendProductInformation","args":"",
	//cmddetail:"descr":"Send query packet (0x01). No arguments needed.",
	//cmddetail:"fn":"TuyaMCU_SendQueryProductInformation","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendProductInformation", TuyaMCU_SendQueryProductInformation, NULL);
	//cmddetail:{"name":"tuyaMcu_sendState","args":"[dpID][dpType][dpValue]",
	//cmddetail:"descr":"Manually send set state command. Do not use it. Use mapping, so communication is bidirectional and automatic.",
	//cmddetail:"fn":"TuyaMCU_SendStateCmd","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendState", TuyaMCU_SendStateCmd, NULL);
	//cmddetail:{"name":"tuyaMcu_sendMCUConf","args":"",
	//cmddetail:"descr":"Send MCU conf command",
	//cmddetail:"fn":"TuyaMCU_SendMCUConf","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendMCUConf", TuyaMCU_SendMCUConf, NULL);
	//cmddetail:{"name":"tuyaMcu_sendCmd","args":"[CommandIndex] [HexPayloadNBytes]",
	//cmddetail:"descr":"This will automatically calculate TuyaMCU checksum and length for given command ID and payload, then it will send a command. It's better to use it than uartSendHex",
	//cmddetail:"fn":"TuyaMCU_SendUserCmd","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_sendCmd", TuyaMCU_SendUserCmd, NULL);
	//cmddetail:{"name":"tuyaMcu_defWiFiState","args":"",
	//cmddetail:"descr":"Command sets the default WiFi state for TuyaMCU when device is not online. It may be required for some devices to work, because Tuya designs them to ignore touch buttons or beep when not paired. Please see [values table and description here](https://www.elektroda.com/rtvforum/viewtopic.php?p=20483899#20483899).",
	//cmddetail:"fn":"Cmd_TuyaMCU_Set_DefaultWiFiState","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_defWiFiState", Cmd_TuyaMCU_Set_DefaultWiFiState, NULL);

	//cmddetail:{"name":"tuyaMcu_setBatteryAckDelay","args":"[ackDelay]",
	//cmddetail:"descr":"Defines the delay before the ACK is sent to TuyaMCU sending the device to sleep. Default value is 0 seconds.",
	//cmddetail:"fn":"Cmd_TuyaMCU_SetBatteryAckDelay","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":""}
	CMD_RegisterCommand("tuyaMcu_setBatteryAckDelay", Cmd_TuyaMCU_SetBatteryAckDelay, NULL);
	
	//cmddetail:{"name":"tuyaMcu_batteryPoweredMode","args": "[Optional 1 or 0, by default 1 is assumed]",
	//cmddetail:"descr":"Enables battery mode communications for version 3 TuyaMCU. tuyaMcu_batteryPoweredMode 0 can be used to disable the mode.",
	//cmddetail:"fn":"Cmd_TuyaMCU_BatteryPoweredMode","file":"driver/drv_tuyaMCU.c","requires":"",
	//cmddetail:"examples":"tuyaMcu_batteryPoweredMode"}
	CMD_RegisterCommand("tuyaMcu_batteryPoweredMode", Cmd_TuyaMCU_BatteryPoweredMode, NULL);

	ADDLOGF_TIMING("%i - %s - Initialization of TuyaMCU", xTaskGetTickCount(), __func__);
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

	mapping = g_tuyaMappings;
	while (mapping) {
		hprintf255(request, "<h2>dpId=%i, type=%s, value=%i</h2>", mapping->dpId, TuyaMCU_GetDataTypeString(mapping->dpType), mapping->prevValue);
		mapping = mapping->next;
	}
}

#endif // ENABLE_DRIVER_TUYAMCU
