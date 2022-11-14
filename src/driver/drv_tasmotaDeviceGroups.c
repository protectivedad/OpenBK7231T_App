

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "../devicegroups/deviceGroups_public.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"

static const char* dgr_group = "239.255.250.250";
static int dgr_port = 4447;
static int dgr_retry_time_left = 20;

static int g_inCmdProcessing = 0;

const char *HAL_GetMyIPString();

void DRV_DGR_Dump(byte *message, int len);

// send all DGR on quick tick?
/*
#define MAX_DGR_PACKET 64;

typedef struct dgrPacket_s {
	dgrPacket_t *next;
	byte buffer[MAX_DGR_PACKET];
	byte length;
} dgrPacket_t;

dgrPacket_t *dgr_pending = 0;


void DGR_AddToSendQueue(byte *data, int len) {
	dgrPacket_t *p;

	p = dgr_pending;
	while(p) {
		if(p->length == 0) {
			
			break;
		}
		p = p->next;
	}
	if(p == 0) {
		p = malloc(sizeof(dgrPacket_t));
		p->next = dgr_pending;
		dgr_pending = p;
	}
	p->length = len;
	memcpy(p->buffer,data,len);
}
void DGR_FlushSendQueue() {
	dgrPacket_t *p;

	p = dgr_pending;
	while(p) {
		if(p->length != 0) {
			
			p->length = 0;
		}
		p = p->next;
	}

}

*/
// DGR send can be called from MQTT LED driver, but doing a DGR send
// directly from there may cause crashes.
// This is a temporary solution to avoid this problem.
bool g_dgr_ledDimmerPendingSend = false;
int g_dgr_ledDimmerPendingSend_value;
bool g_dgr_ledPowerPendingSend = false;
int g_dgr_ledPowerPendingSend_value;

//
//int DRV_DGR_CreateSocket_Send() {
//
//    struct sockaddr_in addr;
//    int flag = 1;
//	int fd;
//
//    // create what looks like an ordinary UDP socket
//    //
//    fd = socket(AF_INET, SOCK_DGRAM, 0);
//    if (fd < 0) {
//        return 1;
//    }
//
//    memset(&addr, 0, sizeof(addr));
//    addr.sin_family = AF_INET;
//    addr.sin_addr.s_addr = inet_addr(dgr_group);
//    addr.sin_port = htons(dgr_port);
//
//
//	return 0;
//}

byte Val255ToVal100(byte v){ 
	float fr;
	// convert to our 0-100 range
	fr = v / 255.0f;
	v = fr * 100;
	return v;
}
byte Val100ToVal255(byte v){ 
	float fr;
	fr = v / 100.0f;
	v = fr * 255;
	return v;
}
static int g_dgr_socket_receive = -1;
static int g_dgr_socket_send = -1;
static uint16_t g_dgr_send_seq = 0;
void DRV_DGR_CreateSocket_Send() {
    // create what looks like an ordinary UDP socket
    //
    g_dgr_socket_send = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_dgr_socket_send < 0) {
		ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Send: failed to do socket\n");
        return;
    }
	ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Send: socket created\n");



}
void DRV_DGR_Send_Generic(byte *message, int len) {
    struct sockaddr_in addr;
	int nbytes;

	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}


	g_dgr_send_seq++;

    // set up destination address
    //
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(dgr_group);
    addr.sin_port = htons(dgr_port);

    nbytes = sendto(
            g_dgr_socket_send,
           (const char*) message,
            len,
            0,
            (struct sockaddr*) &addr,
            sizeof(addr)
        );

	rtos_delay_milliseconds(1);

	// send twice with same seq.
    nbytes = sendto(
            g_dgr_socket_send,
           (const char*) message,
            len,
            0,
            (struct sockaddr*) &addr,
            sizeof(addr)
        );

	DRV_DGR_Dump(message, len);

	ADDLOG_EXTRADEBUG(LOG_FEATURE_DGR,"DRV_DGR_Send_Generic: sent message with seq %i\n",g_dgr_send_seq);

}

void DRV_DGR_Dump(byte *message, int len){
#ifdef DGRLOADMOREDEBUG	
	char tmp[100];
	char *p = tmp;
	for (int i = 0; i < len && i < 49; i++){
		sprintf(p, "%02X", message[i]);
		p+=2;
	}
	*p = 0;
	ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_Send_Generic: %s",tmp);
#endif	
}

void DRV_DGR_Send_Power(const char *groupName, int channelValues, int numChannels){
	int len;
	byte message[64];
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}

	len = DGR_Quick_FormatPowerState(message,sizeof(message),groupName,g_dgr_send_seq, 0,channelValues, numChannels);

	DRV_DGR_Send_Generic(message,len);
}
void DRV_DGR_Send_Brightness(const char *groupName, byte brightness){
	int len;
	byte message[64];
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}

	len = DGR_Quick_FormatBrightness(message,sizeof(message),groupName,g_dgr_send_seq, 0, brightness);

	DRV_DGR_Send_Generic(message,len);
}
void DRV_DGR_Send_RGBCW(const char *groupName, byte *rgbcw){
	int len;
	byte message[64];
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}

	len = DGR_Quick_FormatRGBCW(message,sizeof(message),groupName,g_dgr_send_seq, 0, rgbcw[0],rgbcw[1],rgbcw[2],rgbcw[3],rgbcw[4]);

	DRV_DGR_Send_Generic(message,len);
}
void DRV_DGR_CreateSocket_Receive() {

    struct sockaddr_in addr;
    struct ip_mreq mreq;
    int flag = 1;
	int broadcast = 0;
	int iResult = 1;

    // create what looks like an ordinary UDP socket
    //
    g_dgr_socket_receive = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_dgr_socket_receive < 0) {
		g_dgr_socket_receive = -1;
		ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: failed to do socket\n");
        return ;
    }

	if(broadcast)
	{

		iResult = setsockopt(g_dgr_socket_receive, SOL_SOCKET, SO_BROADCAST, (char *)&flag, sizeof(flag));
		if (iResult != 0)
		{
			ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: failed to do setsockopt SO_BROADCAST\n");
			close(g_dgr_socket_receive);
			g_dgr_socket_receive = -1;
			return ;
		}
	}
	else{
		// allow multiple sockets to use the same PORT number
		//
		if (
			setsockopt(
				g_dgr_socket_receive, SOL_SOCKET, SO_REUSEADDR, (char*) &flag, sizeof(flag)
			) < 0
		){
			ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: failed to do setsockopt SO_REUSEADDR\n");
			close(g_dgr_socket_receive);
			g_dgr_socket_receive = -1;
		  return ;
		}
	}

        // set up destination address
    //
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
    addr.sin_port = htons(dgr_port);

    // bind to receive address
    //
    if (bind(g_dgr_socket_receive, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
		ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: failed to do bind\n");
		close(g_dgr_socket_receive);
		g_dgr_socket_receive = -1;
        return ;
    }

    //if(0 != setsockopt(g_dgr_socket_receive,SOL_SOCKET,SO_BROADCAST,(const char*)&flag,sizeof(int))) {
    //    return 1;
    //}

	if(broadcast)
	{

	}
	else
	{

	  // use setsockopt() to request that the kernel join a multicast group
		//
		mreq.imr_multiaddr.s_addr = inet_addr(dgr_group);
		//mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);//inet_addr(MY_CAPTURE_IP);
    	///mreq.imr_interface.s_addr = inet_addr("192.168.0.122");
	iResult = setsockopt(
				g_dgr_socket_receive, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
			);
		if (
			iResult < 0
		){
			ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: failed to do setsockopt IP_ADD_MEMBERSHIP %i\n",iResult);
			close(g_dgr_socket_receive);
			g_dgr_socket_receive = -1;
			return ;
		}
	}

	lwip_fcntl(g_dgr_socket_receive, F_SETFL,O_NONBLOCK);

	ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_CreateSocket_Receive: Socket created, waiting for packets\n");
}

void DRV_DGR_processRGBCW(byte *rgbcw) {
	LED_SetFinalRGBCW(rgbcw);
}
void DRV_DGR_processPower(int relayStates, byte relaysCount) {
	int startIndex;
	int i;
	int ch;

	if(PIN_CountPinsWithRoleOrRole(IOR_PWM,IOR_PWM_n) > 0) {
		LED_SetEnableAll(BIT_CHECK(relayStates,0));
	} else {
		//if(CHANNEL_HasChannelSomeOutputPin(0)) {
			startIndex = 0;
		//} else {
		//	startIndex = 1;
		//}
		for(i = 0; i < relaysCount; i++) {
			int bOn;
			bOn = BIT_CHECK(relayStates,i);
			ch = startIndex+i;
			if(bOn) {
				if(CHANNEL_HasChannelPinWithRoleOrRole(ch,IOR_PWM,IOR_PWM_n)) {

				} else {
					CHANNEL_Set(ch,1,0);
				}
			} else {
				CHANNEL_Set(ch,0,0);
			}
		}
	}
}
void DRV_DGR_processBrightnessPowerOn(byte brightness) {
	ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_processBrightnessPowerOn: %i\n",(int)brightness);

	LED_SetDimmer(Val255ToVal100(brightness));

	//numPWMs = PIN_CountPinsWithRole(IOR_PWM,IOR_PWM_n);
	//idx_pin = PIN_FindPinIndexForRole(IOR_PWM,IOR_PWM_n,0);
	//idx_channel = PIN_GetPinChannelForPinIndex(idx_pin);

	//CHANNEL_Set(idx_channel,brightness,0);
	
}
void DRV_DGR_processLightFixedColor(byte fixedColor) {
	ADDLOG_INFO(LOG_FEATURE_DGR, "DRV_DGR_processLightFixedColor: %i\n", (int)fixedColor);

	LED_SetColorByIndex(fixedColor);

}
void DRV_DGR_processLightBrightness(byte brightness) {
	ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_processLightBrightness: %i\n",(int)brightness);

	LED_SetDimmer(Val255ToVal100(brightness));
	
}
typedef struct dgrMmember_s {
    struct sockaddr_in addr;
	uint16_t lastSeq;
} dgrMember_t;

#define MAX_DGR_MEMBERS 32
static dgrMember_t g_dgrMembers[MAX_DGR_MEMBERS];
static int g_curDGRMembers = 0;
static struct sockaddr_in addr;

dgrMember_t *findMember() {
	int i;
	for(i = 0; i < g_curDGRMembers; i++) {
		if(!memcmp(&g_dgrMembers[i].addr, &addr,sizeof(addr))) {
			return &g_dgrMembers[i];
		}
	}
	i = g_curDGRMembers;
	if(i>=MAX_DGR_MEMBERS)
		return 0;
	g_curDGRMembers ++;
	memcpy(&g_dgrMembers[i].addr,&addr,sizeof(addr));
	g_dgrMembers[i].lastSeq = 0;
	return &g_dgrMembers[i];
}

int DGR_CheckSequence(uint16_t seq) {
	dgrMember_t *m;
	
	m = findMember();
	
	if(m == 0)
		return 1;
	
	// make it work past wrap at
	if((seq > m->lastSeq) || (seq+10 > m->lastSeq+10)) {
		if(seq != (m->lastSeq+1)){
			ADDLOG_INFO(LOG_FEATURE_DGR,"Seq for %s skip %i->%i\n",inet_ntoa(m->addr.sin_addr), m->lastSeq, seq);
		}
		m->lastSeq = seq;
		return 0;
	}
	if(seq + 16 < m->lastSeq) {
		// hard reset
		m->lastSeq = seq;
		return 0;
	}
	return 1;
}

void DRV_DGR_RunEverySecond() {
	if(g_dgr_socket_receive<=0 || g_dgr_socket_send <= 0) {
		dgr_retry_time_left--;
		ADDLOG_INFO(LOG_FEATURE_DGR,"no sockets, will retry creation soon, in %i secs\n",dgr_retry_time_left);

		if(dgr_retry_time_left <= 0){
			dgr_retry_time_left = 20;
			if(g_dgr_socket_receive <= 0){
				DRV_DGR_CreateSocket_Receive();
			}
			if(g_dgr_socket_send <= 0){
				DRV_DGR_CreateSocket_Send();
			}
		}
	}
}
void DRV_DGR_RunQuickTick() {
	dgrDevice_t def;
    char msgbuf[64];
	struct sockaddr_in me;
	const char *myip;
	socklen_t addrlen;
	int nbytes;

	if(g_dgr_socket_receive<=0 || g_dgr_socket_send <= 0) {
		return ;
	}
    // send pending
	if (g_dgr_ledDimmerPendingSend) {
		g_dgr_ledDimmerPendingSend = false;
		DRV_DGR_Send_Brightness(CFG_DeviceGroups_GetName(), Val100ToVal255(g_dgr_ledDimmerPendingSend_value));
	}
	if (g_dgr_ledPowerPendingSend) {
		g_dgr_ledPowerPendingSend = false;
		DRV_DGR_Send_Power(CFG_DeviceGroups_GetName(), g_dgr_ledPowerPendingSend_value, 1);
	}

	// NOTE: 'addr' is global, and used in callbacks to determine the member.
        addrlen = sizeof(addr);
        nbytes = recvfrom(
            g_dgr_socket_receive,
            msgbuf,
            sizeof(msgbuf),
            0,
            (struct sockaddr *) &addr,
            &addrlen
        );
        if (nbytes <= 0) {
			//ADDLOG_INFO(LOG_FEATURE_DGR,"nothing\n");
            return ;
        }

		myip = HAL_GetMyIPString();
		me.sin_addr.s_addr = inet_addr(myip);

		if (me.sin_addr.s_addr == addr.sin_addr.s_addr){
			ADDLOG_INFO(LOG_FEATURE_DGR,"Ignoring message from self");
			return;
		}

		ADDLOG_EXTRADEBUG(LOG_FEATURE_DGR,"Received %i bytes from %s\n",nbytes,inet_ntoa(((struct sockaddr_in *)&addr)->sin_addr));
        msgbuf[nbytes] = '\0';

		strcpy(def.gr.groupName,CFG_DeviceGroups_GetName());
		def.gr.devGroupShare_In = CFG_DeviceGroups_GetRecvFlags();
		def.gr.devGroupShare_Out = CFG_DeviceGroups_GetSendFlags();
		def.cbs.processBrightnessPowerOn = DRV_DGR_processBrightnessPowerOn;
		def.cbs.processLightBrightness = DRV_DGR_processLightBrightness;
		def.cbs.processLightFixedColor = DRV_DGR_processLightFixedColor;
		def.cbs.processPower = DRV_DGR_processPower;
		def.cbs.processRGBCW = DRV_DGR_processRGBCW;
		def.cbs.checkSequence = DGR_CheckSequence;

		// don't send things that result from something we rxed...
		g_inCmdProcessing = 1;
		DRV_DGR_Dump((byte*)msgbuf, nbytes);

		DGR_Parse((byte*)msgbuf, nbytes, &def, (struct sockaddr *)&addr);
		g_inCmdProcessing = 0;

		//DGR_Parse(msgbuf, nbytes);
       // puts(msgbuf);
}
//static void DRV_DGR_Thread(beken_thread_arg_t arg) {
//
//    (void)( arg );
//
//	DRV_DGR_CreateSocket_Receive();
//	while(1) {
//		DRV_DGR_RunQuickTick();
//	}
//
//	return ;
//}
//xTaskHandle g_dgr_thread = NULL;

//void DRV_DGR_StartThread()
//{
//     OSStatus err = kNoErr;
//
//
//    err = rtos_create_thread( &g_dgr_thread, BEKEN_APPLICATION_PRIORITY,
//									"DGR_server",
//									(beken_thread_function_t)DRV_DGR_Thread,
//									0x100,
//									(beken_thread_arg_t)0 );
//
//}
void DRV_DGR_Shutdown()
{
	if(g_dgr_socket_receive>=0) {
		close(g_dgr_socket_receive);
		g_dgr_socket_receive = -1;
	}
}

// DGR_SendPower testSocket 1 1
// DGR_SendPower stringGroupName integerChannelValues integerChannelsCount
int CMD_DGR_SendPower(const void *context, const char *cmd, const char *args, int flags) {
	const char *groupName;
	int channelValues;
	int channelsCount;

	Tokenizer_TokenizeString(args,0);

	if(Tokenizer_GetArgsCount() < 3) {
		ADDLOG_INFO(LOG_FEATURE_DGR,"Command requires 3 arguments - groupname, channelvalues, valuescount\n");
		return 0;
	}
	groupName = Tokenizer_GetArg(0);
	channelValues = Tokenizer_GetArgInteger(1);
	channelsCount = Tokenizer_GetArgInteger(2);

	DRV_DGR_Send_Power(groupName,channelValues,channelsCount);
	ADDLOG_INFO(LOG_FEATURE_DGR,"CMD_DGR_SendPower: sent message to group %s\n",groupName);

	return 1;
}
void DRV_DGR_OnLedDimmerChange(int iVal) {
	//ADDLOG_INFO(LOG_FEATURE_DGR,"DRV_DGR_OnLedDimmerChange: called\n");
	if (g_dgr_socket_receive == 0) {
		return;
	}
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing) {
		return;
	}
	if ((CFG_DeviceGroups_GetSendFlags() & DGR_SHARE_LIGHT_BRI) == 0) {

		return;
	}
#if 1
	g_dgr_ledDimmerPendingSend = true;
	g_dgr_ledDimmerPendingSend_value = iVal;
#else
	DRV_DGR_Send_Brightness(CFG_DeviceGroups_GetName(), Val100ToVal255(g_dgr_ledDimmerPendingSend_value));
#endif
}



void DRV_DGR_OnLedEnableAllChange(int iVal) {
	if(g_dgr_socket_receive==0) {
		return;
	}
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}

	if((CFG_DeviceGroups_GetSendFlags() & DGR_SHARE_POWER)==0) {

		return;
	}

#if 1
	g_dgr_ledPowerPendingSend = true;
	g_dgr_ledPowerPendingSend_value = iVal;
#else
	DRV_DGR_Send_Power(CFG_DeviceGroups_GetName(), iVal, 1);
#endif
}
void DRV_DGR_OnChannelChanged(int ch, int value) {
	int channelValues;
	int channelsCount;
	int i;
	const char *groupName;

	if(g_dgr_socket_receive==0) {
		return;
	}
	// if this send is as a result of use RXing something, 
	// don't send it....
	if (g_inCmdProcessing){
		return;
	}

	if((CFG_DeviceGroups_GetSendFlags() & DGR_SHARE_POWER)==0) {

		return;
	}
	channelValues = 0;
	channelsCount = 0;
	groupName = CFG_DeviceGroups_GetName();

	for(i = 0; i < CHANNEL_MAX; i++) {
		if(CHANNEL_HasChannelPinWithRole(i,IOR_Relay) || CHANNEL_HasChannelPinWithRole(i,IOR_Relay_n)
			|| CHANNEL_HasChannelPinWithRole(i,IOR_LED) || CHANNEL_HasChannelPinWithRole(i,IOR_LED_n)) {
			channelsCount = i + 1;
			if(CHANNEL_Get(i)) {
				BIT_SET(channelValues ,i);
			}
		} 
	}
	if(channelsCount>0){
		DRV_DGR_Send_Power(groupName,channelValues,channelsCount);
	}


	
}
// DGR_SendBrightness roomLEDstrips 128
// DGR_SendBrightness stringGroupName integerBrightness
int CMD_DGR_SendBrightness(const void *context, const char *cmd, const char *args, int flags) {
	const char *groupName;
	int brightness;

	Tokenizer_TokenizeString(args,0);

	if(Tokenizer_GetArgsCount() < 2) {
		ADDLOG_INFO(LOG_FEATURE_DGR,"Command requires 2 arguments - groupname, brightess\n");
		return 0;
	}
	groupName = Tokenizer_GetArg(0);
	brightness = Tokenizer_GetArgInteger(1);

	DRV_DGR_Send_Brightness(groupName,brightness);
	ADDLOG_INFO(LOG_FEATURE_DGR,"DGR_SendBrightness: sent message to group %s\n",groupName);

	return 1;
}
// DGR_SendRGBCW roomLEDstrips 255 0 0
// DGR_SendRGBCW stringGroupName r g b
int CMD_DGR_SendRGBCW(const void *context, const char *cmd, const char *args, int flags) {
	const char *groupName;
	byte rgbcw[5];

	Tokenizer_TokenizeString(args,0);

	if(Tokenizer_GetArgsCount() < 4) {
		ADDLOG_INFO(LOG_FEATURE_DGR,"Command requires at least 4 arguments - groupname, r, g, b\n");
		return 0;
	}
	groupName = Tokenizer_GetArg(0);
	rgbcw[0] = Tokenizer_GetArgInteger(1);
	rgbcw[1] = Tokenizer_GetArgInteger(2);
	rgbcw[2] = Tokenizer_GetArgInteger(3);
	rgbcw[3] = Tokenizer_GetArgInteger(4);
	rgbcw[4] = Tokenizer_GetArgInteger(5);

	DRV_DGR_Send_RGBCW(groupName,rgbcw);
	ADDLOG_INFO(LOG_FEATURE_DGR,"DGR_SendRGBCW: sent message to group %s\n",groupName);

	return 1;
}
void DRV_DGR_Init()
{
	memset(&g_dgrMembers[0],0,sizeof(g_dgrMembers));
#if 0
	DRV_DGR_StartThread();
#else
	DRV_DGR_CreateSocket_Receive();
	DRV_DGR_CreateSocket_Send();

#endif
    CMD_RegisterCommand("DGR_SendPower", "", CMD_DGR_SendPower, "qqq", NULL);
    CMD_RegisterCommand("DGR_SendBrightness", "", CMD_DGR_SendBrightness, "qqq", NULL);
    CMD_RegisterCommand("DGR_SendRGBCW", "", CMD_DGR_SendRGBCW, "qqq", NULL);
}



