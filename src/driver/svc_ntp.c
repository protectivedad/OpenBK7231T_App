// NTP client
// Based on my previous work here:
// https://www.elektroda.pl/rtvforum/topic3712112.html
#include "../obk_config.h"

#include "../new_common.h"

#if ENABLE_NTP
//#include <time.h>

#include "drv_local.h"
#include "drv_public.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"
#include "../logging/logging.h"
#include "../hal/hal_ota.h"
#include "../libraries/obktime/obktime.h"	// for time functions
#include "lwipopts.h"
#include "lwip/ip_addr.h"
#include "lwip/apps/sntp.h"
#if PLATFORM_BEKEN
#include <tcpip.h>
#else
// these won't exist except on Beken?
#define LOCK_TCPIP_CORE()
#define UNLOCK_TCPIP_CORE()
#endif

#define LOG_FEATURE LOG_FEATURE_NTP

//Set custom NTP server
commandResult_t NTP_SetServer(const void *context, const char *cmd, const char *args, int cmdFlags) {
    const char *newValue;

    Tokenizer_TokenizeString(args,0);
	// following check must be done after 'Tokenizer_TokenizeString',
	// so we know arguments count in Tokenizer. 'cmd' argument is
	// only for warning display
	if (Tokenizer_CheckArgsCountAndPrintWarning(cmd, 1)) {
		return CMD_RES_NOT_ENOUGH_ARGUMENTS;
	}
    newValue = Tokenizer_GetArg(0);
	ip4_addr_t newAddr;
	if (!inet_aton(newValue, &newAddr)) {
		ADDLOGF_ERROR("%s - invalid ntp server IP: %s", __func__, newValue);
		return CMD_RES_BAD_ARGUMENT;
	}
    ADDLOGF_INFO("NTP server set to %s", newValue);
	
	if (newAddr.addr == ip_addr_any.addr)
		newValue = "";

	sntp_setserver(0, &newAddr);
    CFG_SetNTPServer(newValue);
    return CMD_RES_OK;
}

void NTP_Init() {
	//cmddetail:{"name":"ntp_setServer","args":"[ServerIP]",
	//cmddetail:"descr":"Sets the NTP server",
	//cmddetail:"fn":"NTP_SetServer","file":"driver/drv_ntp.c","requires":"",
	//cmddetail:"examples":""}
    CMD_RegisterCommand("ntp_setServer", NTP_SetServer, NULL);
    
    ADDLOGF_INFO("NTP driver initialized with server=%s, syncing every %i seconds", CFG_GetNTPServer(), SNTP_UPDATE_DELAY / 1000);
}

bool NTP_enabled() {
	return sntp_enabled();
}

void NTP_appendHTML(http_request_t* request, int bPreState) {
	if (bPreState)
		return;

	const ip_addr_t *ntp_ip = sntp_getserver(0);
	const char *ntp_addr = inet_ntoa(ntp_ip->addr);	
    if (sntp_enabled())
        hprintf255(request, "<h5>NTP: Syncing with %s every %i seconds</h5>", ntp_addr, SNTP_UPDATE_DELAY / 1000);
}

// framework request function
uint32_t NTP_frameworkRequest(uint32_t obkfRequest, uint32_t arg) {
	switch (obkfRequest) {
	case OBKF_Stop:
		sntp_stop();
		break;
		
	case OBKF_Init:
		NTP_Init();
		LOCK_TCPIP_CORE();
		sntp_setoperatingmode(SNTP_OPMODE_POLL);
		const char *adrString = CFG_GetNTPServer();
		if (adrString == 0 || adrString[0] == 0) {
			sntp_servermode_dhcp(true);
			adrString = DEFAULT_NTP_SERVER;
		} else {
			sntp_servermode_dhcp(false); // should be default, but just in case
		}
		ip4_addr_t addr;
		if (inet_aton(adrString, &addr))
			sntp_setserver(0, &addr);
		else
			ADDLOGF_ERROR("%s - failed to set %s ntp server!", __func__, adrString);

		UNLOCK_TCPIP_CORE();
		break;

	case OBKF_OnConnect:
		// safe to rerun
		LOCK_TCPIP_CORE();
		sntp_init();
		UNLOCK_TCPIP_CORE();
		break;

	default:
		break;
	}

	return true;
}

#endif // #if ENABLE_NTP
