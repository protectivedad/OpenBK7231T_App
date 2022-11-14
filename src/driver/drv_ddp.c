

#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../logging/logging.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"

static const char* group = "239.255.250.250";
static int port = 4048;
static int g_ddp_socket_receive = -1;

void DRV_DDP_CreateSocket_Receive() {

    struct sockaddr_in addr;
    struct ip_mreq mreq;
    int flag = 1;
	int broadcast = 0;
	int iResult = 1;

    // create what looks like an ordinary UDP socket
    //
    g_ddp_socket_receive = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_ddp_socket_receive < 0) {
		g_ddp_socket_receive = -1;
		ADDLOG_INFO(LOG_FEATURE_DDP,"failed to do socket\n");
        return ;
    }

	if(broadcast)
	{

		iResult = setsockopt(g_ddp_socket_receive, SOL_SOCKET, SO_BROADCAST, (char *)&flag, sizeof(flag));
		if (iResult != 0)
		{
			ADDLOG_INFO(LOG_FEATURE_DDP,"failed to do setsockopt SO_BROADCAST\n");
			close(g_ddp_socket_receive);
			g_ddp_socket_receive = -1;
			return ;
		}
	}
	else{
		// allow multiple sockets to use the same PORT number
		//
		if (
			setsockopt(
				g_ddp_socket_receive, SOL_SOCKET, SO_REUSEADDR, (char*) &flag, sizeof(flag)
			) < 0
		){
			ADDLOG_INFO(LOG_FEATURE_DDP,"failed to do setsockopt SO_REUSEADDR\n");
			close(g_ddp_socket_receive);
			g_ddp_socket_receive = -1;
		  return ;
		}
	}

        // set up destination address
    //
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
    addr.sin_port = htons(port);

    // bind to receive address
    //
    if (bind(g_ddp_socket_receive, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
		ADDLOG_INFO(LOG_FEATURE_DDP,"failed to do bind\n");
		close(g_ddp_socket_receive);
		g_ddp_socket_receive = -1;
        return ;
    }

    //if(0 != setsockopt(g_ddp_socket_receive,SOL_SOCKET,SO_BROADCAST,(const char*)&flag,sizeof(int))) {
    //    return 1;
    //}

	if(broadcast)
	{

	}
	else
	{

	  // use setsockopt() to request that the kernel join a multicast group
		//
		mreq.imr_multiaddr.s_addr = inet_addr(group);
		//mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	mreq.imr_interface.s_addr = htonl(INADDR_ANY);//inet_addr(MY_CAPTURE_IP);
    	///mreq.imr_interface.s_addr = inet_addr("192.168.0.122");
	iResult = setsockopt(
				g_ddp_socket_receive, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
			);
		if (
			iResult < 0
		){
			ADDLOG_INFO(LOG_FEATURE_DDP,"failed to do setsockopt IP_ADD_MEMBERSHIP %i\n",iResult);
			close(g_ddp_socket_receive);
			g_ddp_socket_receive = -1;
			return ;
		}
	}

	lwip_fcntl(g_ddp_socket_receive, F_SETFL,O_NONBLOCK);

	ADDLOG_INFO(LOG_FEATURE_DDP,"Waiting for packets\n");
}
void DDP_Parse(byte *data, int len) {
	if(len > 12) {
		byte r, g, b;
		r = data[10];
		g = data[11];
		b = data[12];

		LED_SetFinalRGB(r,g,b);
	}
}
void DRV_DDP_RunFrame() {
    char msgbuf[64];
	struct sockaddr_in addr;

	if(g_ddp_socket_receive<0) {
		ADDLOG_INFO(LOG_FEATURE_DDP,"no sock\n");
            return ;
        }
    // now just enter a read-print loop
    //
        socklen_t addrlen = sizeof(addr);
        int nbytes = recvfrom(
            g_ddp_socket_receive,
            msgbuf,
            sizeof(msgbuf),
            0,
            (struct sockaddr *) &addr,
            &addrlen
        );
        if (nbytes <= 0) {
			//ADDLOG_INFO(LOG_FEATURE_DDP,"nothing\n");
            return ;
        }
		//ADDLOG_INFO(LOG_FEATURE_DDP,"Received %i bytes from %s\n",nbytes,inet_ntoa(((struct sockaddr_in *)&addr)->sin_addr));
        msgbuf[nbytes] = '\0';


		DDP_Parse((byte*)msgbuf, nbytes);

}
void DRV_DDP_Shutdown()
{
	if(g_ddp_socket_receive>=0) {
		close(g_ddp_socket_receive);
		g_ddp_socket_receive = -1;
	}
}

void DRV_DDP_Init()
{
	DRV_DDP_CreateSocket_Receive();
}



