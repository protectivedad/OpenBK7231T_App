#pragma once

#include <stdint.h>
#include <stdbool.h>

//---------------------------------------------------
// Routines using UART port depending on config 
// flag OBK_FLAG_USE_SECONDARY_UART
// Note: using same buffer as routines below
//--------------------------------------------------
void UART_InitReceiveRingBuffer(uint32_t size);
int UART_GetReceiveRingBufferSize();
int UART_GetDataSize();
uint8_t UART_GetByte(uint32_t idx);
void UART_ConsumeBytes(uint32_t idx);
void UART_AppendByteToReceiveRingBuffer(uint8_t rc);
void UART_SendByte(uint8_t b);
int UART_InitUART(uint32_t baud, uint32_t parity, bool hwflowc);
void UART_AddCommands();
void UART_RunEverySecond();

// used to detect uart reinit/takeover by driver
int get_g_uart_init_counter();
// used to get selected port from config - OBK_FLAG_USE_SECONDARY_UART
int UART_GetSelectedPortIndex();

//---------------------------------------------------
// XJIKKA 20241123 new routines with uart index param 
// BEKEN platform only (yet)
// Independent of OBK_FLAG_USE_SECONDARY_UART 
//---------------------------------------------------
//index of UART port 
//	BEKEN - UART_PORT_INDEX_0 = BK_UART_1 RX1 TX1
//	BEKEN - UART_PORT_INDEX_1 = BK_UART_2 RX2 TX2
#define UART_PORT_INDEX_0 0
#define UART_PORT_INDEX_1 1
//
int UART_GetBufIndexFromPort(int aport);
void UART_InitReceiveRingBufferEx(uint32_t auartindex, uint32_t size);
void UART_AppendByteToReceiveRingBufferEx(uint32_t auartindex, uint8_t rc);
int UART_GetReceiveRingBufferSizeEx(uint32_t auartindex);
int UART_GetDataSizeEx(uint32_t auartindex);
uint8_t UART_GetByteEx(uint32_t auartindex, uint32_t idx);
void UART_ConsumeBytesEx(uint32_t auartindex, uint32_t idx);
void UART_SendByteEx(uint32_t auartindex, uint8_t b);
int UART_InitUARTEx(uint32_t auartindex, uint32_t baud, uint32_t parity, bool hwflowc);
void UART_LogBufState(uint32_t auartindex);

