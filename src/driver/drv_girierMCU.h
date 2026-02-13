#pragma once

#include <stdint.h>
#include <stdbool.h>

void GirierMCU_Init();
void GirierMCU_RunFrame();
void GirierMCU_Shutdown();
void GirierMCU_OnChannelChanged(uint32_t channelIndex, int32_t iVal);
bool GirierMCU_IsChannelUsedByGirierMCU(uint32_t channelIndex);
void GirierMCU_ForcePublishChannelValues();
