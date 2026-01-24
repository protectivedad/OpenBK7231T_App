#ifndef __HALK_FLASH_VARS_H__
#define __HALK_FLASH_VARS_H__

//#define DISABLE_FLASH_VARS_VARS
#include "../new_common.h"

#define BOOT_COMPLETE_SECONDS 30
#define MAX_RETAIN_CHANNELS OBK_MAX_RETAIN_CHANNELS

#ifdef ENABLE_DRIVER_LED
typedef struct led_data
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t unused;
	uint8_t brightness;
	uint8_t temperature;
	uint8_t mode;
	uint8_t bEnableAll;
} LED_DATA;
#endif // ENABLE_DRIVER_LED

#ifdef ENABLE_POWERMETERING
/* Fixed size 32 bytes */
typedef struct ENERGY_METERING_DATA {
	float TotalConsumption;
	float TodayConsumpion;
	float YesterdayConsumption;
	long save_counter;
#if ENABLE_BL_TWIN
	float TotalConsumption_b;
	float TodayConsumpion_b;
#else
	float ConsumptionHistory[2];
#endif
#ifdef PLATFORM_BEKEN_NEW
	unsigned int ConsumptionResetTime;
#else
	time_t ConsumptionResetTime;
#endif
	unsigned char reseved[3];
	char actual_mday;
} ENERGY_METERING_DATA;

// size 8 bytes
typedef struct {
	float Import;
	float Export;
} ENERGY_DATA;

typedef enum {
	ENERGY_CHANNEL_A = 0,
	ENERGY_CHANNEL_B = 1,
} ENERGY_CHANNEL;
#endif // ENABLE_POWERMETERING

#define FLASH_VARS_MAGIC_V0 0xfefefefe

// save at least 32 bytes for header
// this maybe more efficient for flash writes ???
#define FLASH_VARS_MAGIC 'VKBO'
#define FLASH_VARS_VERS 1
typedef struct flash_vars_header {
	uint32_t magic;
	uint32_t version;
	uint8_t vars_len;
	uint8_t number_ch;
	uint8_t emetering_offset;
	uint8_t led_offset;
	uint32_t unused[4];
	// flash_vars_header.footer = flash_vars_structure.footer = flash_vars_length.footer 
	uint32_t footer;
} FLASH_VARS_HEADER;

#define FLASH_VARS_FOOTER 0xA932A932
typedef struct flash_vars_structure {
	uint16_t boot_count; // number of times the device has booted
	uint16_t boot_success_count; // if a device boots completely (>30s), will equal boot_success_count
	// start of savedValues needs to be consistent for version
	int16_t savedValues[MAX_RETAIN_CHANNELS];
#ifdef ENABLE_POWERMETERING
	ENERGY_METERING_DATA emetering;
#endif // ENABLE_POWERMETERING
#ifdef ENABLE_DRIVER_LED
	LED_DATA led;
#endif // ENABLE_DRIVER_LED
	// flash_vars_header.footer = flash_vars_structure.footer = flash_vars_length.footer 
	uint32_t footer;
} FLASH_VARS_STRUCTURE;

// call at startup
void HAL_FlashVars_IncreaseBootCount();
// call once started (>30s?)
void HAL_FlashVars_SaveBootComplete();
// call to return the number of boots since a HAL_FlashVars_SaveBootComplete
int HAL_FlashVars_GetBootFailures();
int HAL_FlashVars_GetBootCount();
void HAL_FlashVars_SaveChannel(int index, int value);
#ifdef ENABLE_DRIVER_LED
void HAL_FlashVars_SaveLED(byte mode, short brightness, short temperature, byte r, byte g, byte b, byte bEnableAll);
void HAL_FlashVars_ReadLED(byte* mode, short* brightness, short* temperature, byte* rgb, byte* bEnableAll);
#endif // ENABLE_DRIVER_LED
int HAL_FlashVars_GetChannelValue(int ch);
#ifdef ENABLE_POWERMETERING
int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA* data);
int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA* data);
void HAL_FlashVars_SaveTotalConsumption(float total_consumption);
void HAL_FlashVars_SaveEnergyExport(float f);
float HAL_FlashVars_GetEnergyExport();

#ifdef ENABLE_DRIVER_HLW8112SPI
void HAL_FlashVars_SaveEnergy(ENERGY_DATA** data, int channel_count);
void HAL_FlashVars_GetEnergy(ENERGY_DATA* data, ENERGY_CHANNEL channel);
#endif
#endif // ENABLE_POWERMETERING

#endif /* __HALK_FLASH_VARS_H__ */
